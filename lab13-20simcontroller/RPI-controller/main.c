#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>

#include "jiwy_config.h"
#include "app/include/control_loop.h"
#include "app/include/homing.h"
#include "control/include/jiwy_calibration.h"
#include "comm/include/rpi_spi_master.h"
#include "control/include/twentysim_controller.h"
#include "vision/include/vision_tracker.h"

static volatile sig_atomic_t keep_running = 1;

static void handle_signal(int signal_number)
{
    (void)signal_number;
    keep_running = 0;
}

static void print_usage(const char *program)
{
    fprintf(stderr,
            "Usage: %s [--hold|--track] [--log csv_path]\n"
            "Default config: SPI=%u Hz channel=%u, homing PWM=%u of %u\n"
            "--hold runs the configured two-step calibration schedule after homing.\n"
            "--track follows a green object using camera and streaming settings from jiwy_config.h.\n",
            program,
            RPI_SPI_DEFAULT_SPEED_HZ,
            RPI_SPI_DEFAULT_CHANNEL,
            JIWY_HOMING_PWM,
            MOTOR_PWM_MAX);
}

int main(int argc, char *argv[])
{
    unsigned speed_hz = RPI_SPI_DEFAULT_SPEED_HZ;
    HomingConfig homing_config = homing_default_config();
    JiwyCalibration calibration = jiwy_default_calibration();
    RpiSpiComm spi = {.fd = -1, .speed_hz = 0, .channel = 0};
    MotorComm comm;
    ControlLoopConfig control_config = control_loop_default_config();
    HoldSchedule hold_schedule = {
        {JIWY_HOLD_TARGET1_YAW_RAD, JIWY_HOLD_TARGET1_PITCH_RAD},
        JIWY_HOLD_TARGET1_DURATION_S,
        {JIWY_HOLD_TARGET2_YAW_RAD, JIWY_HOLD_TARGET2_PITCH_RAD},
        JIWY_HOLD_TARGET2_DURATION_S,
        JIWY_HOLD_SCHEDULE_REPEAT
    };
    VisionTracker vision_tracker;
    VisionTracker *active_vision_tracker = 0;
    const char *csv_log_path = JIWY_CONTROL_LOOP_DEFAULT_CSV_LOG_PATH;
    const char *camera_device = JIWY_VISION_DEFAULT_CAMERA;
    int run_hold_loop = 0;
    int run_track_loop = 0;
    int vision_debug_enabled = JIWY_VISION_DEBUG_ENABLED;
    int vision_stream_enabled = JIWY_VISION_STREAM_ENABLED;
    int vision_stream_port = JIWY_VISION_STREAM_PORT;
    int result;
    int arg_index = 1;

    for (; arg_index < argc; ++arg_index) {
        if (strcmp(argv[arg_index], "--hold") == 0) {
            run_hold_loop = 1;
        } else if (strcmp(argv[arg_index], "--track") == 0) {
            run_track_loop = 1;
        } else if (strcmp(argv[arg_index], "--log") == 0 && arg_index + 1 < argc) {
            csv_log_path = argv[++arg_index];
        } else if (strcmp(argv[arg_index], "--help") == 0) {
            print_usage(argv[0]);
            return 0;
        } else {
            print_usage(argv[0]);
            return 2;
        }
    }

    control_config.csv_log_path = csv_log_path;

    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);

    /*
     * All runtime behavior goes through MotorComm. Today that backend is
     * Raspberry Pi spidev, but homing and control_loop do not depend on spidev
     * details.
     */
    result = rpi_spi_comm_open(&spi,
                               &comm,
                               RPI_SPI_DEFAULT_CHANNEL,
                               speed_hz,
                               JIWY_SPI_DEFAULT_FLAGS);
    if (result < 0) {
        fprintf(stderr,
                "Failed to open /dev/spidev0.%u: %s\n",
                RPI_SPI_DEFAULT_CHANNEL,
                strerror(-result));
        return 1;
    }

    printf("Starting homing over SPI channel %u at %u Hz\n",
           RPI_SPI_DEFAULT_CHANNEL,
           speed_hz);
    printf("TX protocol: yaw[dir enable pwm14], then pitch[dir enable pwm14]\n");
    printf("RX protocol: signed int16 yaw encoder, then signed int16 pitch encoder\n");

    /*
     * Homing defines software zero. Do not initialize the generated 20-sim
     * controller before this point, because encoder feedback would still be in
     * raw hardware coordinates.
     */
    result = homing_run(&comm, &homing_config, &calibration, &keep_running);
    motor_comm_exchange(&comm, protocol_stop_command(), 0);

    if (result < 0) {
        fprintf(stderr, "Homing failed: %s\n", strerror(-result));
        motor_comm_close(&comm);
        return 1;
    }

    if (!keep_running) {
        printf("Homing interrupted; motors disabled\n");
        motor_comm_close(&comm);
        return 130;
    }

    printf("Homing complete\n");
    printf("Software home offsets: yaw=%d counts, pitch=%d counts\n",
           calibration.yaw_home_count,
           calibration.pitch_home_count);

    if (run_hold_loop || run_track_loop) {
        EncoderSample initial_encoders;
        TwentySimController controller;
        ControlTarget initial_target = hold_schedule.target1;
        double controller_step_size_s =
            (double)control_config.sample_period_us / 1000000.0;
        /*
         * Seed the generated controller with the first post-homing feedback
         * sample so its initial error is consistent with the software home.
         */
        result = motor_comm_exchange(&comm,
                                     protocol_stop_command(),
                                     &initial_encoders);
        if (result < 0) {
            fprintf(stderr,
                    "Failed to read initial encoders: %s\n",
                    strerror(-result));
            motor_comm_close(&comm);
            return 1;
        }

        if (run_track_loop) {
            initial_target.yaw_target_rad =
                (calibration.yaw_min_rad + calibration.yaw_max_rad) / 2.0;
            initial_target.pitch_target_rad =
                (calibration.pitch_min_rad + calibration.pitch_max_rad) / 2.0;

            vision_tracker_init(&vision_tracker);
            result = vision_tracker_start(&vision_tracker,
                                          camera_device,
                                          vision_debug_enabled,
                                          vision_stream_enabled,
                                          vision_stream_port);
            if (result < 0) {
                fprintf(stderr,
                        "Failed to start vision tracker on %s: %s\n",
                        camera_device,
                        strerror(-result));
                motor_comm_close(&comm);
                return 1;
            }
            active_vision_tracker = &vision_tracker;
        }

        twentysim_controller_init(&controller,
                                  controller_step_size_s,
                                  &calibration,
                                  initial_encoders,
                                  initial_target.yaw_target_rad,
                                  initial_target.pitch_target_rad);

        if (run_track_loop) {
            printf("Starting vision tracking control loop using %s\n",
                   camera_device);
        } else {
            printf("Starting hold calibration schedule:\n");
            printf("  target1 yaw=%.4f rad pitch=%.4f rad duration=%.2f s\n",
                   hold_schedule.target1.yaw_target_rad,
                   hold_schedule.target1.pitch_target_rad,
                   hold_schedule.target1_duration_s);
            printf("  target2 yaw=%.4f rad pitch=%.4f rad duration=%.2f s\n",
                   hold_schedule.target2.yaw_target_rad,
                   hold_schedule.target2.pitch_target_rad,
                   hold_schedule.target2_duration_s);
            printf("  repeat=%s\n",
                   hold_schedule.repeat ? "on" : "off");
        }
        if (control_config.csv_log_path != 0) {
            printf("Writing control-loop CSV log to %s\n",
                   control_config.csv_log_path);
        }
        result = control_loop_run(&comm,
                                  &controller,
                                  &calibration,
                                  &control_config,
                                  initial_target,
                                  run_hold_loop ? &hold_schedule : 0,
                                  active_vision_tracker,
                                  &keep_running);
        motor_comm_exchange(&comm, protocol_stop_command(), 0);
        vision_tracker_stop(active_vision_tracker);

        if (result < 0) {
            fprintf(stderr, "Control loop failed: %s\n", strerror(-result));
            motor_comm_close(&comm);
            return 1;
        }
    }

    motor_comm_close(&comm);
    return 0;
}
