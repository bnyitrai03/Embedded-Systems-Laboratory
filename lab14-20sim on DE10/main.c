#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "control_loop.h"
#include "homing.h"
#include "jiwy_calibration.h"
#include "rpi_spi_master.h"
#include "twentysim_controller.h"

static volatile sig_atomic_t keep_running = 1;

static void handle_signal(int signal_number)
{
    (void)signal_number;
    keep_running = 0;
}

static unsigned parse_unsigned_arg(const char *text, unsigned fallback)
{
    char *end = 0;
    unsigned long value;

    if (text == 0) {
        return fallback;
    }

    errno = 0;
    value = strtoul(text, &end, 10);
    if (errno != 0 || end == text || *end != '\0' || value > 0xFFFFFFFFul) {
        return fallback;
    }

    return (unsigned)value;
}

static int is_option_arg(const char *text)
{
    return text != 0 && text[0] == '-' && text[1] == '-';
}

static void print_usage(const char *program)
{
    fprintf(stderr,
            "Usage: %s [spi_speed_hz] [home_pwm] [--hold] [--log csv_path]\n"
            "Default: speed=%u Hz, home_pwm=%u of %u\n"
            "When --hold is used without --log, CSV is written to pid_log.csv.\n",
            program,
            RPI_SPI_DEFAULT_SPEED_HZ,
            MOTOR_PWM_MAX,
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
    ControlTarget hold_target = {0.0, 0.0};
    
    const char *csv_log_path = 0;
    unsigned home_pwm;
    int run_hold_loop = 0;
    int result;
    int arg_index = 1;

    if (arg_index < argc && !is_option_arg(argv[arg_index])) {
        speed_hz = parse_unsigned_arg(argv[arg_index], speed_hz);
        ++arg_index;
    }

    if (arg_index < argc && !is_option_arg(argv[arg_index])) {
        home_pwm = parse_unsigned_arg(argv[arg_index], homing_config.pwm);
        ++arg_index;
    } else {
        home_pwm = homing_config.pwm;
    }

    for (; arg_index < argc; ++arg_index) {
        if (strcmp(argv[arg_index], "--hold") == 0) {
            run_hold_loop = 1;
        } else if (strcmp(argv[arg_index], "--log") == 0 && arg_index + 1 < argc) {
            csv_log_path = argv[++arg_index];
        } else {
            print_usage(argv[0]);
            return 2;
        }
    }

    if (home_pwm > MOTOR_PWM_MAX) {
        home_pwm = MOTOR_PWM_MAX;
    }
    homing_config.pwm = (uint16_t)home_pwm;
    if (run_hold_loop && csv_log_path == 0) {
        csv_log_path = "pid_log.csv";
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
                               0);
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

    if (run_hold_loop) {
        EncoderSample initial_encoders;
        TwentySimController controller;
        double controller_step_size_s =
            (double)control_config.sample_period_us / 1000000.0;
        hold_target.yaw_target_rad = 0.5 * (calibration.yaw_min_rad + calibration.yaw_max_rad);
        hold_target.pitch_target_rad = 0.5 * (calibration.pitch_min_rad + calibration.pitch_max_rad);
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

        twentysim_controller_init(&controller,
                                  controller_step_size_s,
                                  &calibration,
                                  initial_encoders,
                                  hold_target.yaw_target_rad,
                                  hold_target.pitch_target_rad);

        /* Vision can later replace this fixed target with live target updates. */
        printf("Starting fixed target control loop at yaw=0 rad, pitch=0 rad\n");
        if (control_config.csv_log_path != 0) {
            printf("Writing control-loop CSV log to %s\n",
                   control_config.csv_log_path);
        }
        result = control_loop_run(&comm,
                                  &controller,
                                  &calibration,
                                  &control_config,
                                  hold_target,
                                  &keep_running);
        motor_comm_exchange(&comm, protocol_stop_command(), 0);

        if (result < 0) {
            fprintf(stderr, "Control loop failed: %s\n", strerror(-result));
            motor_comm_close(&comm);
            return 1;
        }
    }

    motor_comm_close(&comm);
    return 0;
}
