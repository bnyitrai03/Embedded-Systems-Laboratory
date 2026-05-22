#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "control_loop.h"
#include "controller_adapter.h"
#include "homing.h"
#include "rpi_spi_master.h"

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

static void print_usage(const char *program)
{
    fprintf(stderr,
            "Usage: %s [spi_speed_hz] [home_pwm] [--hold]\n"
            "Default: speed=%u Hz, home_pwm=1500 of %u\n",
            program,
            RPI_SPI_DEFAULT_SPEED_HZ,
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
    ControlTarget hold_target = {0.0, 0.0, 0.0};
    unsigned home_pwm;
    int run_hold_loop = 0;
    int result;

    if (argc > 4) {
        print_usage(argv[0]);
        return 2;
    }

    if (argc > 3) {
        if (strcmp(argv[3], "--hold") != 0) {
            print_usage(argv[0]);
            return 2;
        }
        run_hold_loop = 1;
    }

    speed_hz = parse_unsigned_arg(argc > 1 ? argv[1] : 0, speed_hz);
    home_pwm = parse_unsigned_arg(argc > 2 ? argv[2] : 0, homing_config.pwm);
    if (home_pwm > MOTOR_PWM_MAX) {
        home_pwm = MOTOR_PWM_MAX;
    }
    homing_config.pwm = (uint16_t)home_pwm;

    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);

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

    controller_20sim_init(0.01);

    printf("Starting homing over SPI channel %u at %u Hz\n",
           RPI_SPI_DEFAULT_CHANNEL,
           speed_hz);
    printf("TX protocol: yaw[dir enable pwm14], then pitch[dir enable pwm14]\n");
    printf("RX protocol: signed int16 yaw encoder, then signed int16 pitch encoder\n");

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
        printf("Starting fixed target control loop at yaw=0 rad, pitch=0 rad\n");
        result = control_loop_run(&comm,
                                  &calibration,
                                  &control_config,
                                  control_loop_fixed_target,
                                  &hold_target,
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
