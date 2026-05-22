#include "homing.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

typedef enum {
    AXIS_YAW,
    AXIS_PITCH
} HomeAxis;

HomingConfig homing_default_config(void)
{
    HomingConfig config;

    config.pwm = 1500;
    config.yaw_home_direction = MOTOR_DIR_NEGATIVE;
    config.pitch_home_direction = MOTOR_DIR_NEGATIVE;
    config.sample_period_us = 10000;
    config.stop_window_samples = 50;
    config.movement_threshold_counts = 2;
    config.max_samples_per_axis = 1000;
    config.settle_samples = 20;
    return config;
}

static void sleep_us(unsigned usec)
{
    struct timespec request;

    request.tv_sec = (time_t)(usec / 1000000u);
    request.tv_nsec = (long)(usec % 1000000u) * 1000L;
    nanosleep(&request, 0);
}

static int axis_count(EncoderSample sample, HomeAxis axis)
{
    return (axis == AXIS_YAW) ? sample.yaw : sample.pitch;
}

static const char *axis_name(HomeAxis axis)
{
    return (axis == AXIS_YAW) ? "yaw" : "pitch";
}

static MotorCommand home_command(HomeAxis axis,
                                 MotorDirection direction,
                                 uint16_t pwm)
{
    MotorCommand command = protocol_stop_command();
    AxisCommand axis_command;

    axis_command.direction = direction;
    axis_command.enable = 1;
    axis_command.pwm = pwm;

    if (axis == AXIS_YAW) {
        command.yaw = axis_command;
    } else {
        command.pitch = axis_command;
    }

    return command;
}

static int stop_and_settle(MotorComm *comm,
                           const HomingConfig *config,
                           volatile sig_atomic_t *keep_running)
{
    unsigned i;
    EncoderSample ignored;
    int result;

    for (i = 0; i < config->settle_samples && *keep_running; ++i) {
        result = motor_comm_exchange(comm, protocol_stop_command(), &ignored);
        if (result < 0) {
            return result;
        }
        sleep_us(config->sample_period_us);
    }

    return 0;
}

static int home_one_axis(MotorComm *comm,
                         const HomingConfig *config,
                         HomeAxis axis,
                         MotorDirection direction,
                         int16_t *home_count,
                         volatile sig_atomic_t *keep_running)
{
    EncoderSample sample;
    int result;
    int last_moving_count;
    unsigned stagnant_samples = 0;
    unsigned sample_index;

    result = motor_comm_exchange(comm, protocol_stop_command(), &sample);
    if (result < 0) {
        return result;
    }

    last_moving_count = axis_count(sample, axis);
    printf("Homing %s from encoder count %d\n", axis_name(axis), last_moving_count);

    for (sample_index = 0;
         sample_index < config->max_samples_per_axis && *keep_running;
         ++sample_index) {
        result = motor_comm_exchange(comm,
                                     home_command(axis, direction, config->pwm),
                                     &sample);
        if (result < 0) {
            return result;
        }

        if (abs(axis_count(sample, axis) - last_moving_count) >=
            (int)config->movement_threshold_counts) {
            last_moving_count = axis_count(sample, axis);
            stagnant_samples = 0;
        } else {
            ++stagnant_samples;
        }

        /*
         * With no limit switch, the mechanical stop is inferred from encoder
         * stagnation. Keep PWM low: this intentionally pushes into the end stop.
         */
        if (sample_index >= config->stop_window_samples &&
            stagnant_samples >= config->stop_window_samples) {
            *home_count = (int16_t)axis_count(sample, axis);
            printf("%s home found at encoder count %d\n",
                   axis_name(axis),
                   *home_count);
            return stop_and_settle(comm, config, keep_running);
        }

        sleep_us(config->sample_period_us);
    }

    result = stop_and_settle(comm, config, keep_running);
    if (result < 0) {
        return result;
    }
    return *keep_running ? -ETIMEDOUT : 0;
}

int homing_run(MotorComm *comm,
               const HomingConfig *config,
               JiwyCalibration *calibration,
               volatile sig_atomic_t *keep_running)
{
    int result;

    result = home_one_axis(comm,
                           config,
                           AXIS_YAW,
                           config->yaw_home_direction,
                           &calibration->yaw_home_count,
                           keep_running);
    if (result < 0) {
        return result;
    }

    result = home_one_axis(comm,
                           config,
                           AXIS_PITCH,
                           config->pitch_home_direction,
                           &calibration->pitch_home_count,
                           keep_running);
    if (result < 0) {
        return result;
    }

    return motor_comm_exchange(comm, protocol_stop_command(), 0);
}
