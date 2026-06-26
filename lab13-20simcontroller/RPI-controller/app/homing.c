#include "homing.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

typedef enum
{
    AXIS_YAW,
    AXIS_PITCH
} HomeAxis;

HomingConfig homing_default_config(void)
{
    HomingConfig config;

    config.pwm = JIWY_HOMING_PWM;
    config.yaw_home_direction = JIWY_HOMING_YAW_HOME_DIRECTION;
    config.pitch_home_direction = JIWY_HOMING_PITCH_HOME_DIRECTION;
    config.sample_period_us = JIWY_HOMING_SAMPLE_PERIOD_US;
    config.stop_window_samples = JIWY_HOMING_STOP_WINDOW_SAMPLES;
    config.movement_threshold_counts = JIWY_HOMING_MOVEMENT_THRESHOLD_COUNTS;
    config.max_samples_per_axis = JIWY_HOMING_MAX_SAMPLES_PER_AXIS;
    config.settle_samples = JIWY_HOMING_SETTLE_SAMPLES;
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

static MotorDirection opposite_direction(MotorDirection direction)
{
    return (direction == MOTOR_DIR_POSITIVE) ? MOTOR_DIR_NEGATIVE : MOTOR_DIR_POSITIVE;
}

static MotorCommand home_command(HomeAxis axis, MotorDirection direction, uint16_t pwm)
{
    MotorCommand command = protocol_stop_command();
    AxisCommand axis_command;

    axis_command.direction = direction;
    axis_command.enable = 1;
    axis_command.pwm = pwm;

    if (axis == AXIS_YAW)
    {
        command.yaw = axis_command;
    }
    else
    {
        command.pitch = axis_command;
    }

    return command;
}

static int stop_and_settle(MotorComm *comm, const HomingConfig *config, volatile sig_atomic_t *keep_running)
{
    unsigned i;
    EncoderSample ignored;
    int result;

    /* Send several stop frames so the motor and sampled encoder count settle. */
    for (i = 0; i < config->settle_samples && *keep_running; ++i)
    {
        result = motor_comm_exchange(comm, protocol_stop_command(), &ignored);
        if (result < 0)
        {
            return result;
        }
        sleep_us(config->sample_period_us);
    }

    return 0;
}

static int drive_axis_until_stalled(MotorComm *comm, const HomingConfig *config, HomeAxis axis, MotorDirection direction, int16_t *stop_count, volatile sig_atomic_t *keep_running)
{
    EncoderSample sample;
    int result;
    int last_moving_count;
    unsigned stagnant_samples = 0;
    unsigned sample_index;

    result = motor_comm_exchange(comm, protocol_stop_command(), &sample);
    if (result < 0)
    {
        return result;
    }

    /*
     * The axis is considered moving only when the encoder changes by at least
     * movement_threshold_counts.
     */
    last_moving_count = axis_count(sample, axis);
    printf("Driving %s direction %u from encoder count %d\n", axis_name(axis), (unsigned)direction, last_moving_count);

    for (sample_index = 0; sample_index < config->max_samples_per_axis && *keep_running; ++sample_index)
    {
        result = motor_comm_exchange(comm, home_command(axis, direction, config->pwm), &sample);
        if (result < 0)
        {
            return result;
        }

        if (abs(axis_count(sample, axis) - last_moving_count) >= (int)config->movement_threshold_counts)
        {
            last_moving_count = axis_count(sample, axis);
            stagnant_samples = 0;
        }
        else
        {
            ++stagnant_samples;
        }

        /*
         * The mechanical stop is inferred from encoder stagnation.
         * Low PWM: this intentionally pushes into the end stop.
         */
        if (sample_index >= config->stop_window_samples && stagnant_samples >= config->stop_window_samples)
        {
            *stop_count = (int16_t)axis_count(sample, axis);
            printf("%s stop found at encoder count %d\n", axis_name(axis), *stop_count);
            return stop_and_settle(comm, config, keep_running);
        }

        sleep_us(config->sample_period_us);
    }

    result = stop_and_settle(comm, config, keep_running);
    if (result < 0)
    {
        return result;
    }
    return *keep_running ? -ETIMEDOUT : 0;
}

static unsigned count_span(int16_t first_count, int16_t second_count)
{
    int diff = (int)first_count - (int)second_count;
    return (unsigned)abs(diff);
}

static int measure_and_home_axis(MotorComm *comm, const HomingConfig *config, HomeAxis axis, MotorDirection home_direction, int16_t *home_count, unsigned *travel_counts, volatile sig_atomic_t *keep_running)
{
    int16_t opposite_stop_count;
    int16_t home_stop_count;
    int result;

    result = drive_axis_until_stalled(comm, config, axis, opposite_direction(home_direction), &opposite_stop_count, keep_running);
    if (result < 0)
    {
        return result;
    }

    result = drive_axis_until_stalled(comm, config, axis, home_direction, &home_stop_count, keep_running);
    if (result < 0)
    {
        return result;
    }

    *travel_counts = count_span(opposite_stop_count, home_stop_count);
    if (*travel_counts == 0u)
    {
        return -ERANGE;
    }

    *home_count = home_stop_count;
    printf("%s measured travel: %u counts, home=%d counts\n", axis_name(axis), *travel_counts, *home_count);
    return 0;
}

int homing_run(MotorComm *comm, const HomingConfig *config, JiwyCalibration *calibration, volatile sig_atomic_t *keep_running)
{
    int result;
    unsigned yaw_travel_counts;
    unsigned pitch_travel_counts;

    /* Measure and home axes one at a time so only one motor pushes a stop. */
    result = measure_and_home_axis(comm, config, AXIS_YAW, config->yaw_home_direction, &calibration->yaw_home_count, &yaw_travel_counts, keep_running);
    if (result < 0)
    {
        return result;
    }
    jiwy_set_yaw_travel_counts(calibration, yaw_travel_counts);

    result = measure_and_home_axis(comm, config, AXIS_PITCH, config->pitch_home_direction, &calibration->pitch_home_count, &pitch_travel_counts, keep_running);
    if (result < 0)
    {
        return result;
    }
    jiwy_set_pitch_travel_counts(calibration, pitch_travel_counts);

    return motor_comm_exchange(comm, protocol_stop_command(), 0);
}