#include "control_loop.h"

#include <errno.h>
#include <stdio.h>
#include <time.h>

ControlLoopConfig control_loop_default_config(void)
{
    ControlLoopConfig config;

    config.sample_period_us = 10000;
    config.log_period_samples = 100;
    return config;
}

static struct timespec timespec_add_us(struct timespec time,
                                       unsigned usec)
{
    time.tv_sec += (time_t)(usec / 1000000u);
    time.tv_nsec += (long)(usec % 1000000u) * 1000L;

    while (time.tv_nsec >= 1000000000L) {
        time.tv_nsec -= 1000000000L;
        ++time.tv_sec;
    }

    return time;
}

static long timespec_diff_us(struct timespec end, struct timespec start)
{
    time_t sec = end.tv_sec - start.tv_sec;
    long nsec = end.tv_nsec - start.tv_nsec;

    return (long)sec * 1000000L + nsec / 1000L;
}

static int sleep_until(struct timespec deadline)
{
    int result;

    do {
        result = clock_nanosleep(CLOCK_MONOTONIC,
                                 TIMER_ABSTIME,
                                 &deadline,
                                 0);
    } while (result == EINTR);

    return result;
}

int control_loop_fixed_target(void *context,
                              const JiwyCalibration *calibration,
                              EncoderSample encoders,
                              ControlTarget *target)
{
    const ControlTarget *fixed_target = (const ControlTarget *)context;

    (void)calibration;
    (void)encoders;

    *target = *fixed_target;
    return 0;
}

int control_loop_run(MotorComm *comm,
                     TwentySimController *controller,
                     const JiwyCalibration *calibration,
                     const ControlLoopConfig *config,
                     ControlTargetProvider target_provider,
                     void *target_context,
                     volatile sig_atomic_t *keep_running)
{
    EncoderSample encoders;
    ControlTarget target;
    ControllerOutput output;
    MotorCommand command = protocol_stop_command();
    unsigned sample_index = 0;
    unsigned overrun_count = 0;
    struct timespec next_deadline;
    int result;

    result = clock_gettime(CLOCK_MONOTONIC, &next_deadline);
    if (result < 0) {
        return -errno;
    }

    while (*keep_running) {
        struct timespec work_start;
        struct timespec work_end;
        long work_us;
        long lateness_us;

        next_deadline = timespec_add_us(next_deadline,
                                        config->sample_period_us);

        result = clock_gettime(CLOCK_MONOTONIC, &work_start);
        if (result < 0) {
            return -errno;
        }

        /*
         * SPI is full duplex. This exchange sends the command computed during
         * the previous sample while receiving the encoder sample used for the
         * next command. That creates one sample of command delay but avoids a
         * separate "read encoders" transaction.
         */
        result = motor_comm_exchange(comm, command, &encoders);
        if (result < 0) {
            return result;
        }

        result = target_provider(target_context, calibration, encoders, &target);
        if (result < 0) {
            return result;
        }

        output = twentysim_controller_step(controller,
                                           calibration,
                                           encoders,
                                           target.yaw_target_rad,
                                           target.pitch_target_rad);
        command = controller_output_to_command(output);

        result = clock_gettime(CLOCK_MONOTONIC, &work_end);
        if (result < 0) {
            return -errno;
        }

        work_us = timespec_diff_us(work_end, work_start);
        lateness_us = timespec_diff_us(work_end, next_deadline);
        if (lateness_us > 0) {
            ++overrun_count;
        }

        if (config->log_period_samples != 0 &&
            sample_index % config->log_period_samples == 0) {
            printf("enc yaw=%d pitch=%d target yaw=%.4f pitch=%.4f out yaw=%.3f pitch=%.3f work=%ldus late=%ldus overruns=%u\n",
                   encoders.yaw,
                   encoders.pitch,
                   target.yaw_target_rad,
                   target.pitch_target_rad,
                   output.yaw,
                   output.pitch,
                   work_us,
                   lateness_us > 0 ? lateness_us : 0,
                   overrun_count);
        }

        ++sample_index;
        if (lateness_us <= 0) {
            result = sleep_until(next_deadline);
            if (result != 0) {
                return -result;
            }
        }
    }

    return motor_comm_exchange(comm, protocol_stop_command(), 0);
}
