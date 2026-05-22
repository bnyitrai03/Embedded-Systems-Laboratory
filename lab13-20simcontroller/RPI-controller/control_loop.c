#include "control_loop.h"

#include <stdio.h>
#include <time.h>

ControlLoopConfig control_loop_default_config(void)
{
    ControlLoopConfig config;

    config.sample_period_us = 10000;
    config.log_period_samples = 100;
    return config;
}

static void sleep_us(unsigned usec)
{
    struct timespec request;

    request.tv_sec = (time_t)(usec / 1000000u);
    request.tv_nsec = (long)(usec % 1000000u) * 1000L;
    nanosleep(&request, 0);
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
    int result;

    while (*keep_running) {
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

        output = controller_20sim_step(calibration,
                                       encoders,
                                       target.yaw_target_rad,
                                       target.pitch_target_rad,
                                       target.pitch_correction_rad);
        command = controller_output_to_command(output);

        if (config->log_period_samples != 0 &&
            sample_index % config->log_period_samples == 0) {
            printf("enc yaw=%d pitch=%d target yaw=%.4f pitch=%.4f out yaw=%.3f pitch=%.3f\n",
                   encoders.yaw,
                   encoders.pitch,
                   target.yaw_target_rad,
                   target.pitch_target_rad,
                   output.yaw,
                   output.pitch);
        }

        ++sample_index;
        sleep_us(config->sample_period_us);
    }

    return motor_comm_exchange(comm, protocol_stop_command(), 0);
}
