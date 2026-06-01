#include "control_loop.h"

#include <errno.h>
#include <stdio.h>
#include <time.h>

ControlLoopConfig control_loop_default_config(void)
{
    ControlLoopConfig config;

    config.sample_period_us = 10000;
    config.log_period_samples = 0;
    config.csv_log_path = 0;
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

    /*
     * Raspberry Pi/Linux absolute sleep. TIMER_ABSTIME keeps the loop anchored
     * to the original schedule instead of adding work time to every period.
     */
    do {
        result = clock_nanosleep(CLOCK_MONOTONIC,
                                 TIMER_ABSTIME,
                                 &deadline,
                                 0);
    } while (result == EINTR);

    return result;
}

int control_loop_run(MotorComm *comm,
                     TwentySimController *controller,
                     const JiwyCalibration *calibration,
                     const ControlLoopConfig *config,
                     ControlTarget target,
                     VisionTracker *vision_tracker,
                     volatile sig_atomic_t *keep_running)
{
    EncoderSample encoders;
    ControllerOutput output;
    MotorCommand command = protocol_stop_command();
    FILE *csv_log = 0;
    unsigned sample_index = 0;
    unsigned overrun_count = 0;
    struct timespec next_deadline;
    struct timespec loop_start;
    int result;

    result = clock_gettime(CLOCK_MONOTONIC, &next_deadline);
    if (result < 0) {
        return -errno;
    }
    loop_start = next_deadline;

    if (config->csv_log_path != 0) {
        csv_log = fopen(config->csv_log_path, "w");
        if (csv_log == 0) {
            return -errno;
        }

        fprintf(csv_log,
                "sample,time_s,"
                "yaw_encoder,pitch_encoder,"
                "yaw_target_rad,pitch_target_rad,"
                "yaw_actual_rad,pitch_actual_rad,"
                "yaw_error_rad,pitch_error_rad,"
                "yaw_output,pitch_output,"
                "yaw_pwm,pitch_pwm,"
                "yaw_dir,pitch_dir,"
                "work_us,lateness_us,overruns\n");
    }

    /*
     * next_deadline is advanced before each sample. That means sample N targets
     * start_time + N * period, independent of how long previous samples took.
     */
    while (*keep_running) {
        struct timespec work_start;
        struct timespec work_end;
        double yaw_actual_rad;
        double pitch_actual_rad;
        VisionTargetSnapshot vision_snapshot;
        long work_us;
        long lateness_us;

        next_deadline = timespec_add_us(next_deadline,
                                        config->sample_period_us);

        result = clock_gettime(CLOCK_MONOTONIC, &work_start);
        if (result < 0) {
            if (csv_log != 0) {
                fclose(csv_log);
            }
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
            if (csv_log != 0) {
                fclose(csv_log);
            }
            return result;
        }

        yaw_actual_rad = jiwy_yaw_rad(calibration, encoders.yaw);
        pitch_actual_rad = jiwy_pitch_rad(calibration, encoders.pitch);

        if (vision_tracker != 0 &&
            vision_tracker_read_latest(vision_tracker, &vision_snapshot) == 0) {
            if (vision_snapshot.valid) {
                target.yaw_target_rad =
                    yaw_actual_rad + vision_snapshot.yaw_error_rad;
                target.pitch_target_rad =
                    pitch_actual_rad + vision_snapshot.pitch_error_rad;
            } else {
                target.yaw_target_rad = yaw_actual_rad;
                target.pitch_target_rad = pitch_actual_rad;
            }
        }

        output = twentysim_controller_step(controller,
                                           calibration,
                                           encoders,
                                           target.yaw_target_rad,
                                           target.pitch_target_rad);
        command = controller_output_to_command(output);

        result = clock_gettime(CLOCK_MONOTONIC, &work_end);
        if (result < 0) {
            if (csv_log != 0) {
                fclose(csv_log);
            }
            return -errno;
        }

        work_us = timespec_diff_us(work_end, work_start);
        lateness_us = timespec_diff_us(work_end, next_deadline);
        if (lateness_us > 0) {
            ++overrun_count;
        }

        if (config->log_period_samples != 0 &&
            sample_index % config->log_period_samples == 0) {
            printf("enc yaw=%d pitch=%d target yaw=%.4f pitch=%.4f actual yaw=%.4f pitch=%.4f work=%ldus late=%ldus overruns=%u\n",
                   encoders.yaw,
                   encoders.pitch,
                   target.yaw_target_rad,
                   target.pitch_target_rad,
                   yaw_actual_rad,
                   pitch_actual_rad,
                   work_us,
                   lateness_us > 0 ? lateness_us : 0,
                   overrun_count);
        }

        if (csv_log != 0) {
            fprintf(csv_log,
                    "%u,%.6f,"
                    "%d,%d,"
                    "%.9f,%.9f,"
                    "%.9f,%.9f,"
                    "%.9f,%.9f,"
                    "%.9f,%.9f,"
                    "%u,%u,"
                    "%u,%u,"
                    "%ld,%ld,%u\n",
                    sample_index,
                    (double)timespec_diff_us(work_start, loop_start) / 1000000.0,
                    encoders.yaw,
                    encoders.pitch,
                    target.yaw_target_rad,
                    target.pitch_target_rad,
                    yaw_actual_rad,
                    pitch_actual_rad,
                    target.yaw_target_rad - yaw_actual_rad,
                    target.pitch_target_rad - pitch_actual_rad,
                    output.yaw,
                    output.pitch,
                    command.yaw.pwm,
                    command.pitch.pwm,
                    command.yaw.direction,
                    command.pitch.direction,
                    work_us,
                    lateness_us > 0 ? lateness_us : 0,
                    overrun_count);
        }

        ++sample_index;
        if (lateness_us > 0) {
            /*
             * If the sample has already missed its deadline, skip sleeping.
             * The next iteration advances to the next absolute deadline and
             * tries to recover without accumulating drift.
             */
            continue;
        }

        result = sleep_until(next_deadline);
        if (result != 0) {
            if (csv_log != 0) {
                fclose(csv_log);
            }
            return -result;
        }
    }

    if (csv_log != 0) {
        fclose(csv_log);
    }

    return motor_comm_exchange(comm, protocol_stop_command(), 0);
}
