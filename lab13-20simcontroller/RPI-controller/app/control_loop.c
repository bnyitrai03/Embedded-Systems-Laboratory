#include "control_loop.h"

#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include "jiwy_config.h"
ControlLoopConfig control_loop_default_config(void)
{
    ControlLoopConfig config;

    config.sample_period_us = JIWY_CONTROL_LOOP_SAMPLE_PERIOD_US;
    config.log_period_samples = JIWY_CONTROL_LOOP_STATUS_EVERY;
    config.csv_log_path = JIWY_CONTROL_LOOP_DEFAULT_CSV_LOG_PATH;
    return config;
}

static struct timespec timespec_add_us(struct timespec time, unsigned usec)
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

static ControlTarget hold_schedule_target_at(const HoldSchedule *schedule, double elapsed_s, int *phase_out)
{
    double cycle_duration_s;

    if (schedule == 0) {
        ControlTarget target = {0.0, 0.0};
        if (phase_out != 0) {
            *phase_out = 0;
        }
        return target;
    }

    if (!(schedule->target1_duration_s > 0.0)) {
        if (phase_out != 0) {
            *phase_out = 2;
        }
        return schedule->target2;
    }
    if (!(schedule->target2_duration_s > 0.0)) {
        if (phase_out != 0) {
            *phase_out = 1;
        }
        return schedule->target1;
    }

    cycle_duration_s = schedule->target1_duration_s + schedule->target2_duration_s;
    if (schedule->repeat && cycle_duration_s > 0.0) {
        elapsed_s = fmod(elapsed_s, cycle_duration_s);
    }

    if (elapsed_s < schedule->target1_duration_s) {
        if (phase_out != 0) {
            *phase_out = 1;
        }
        return schedule->target1;
    }
    if (elapsed_s < cycle_duration_s) {
        if (phase_out != 0) {
            *phase_out = 2;
        }
        return schedule->target2;
    }

    if (phase_out != 0) {
        *phase_out = 2;
    }
    return schedule->target2;
}

static int sleep_until(struct timespec deadline)
{
    int result;

    /*
     * Raspberry Pi absolute sleep. TIMER_ABSTIME keeps the loop anchored
     * to the original schedule instead of adding work time to every period.
     */
    do {
        result = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &deadline, 0);
    } while (result == EINTR);

    return result;
}

int control_loop_run(MotorComm *comm,
                     TwentySimController *controller,
                     const JiwyCalibration *calibration,
                     const ControlLoopConfig *config,
                     ControlTarget target,
                     const HoldSchedule *hold_schedule,
                     VisionTracker *vision_tracker,
                     volatile sig_atomic_t *keep_running)
{
    EncoderSample encoders;
    ControllerOutput output;
    MotorCommand command = protocol_stop_command();
    FILE *csv_log = 0;
    unsigned sample_index = 0;
    unsigned overrun_count = 0;
    unsigned long long spi_exchange_total_us = 0;
    unsigned long long control_compute_total_us = 0;
    unsigned long long work_total_us = 0;
    long spi_exchange_max_us = 0;
    long control_compute_max_us = 0;
    long work_max_us = 0;
    struct timespec next_deadline;
    struct timespec loop_start;
    uint64_t last_vision_frame_count = 0;
    int have_vision_frame_count = 0;
    ControlTarget active_vision_target = target;
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
                "work_us,lateness_us,overruns,"
                "target_source,hold_phase,"
                "spi_exchange_us,control_compute_us,"
                "vision_frame_interval_ms,vision_process_us,vision_late_frame\n");
    }

    /*
     * next_deadline is advanced before each sample. That means sample N targets
     * start_time + N * period, independent of how long previous samples took.
     */
    while (*keep_running) {
        struct timespec work_start;
        struct timespec spi_start;
        struct timespec spi_end;
        struct timespec compute_end;
        struct timespec work_end;
        double yaw_actual_rad;
        double pitch_actual_rad;
        VisionTargetSnapshot vision_snapshot;
        TargetSource target_source = TARGET_SOURCE_FIXED;
        int hold_phase = 0;
        long spi_exchange_us;
        long control_compute_us;
        long work_us;
        long lateness_us;

        memset(&vision_snapshot, 0, sizeof(vision_snapshot));

        next_deadline = timespec_add_us(next_deadline, config->sample_period_us);

        result = clock_gettime(CLOCK_MONOTONIC, &work_start);
        if (result < 0) {
            if (csv_log != 0) {
                fclose(csv_log);
            }
            return -errno;
        }

        /*
         * This exchange sends the command computed during the previous sample 
         * while receiving the encoder sample used for the next command.
         */
        result = clock_gettime(CLOCK_MONOTONIC, &spi_start);
        if (result < 0) {
            if (csv_log != 0) {
                fclose(csv_log);
            }
            return -errno;
        }
        result = motor_comm_exchange(comm, command, &encoders);
        if (result < 0) {
            if (csv_log != 0) {
                fclose(csv_log);
            }
            return result;
        }
        result = clock_gettime(CLOCK_MONOTONIC, &spi_end);
        if (result < 0) {
            if (csv_log != 0) {
                fclose(csv_log);
            }
            return -errno;
        }

        yaw_actual_rad = jiwy_yaw_rad(calibration, encoders.yaw);
        pitch_actual_rad = jiwy_pitch_rad(calibration, encoders.pitch);

        if (hold_schedule != 0) {
            double elapsed_s = (double)timespec_diff_us(work_start, loop_start) / 1000000.0;
            target = hold_schedule_target_at(hold_schedule, elapsed_s, &hold_phase);
            target_source = TARGET_SOURCE_HOLD_SCHEDULE;
        }

        if (vision_tracker != 0) {
            target = active_vision_target;
            hold_phase = 0;

            if (vision_tracker_read_latest(vision_tracker, &vision_snapshot) == 0 &&
                vision_snapshot.valid) {
                if (!have_vision_frame_count ||
                    vision_snapshot.frame_count != last_vision_frame_count) {
                    last_vision_frame_count = vision_snapshot.frame_count;
                    have_vision_frame_count = 1;
                    active_vision_target.yaw_target_rad = 
                        jiwy_clamp_yaw_target(calibration, yaw_actual_rad + JIWY_VISION_TARGET_GAIN * vision_snapshot.yaw_error_rad);
                    active_vision_target.pitch_target_rad =
                        jiwy_clamp_pitch_target(calibration, pitch_actual_rad + JIWY_VISION_TARGET_GAIN * vision_snapshot.pitch_error_rad);
                    target = active_vision_target;
                }
            }

            target_source = TARGET_SOURCE_VISION;
        }

        output = twentysim_controller_step(controller, calibration, encoders, target.yaw_target_rad, target.pitch_target_rad);
        command = controller_output_to_command(output);
        result = clock_gettime(CLOCK_MONOTONIC, &compute_end);
        if (result < 0) {
            if (csv_log != 0) {
                fclose(csv_log);
            }
            return -errno;
        }

        result = clock_gettime(CLOCK_MONOTONIC, &work_end);
        if (result < 0) {
            if (csv_log != 0) {
                fclose(csv_log);
            }
            return -errno;
        }

        spi_exchange_us = timespec_diff_us(spi_end, spi_start);
        control_compute_us = timespec_diff_us(compute_end, spi_end);
        work_us = timespec_diff_us(work_end, work_start);
        lateness_us = timespec_diff_us(work_end, next_deadline);
        if (lateness_us > 0) {
            ++overrun_count;
        }
        spi_exchange_total_us += (unsigned long long)spi_exchange_us;
        control_compute_total_us += (unsigned long long)control_compute_us;
        work_total_us += (unsigned long long)work_us;
        if (spi_exchange_us > spi_exchange_max_us) {
            spi_exchange_max_us = spi_exchange_us;
        }
        if (control_compute_us > control_compute_max_us) {
            control_compute_max_us = control_compute_us;
        }
        if (work_us > work_max_us) {
            work_max_us = work_us;
        }

        if (config->log_period_samples != 0 && sample_index % config->log_period_samples == 0) {
            printf("enc yaw=%d pitch=%d target yaw=%.4f pitch=%.4f actual yaw=%.4f pitch=%.4f spi=%ldus ctrl=%ldus work=%ldus late=%ldus overruns=%u src=%d phase=%d\n",
                   encoders.yaw,
                   encoders.pitch,
                   target.yaw_target_rad,
                   target.pitch_target_rad,
                   yaw_actual_rad,
                   pitch_actual_rad,
                   spi_exchange_us,
                   control_compute_us,
                   work_us,
                   lateness_us > 0 ? lateness_us : 0,
                   overrun_count,
                   (int)target_source,
                   hold_phase);
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
                    "%ld,%ld,%u,"
                    "%d,%d,"
                    "%ld,%ld,"
                    "%.3f,%.3f,%d\n",
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
                    overrun_count,
                    (int)target_source,
                    hold_phase,
                    spi_exchange_us,
                    control_compute_us,
                    vision_tracker != 0 ? vision_snapshot.frame_interval_ms : 0.0,
                    vision_tracker != 0 ? vision_snapshot.process_us : 0.0,
                    vision_tracker != 0 ? vision_snapshot.late_frame : 0);
        }

        ++sample_index;
        if (lateness_us > 0) {
            /*
             * If the sample has already missed its deadline, skip sleeping.
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

    if (sample_index > 0u) {
        printf("timing summary: samples=%u overruns=%u "
               "spi_avg=%.1fus spi_max=%ldus "
               "ctrl_avg=%.1fus ctrl_max=%ldus "
               "work_avg=%.1fus work_max=%ldus\n",
               sample_index,
               overrun_count,
               (double)spi_exchange_total_us / (double)sample_index,
               spi_exchange_max_us,
               (double)control_compute_total_us / (double)sample_index,
               control_compute_max_us,
               (double)work_total_us / (double)sample_index,
               work_max_us);
    }

    return motor_comm_exchange(comm, protocol_stop_command(), 0);
}