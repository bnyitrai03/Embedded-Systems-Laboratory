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

static uint64_t timespec_to_ns(struct timespec ts)
{
    return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}

typedef struct {
    uint64_t monotonic_ns;
    double yaw_rad;
    double pitch_rad;
} EncoderHistorySample;

static void encoder_history_push(EncoderHistorySample *ring,
                                 unsigned capacity,
                                 unsigned *head,
                                 unsigned *count,
                                 uint64_t ns,
                                 double yaw_rad,
                                 double pitch_rad)
{
    ring[*head].monotonic_ns = ns;
    ring[*head].yaw_rad = yaw_rad;
    ring[*head].pitch_rad = pitch_rad;
    *head = (*head + 1u) % capacity;
    if (*count < capacity) {
        ++*count;
    }
}

/*
 * Look up the camera angle at a given monotonic timestamp by linear
 * interpolation between the two nearest history samples. Returns 0 on success,
 * -1 if the history is empty. If ns is older than the oldest sample, the oldest
 * sample is returned; if newer than the newest, the newest is returned.
 */
static int encoder_history_lookup(const EncoderHistorySample *ring,
                                  unsigned capacity,
                                  unsigned head,
                                  unsigned count,
                                  uint64_t ns,
                                  double *yaw_rad,
                                  double *pitch_rad)
{
    unsigned newest_index;
    unsigned oldest_index;
    unsigned i;
    const EncoderHistorySample *newest;
    const EncoderHistorySample *oldest;

    if (count == 0) {
        return -1;
    }

    newest_index = (head + capacity - 1u) % capacity;
    oldest_index = (head + capacity - count) % capacity;
    newest = &ring[newest_index];
    oldest = &ring[oldest_index];

    if (ns <= oldest->monotonic_ns) {
        *yaw_rad = oldest->yaw_rad;
        *pitch_rad = oldest->pitch_rad;
        return 0;
    }
    if (ns >= newest->monotonic_ns) {
        *yaw_rad = newest->yaw_rad;
        *pitch_rad = newest->pitch_rad;
        return 0;
    }

    for (i = 0; i + 1u < count; ++i) {
        unsigned idx_a = (oldest_index + i) % capacity;
        unsigned idx_b = (oldest_index + i + 1u) % capacity;
        const EncoderHistorySample *a = &ring[idx_a];
        const EncoderHistorySample *b = &ring[idx_b];
        uint64_t span;
        double frac;

        if (ns < a->monotonic_ns || ns > b->monotonic_ns) {
            continue;
        }

        span = b->monotonic_ns - a->monotonic_ns;
        frac = 0.0;
        if (span > 0u) {
            frac = (double)(ns - a->monotonic_ns) / (double)span;
        }
        *yaw_rad = a->yaw_rad + frac * (b->yaw_rad - a->yaw_rad);
        *pitch_rad = a->pitch_rad + frac * (b->pitch_rad - a->pitch_rad);
        return 0;
    }

    *yaw_rad = newest->yaw_rad;
    *pitch_rad = newest->pitch_rad;
    return 0;
}

static ControlTarget hold_schedule_target_at(const HoldSchedule *schedule,
                                             double elapsed_s,
                                             int *phase_out)
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
    unsigned stale_vision_samples = JIWY_VISION_MAX_STALE_CONTROL_SAMPLES + 1u;
    int have_vision_frame_count = 0;
    EncoderHistorySample encoder_history[JIWY_CONTROL_ENCODER_HISTORY_SAMPLES];
    unsigned encoder_history_head = 0;
    unsigned encoder_history_count = 0;
    double est_ball_yaw_world = 0.0;
    double est_ball_pitch_world = 0.0;
    int have_est = 0;
    double ball_vel_yaw_rad_per_s = 0.0;
    double ball_vel_pitch_rad_per_s = 0.0;
    double ramped_yaw_target_rad = 0.0;
    double ramped_pitch_target_rad = 0.0;
    int have_ramped_target = 0;
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
                "vision_frame_interval_ms,vision_process_us,vision_late_frame,"
                "est_ball_yaw,est_ball_pitch,"
                "ramped_yaw,ramped_pitch,"
                "ball_vel_yaw,ball_vel_pitch,"
                "captured_age_ms,vision_lost\n");
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
        int vision_lost = 0;

        memset(&vision_snapshot, 0, sizeof(vision_snapshot));

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

        encoder_history_push(encoder_history,
                             JIWY_CONTROL_ENCODER_HISTORY_SAMPLES,
                             &encoder_history_head,
                             &encoder_history_count,
                             timespec_to_ns(work_start),
                             yaw_actual_rad,
                             pitch_actual_rad);

        if (hold_schedule != 0) {
            double elapsed_s =
                (double)timespec_diff_us(work_start, loop_start) / 1000000.0;
            target = hold_schedule_target_at(hold_schedule, elapsed_s, &hold_phase);
            target_source = TARGET_SOURCE_HOLD_SCHEDULE;
        }

        if (vision_tracker != 0) {
            double dt_s = (double)config->sample_period_us / 1000000.0;

            target.yaw_target_rad = yaw_actual_rad;
            target.pitch_target_rad = pitch_actual_rad;
            target_source = TARGET_SOURCE_FIXED;
            hold_phase = 0;

            if (vision_tracker_read_latest(vision_tracker, &vision_snapshot) == 0 &&
                vision_snapshot.valid) {
                if (!have_vision_frame_count ||
                    vision_snapshot.frame_count != last_vision_frame_count) {
                    double camera_yaw_at_capture;
                    double camera_pitch_at_capture;
                    double measured_yaw_world;
                    double measured_pitch_world;
                    double yaw_diff = 0.0;
                    double pitch_diff = 0.0;
                    int update_estimate;

                    stale_vision_samples = 0;
                    last_vision_frame_count = vision_snapshot.frame_count;
                    have_vision_frame_count = 1;

                    /*
                     * Latency correction: build the ball world angle from the
                     * camera angle at the instant the frame was captured, not
                     * the current camera angle. This decouples the setpoint
                     * from camera motion and removes the positive feedback
                     * that caused the oscillation.
                     */
                    if (encoder_history_lookup(encoder_history,
                                               JIWY_CONTROL_ENCODER_HISTORY_SAMPLES,
                                               encoder_history_head,
                                               encoder_history_count,
                                               vision_snapshot.captured_monotonic_ns,
                                               &camera_yaw_at_capture,
                                               &camera_pitch_at_capture) == 0) {
                        measured_yaw_world =
                            camera_yaw_at_capture + vision_snapshot.yaw_error_rad;
                        measured_pitch_world =
                            camera_pitch_at_capture + vision_snapshot.pitch_error_rad;
                    } else {
                        measured_yaw_world =
                            yaw_actual_rad + vision_snapshot.yaw_error_rad;
                        measured_pitch_world =
                            pitch_actual_rad + vision_snapshot.pitch_error_rad;
                    }

                    /*
                     * Deadband gate at the estimate update: only fold in the
                     * new measurement when it moves the estimate beyond the
                     * deadband. This kills the centered-ball limit cycle at
                     * the source instead of relying on the PID to damp it.
                     */
                    if (have_est) {
                        yaw_diff = measured_yaw_world - est_ball_yaw_world;
                        pitch_diff = measured_pitch_world - est_ball_pitch_world;
                        update_estimate =
                            fabs(yaw_diff) > JIWY_VISION_YAW_DEADBAND_RAD ||
                            fabs(pitch_diff) > JIWY_VISION_PITCH_DEADBAND_RAD;
                    } else {
                        update_estimate = 1;
                    }

                    if (update_estimate) {
                        if (!have_est) {
                            est_ball_yaw_world = measured_yaw_world;
                            est_ball_pitch_world = measured_pitch_world;
                            ball_vel_yaw_rad_per_s = 0.0;
                            ball_vel_pitch_rad_per_s = 0.0;
                        } else {
                            double prev_yaw = est_ball_yaw_world;
                            double prev_pitch = est_ball_pitch_world;
                            double alpha = JIWY_VISION_WORLD_FILTER_ALPHA;
                            double frame_dt_s;

                            if (alpha < 0.0) {
                                alpha = 0.0;
                            } else if (alpha > 1.0) {
                                alpha = 1.0;
                            }

                            est_ball_yaw_world +=
                                alpha * (measured_yaw_world - est_ball_yaw_world);
                            est_ball_pitch_world +=
                                alpha * (measured_pitch_world - est_ball_pitch_world);

                            frame_dt_s =
                                vision_snapshot.frame_interval_ms / 1000.0;
                            if (frame_dt_s < 1e-3) {
                                frame_dt_s = 1e-3;
                            }
                            ball_vel_yaw_rad_per_s =
                                (est_ball_yaw_world - prev_yaw) / frame_dt_s;
                            ball_vel_pitch_rad_per_s =
                                (est_ball_pitch_world - prev_pitch) / frame_dt_s;
                        }
                        have_est = 1;
                    } else {
                        ball_vel_yaw_rad_per_s *= 0.5;
                        ball_vel_pitch_rad_per_s *= 0.5;
                    }
                } else if (stale_vision_samples <=
                           JIWY_VISION_MAX_STALE_CONTROL_SAMPLES) {
                    ++stale_vision_samples;
                }
            } else {
                stale_vision_samples =
                    JIWY_VISION_MAX_STALE_CONTROL_SAMPLES + 1u;
                /*
                 * Keep the last world estimate (freeze where the ball was), but
                 * decay any stale velocity so feedforward does not push the
                 * setpoint while the ball is lost.
                 */
                ball_vel_yaw_rad_per_s *= 0.5;
                ball_vel_pitch_rad_per_s *= 0.5;
            }

            vision_lost = (stale_vision_samples >
                           JIWY_VISION_MAX_STALE_CONTROL_SAMPLES) || !have_est;

            if (have_est && !vision_lost) {
                double desired_yaw =
                    est_ball_yaw_world +
                    JIWY_VISION_FEEDFORWARD_GAIN_YAW * ball_vel_yaw_rad_per_s;
                double desired_pitch =
                    est_ball_pitch_world +
                    JIWY_VISION_FEEDFORWARD_GAIN_PITCH * ball_vel_pitch_rad_per_s;
                double max_step =
                    JIWY_VISION_SETPOINT_MAX_RATE_RAD_PER_S * dt_s;

                if (!have_ramped_target) {
                    ramped_yaw_target_rad = desired_yaw;
                    ramped_pitch_target_rad = desired_pitch;
                    have_ramped_target = 1;
                } else {
                    double yaw_step = desired_yaw - ramped_yaw_target_rad;
                    double pitch_step = desired_pitch - ramped_pitch_target_rad;

                    if (yaw_step > max_step) {
                        yaw_step = max_step;
                    } else if (yaw_step < -max_step) {
                        yaw_step = -max_step;
                    }
                    if (pitch_step > max_step) {
                        pitch_step = max_step;
                    } else if (pitch_step < -max_step) {
                        pitch_step = -max_step;
                    }
                    ramped_yaw_target_rad += yaw_step;
                    ramped_pitch_target_rad += pitch_step;
                }

                target.yaw_target_rad = ramped_yaw_target_rad;
                target.pitch_target_rad = ramped_pitch_target_rad;
                target_source = TARGET_SOURCE_VISION;
            } else if (have_ramped_target) {
                /*
                 * Ball lost: freeze the ramped target at the last known ball
                 * direction so the camera holds where the ball was, instead of
                 * snapping to the current (possibly drifted) camera angle.
                 */
                target.yaw_target_rad = ramped_yaw_target_rad;
                target.pitch_target_rad = ramped_pitch_target_rad;
                target_source = TARGET_SOURCE_VISION;
            }
        }

        output = twentysim_controller_step(controller,
                                           calibration,
                                           encoders,
                                           target.yaw_target_rad,
                                           target.pitch_target_rad);
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

        if (config->log_period_samples != 0 &&
            sample_index % config->log_period_samples == 0) {
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
            double captured_age_ms = 0.0;

            if (vision_tracker != 0 && vision_snapshot.captured_monotonic_ns != 0u) {
                uint64_t work_start_ns = timespec_to_ns(work_start);

                if (work_start_ns >= vision_snapshot.captured_monotonic_ns) {
                    captured_age_ms =
                        (double)(work_start_ns -
                                 vision_snapshot.captured_monotonic_ns) /
                        1000000.0;
                }
            }

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
                    "%.3f,%.3f,%d,"
                    "%.9f,%.9f,"
                    "%.9f,%.9f,"
                    "%.6f,%.6f,"
                    "%.3f,%d\n",
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
                    vision_tracker != 0 ? vision_snapshot.late_frame : 0,
                    vision_tracker != 0 && have_est ? est_ball_yaw_world : 0.0,
                    vision_tracker != 0 && have_est ? est_ball_pitch_world : 0.0,
                    vision_tracker != 0 && have_ramped_target ? ramped_yaw_target_rad : 0.0,
                    vision_tracker != 0 && have_ramped_target ? ramped_pitch_target_rad : 0.0,
                    vision_tracker != 0 ? ball_vel_yaw_rad_per_s : 0.0,
                    vision_tracker != 0 ? ball_vel_pitch_rad_per_s : 0.0,
                    captured_age_ms,
                    vision_tracker != 0 ? vision_lost : 0);
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
