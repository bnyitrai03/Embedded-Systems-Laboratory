#ifndef CONTROL_LOOP_H
#define CONTROL_LOOP_H

#include <signal.h>

#include "jiwy_calibration.h"
#include "motor_comm.h"
#include "twentysim_controller.h"
#include "vision_tracker.h"

/**
 * @file control_loop.h
 * @brief Fixed-period motor control loop for one yaw/pitch target.
 */

/** @brief Position target consumed by the 20-sim controller wrapper. */
typedef struct {
    /** Desired yaw angle in radians relative to software home. */
    double yaw_target_rad;
    /** Desired pitch angle in radians relative to software home. */
    double pitch_target_rad;
} ControlTarget;

/** @brief Timing and logging settings for the control loop. */
typedef struct {
    /** Control period in microseconds. */
    unsigned sample_period_us;
    /** Print one status line every N samples; zero disables logging. */
    unsigned log_period_samples;
    /** Optional CSV path for control-loop samples; NULL disables CSV logging. */
    const char *csv_log_path;
} ControlLoopConfig;

/**
 * @brief Return default 100 Hz control loop settings.
 */
ControlLoopConfig control_loop_default_config(void);

/**
 * @brief Run the full-duplex SPI control loop until keep_running is cleared.
 *
 * The loop is scheduled against absolute CLOCK_MONOTONIC deadlines. If a sample
 * overruns its deadline, the next iteration starts immediately and the overrun
 * is counted in the periodic status log.
 *
 * @return 0 on clean shutdown or a negative errno-style value on failure.
 */
int control_loop_run(MotorComm *comm,
                     TwentySimController *controller,
                     const JiwyCalibration *calibration,
                     const ControlLoopConfig *config,
                     ControlTarget target,
                     VisionTracker *vision_tracker,
                     volatile sig_atomic_t *keep_running);

#endif
