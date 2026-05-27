#ifndef CONTROL_LOOP_H
#define CONTROL_LOOP_H

#include <signal.h>

#include "jiwy_calibration.h"
#include "motor_comm.h"
#include "twentysim_controller.h"

/**
 * @file control_loop.h
 * @brief Fixed-period motor control loop with pluggable target providers.
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
} ControlLoopConfig;

/**
 * @brief Provide the next yaw/pitch target for one control sample.
 * @param context Provider-specific state.
 * @param calibration Current calibration and software home offsets.
 * @param encoders Latest encoder sample.
 * @param target Destination for the target used this sample.
 * @return 0 on success or a negative errno-style value on failure.
 */
typedef int (*ControlTargetProvider)(void *context,
                                     const JiwyCalibration *calibration,
                                     EncoderSample encoders,
                                     ControlTarget *target);

/**
 * @brief Return default 100 Hz control loop settings.
 */
ControlLoopConfig control_loop_default_config(void);

/**
 * @brief Target provider that always returns the ControlTarget in context.
 */
int control_loop_fixed_target(void *context,
                              const JiwyCalibration *calibration,
                              EncoderSample encoders,
                              ControlTarget *target);

/**
 * @brief Run the full-duplex SPI control loop until keep_running is cleared.
 *
 * Target providers are the extension point for future behavior:
 * fixed-position hold, joystick/manual input, trajectory playback, or vision.
 * They should be non-blocking so the 20-sim controller keeps its sample period.
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
                     ControlTargetProvider target_provider,
                     void *target_context,
                     volatile sig_atomic_t *keep_running);

#endif
