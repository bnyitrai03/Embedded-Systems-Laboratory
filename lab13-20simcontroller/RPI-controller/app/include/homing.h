#ifndef HOMING_H
#define HOMING_H

#include <signal.h>
#include <stdint.h>

#include "control/include/jiwy_calibration.h"
#include "comm/include/motor_comm.h"

/** @brief Runtime settings for homing. */
typedef struct {
    uint16_t pwm;
    MotorDirection yaw_home_direction;
    MotorDirection pitch_home_direction;
    unsigned sample_period_us;
    /** Consecutive stagnant samples required before home is accepted. */
    unsigned stop_window_samples;
    /** Encoder-count delta that still counts as movement. */
    unsigned movement_threshold_counts;
    unsigned max_samples_per_axis;
    /** Stop-command samples sent after home is detected. */
    unsigned settle_samples;
} HomingConfig;

/**
 * @brief Return conservative default homing settings.
 */
HomingConfig homing_default_config(void);

/**
 * @brief Measure and home yaw and pitch sequentially.
 *
 * Homing drives one axis at a time to both mechanical stops,
 * declares each stop when encoder counts stagnate, then
 * uses the measured span for runtime count to radian conversion.
 *
 * @return 0 on success or a negative errno-style value on failure.
 */
int homing_run(MotorComm *comm,
               const HomingConfig *config,
               JiwyCalibration *calibration,
               volatile sig_atomic_t *keep_running);

#endif
