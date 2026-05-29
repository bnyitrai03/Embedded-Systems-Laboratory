#ifndef HOMING_H
#define HOMING_H

#include <signal.h>
#include <stdint.h>

#include "jiwy_calibration.h"
#include "motor_comm.h"

/**
 * @file homing.h
 * @brief Software homing for the JIWY yaw and pitch axes.
 */

/** @brief Runtime settings for encoder-stagnation homing. */
typedef struct {
    /** PWM magnitude used while driving into the mechanical stop. */
    uint16_t pwm;
    /** Direction used to find yaw home. */
    MotorDirection yaw_home_direction;
    /** Direction used to find pitch home. */
    MotorDirection pitch_home_direction;
    /** Delay between homing samples. */
    unsigned sample_period_us;
    /** Consecutive stagnant samples required before home is accepted. */
    unsigned stop_window_samples;
    /** Encoder-count delta that still counts as movement. */
    unsigned movement_threshold_counts;
    /** Timeout limit per axis. */
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
 * Homing assumes there are no limit switches. It drives one axis at a time to
 * both mechanical stops, declares each stop when encoder counts stagnate, then
 * uses the measured span for runtime count-to-radian calibration.
 *
 * @param comm Motor communication backend.
 * @param config Homing settings.
 * @param calibration Calibration object whose home counts and spans are updated.
 * @param keep_running Signal-controlled run flag.
 * @return 0 on success or a negative errno-style value on failure.
 */
int homing_run(MotorComm *comm,
               const HomingConfig *config,
               JiwyCalibration *calibration,
               volatile sig_atomic_t *keep_running);

#endif
