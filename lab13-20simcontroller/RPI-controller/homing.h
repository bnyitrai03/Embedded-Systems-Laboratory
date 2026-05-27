#ifndef HOMING_H
#define HOMING_H

#include <signal.h>
#include <stdint.h>

#include "jiwy_calibration.h"
#include "motor_comm.h"

typedef struct {
    uint16_t pwm;
    MotorDirection yaw_home_direction;
    MotorDirection pitch_home_direction;
    unsigned sample_period_us;
    unsigned stop_window_samples;
    unsigned movement_threshold_counts;
    unsigned max_samples_per_axis;
    unsigned settle_samples;
} HomingConfig;

/*
 * Homing assumes there are no limit switches. It drives one axis at low PWM and
 * declares home when encoder counts stop changing for stop_window_samples.
 */
HomingConfig homing_default_config(void);
int homing_run(MotorComm *comm,
               const HomingConfig *config,
               JiwyCalibration *calibration,
               volatile sig_atomic_t *keep_running);

#endif
