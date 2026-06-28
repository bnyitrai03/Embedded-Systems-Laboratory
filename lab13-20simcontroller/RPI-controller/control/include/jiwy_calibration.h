#ifndef JIWY_CALIBRATION_H
#define JIWY_CALIBRATION_H

#include <stdint.h>

/**
 * @brief Runtime calibration for converting encoder counts to radians.
 */
typedef struct {
    double yaw_counts_per_rad;
    double pitch_counts_per_rad;
    /** Encoder count treated as yaw zero after homing. */
    int16_t yaw_home_count;
    int16_t pitch_home_count;
    /** Minimum allowed yaw target, relative to home. */
    double yaw_min_rad;
    double yaw_max_rad;
    double pitch_min_rad;
    double pitch_max_rad;
} JiwyCalibration;

/**
 * @brief Build default calibration from jiwy_config.h.
 * @return Calibration with zero homes and travel limits.
 */
JiwyCalibration jiwy_default_calibration(void);

/**
 * @brief Update yaw count-to-radian calibration from measured encoder travel.
 */
void jiwy_set_yaw_travel_counts(JiwyCalibration *calibration, unsigned travel_counts);

void jiwy_set_pitch_travel_counts(JiwyCalibration *calibration, unsigned travel_counts);

/**
 * @brief Convert a raw yaw encoder count to radians relative to yaw home.
 */
double jiwy_yaw_rad(const JiwyCalibration *calibration, int16_t count);

double jiwy_pitch_rad(const JiwyCalibration *calibration, int16_t count);

/**
 * @brief Clamp a yaw target to configured software travel limits.
 */
double jiwy_clamp_yaw_target(const JiwyCalibration *calibration, double target_rad);

double jiwy_clamp_pitch_target(const JiwyCalibration *calibration, double target_rad);

#endif
