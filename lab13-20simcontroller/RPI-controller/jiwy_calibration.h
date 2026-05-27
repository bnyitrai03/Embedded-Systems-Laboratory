#ifndef JIWY_CALIBRATION_H
#define JIWY_CALIBRATION_H

#include <stdint.h>

/**
 * @file jiwy_calibration.h
 * @brief Encoder calibration, software home offsets, and target limits.
 */

/**
 * @brief Runtime calibration for converting encoder counts to radians.
 *
 * Calibration values are deliberately runtime data, even though defaults come
 * from jiwy_config.h. Homing updates the *_home_count fields, and future setup
 * tools can load these values from a file without touching generated 20-sim C.
 */
typedef struct {
    /** Yaw encoder counts per radian. */
    double yaw_counts_per_rad;
    /** Pitch encoder counts per radian. */
    double pitch_counts_per_rad;
    /** Encoder count treated as yaw zero after homing. */
    int16_t yaw_home_count;
    /** Encoder count treated as pitch zero after homing. */
    int16_t pitch_home_count;
    /** Minimum allowed yaw target, relative to home. */
    double yaw_min_rad;
    /** Maximum allowed yaw target, relative to home. */
    double yaw_max_rad;
    /** Minimum allowed pitch target, relative to home. */
    double pitch_min_rad;
    /** Maximum allowed pitch target, relative to home. */
    double pitch_max_rad;
} JiwyCalibration;

/**
 * @brief Build default calibration from jiwy_config.h.
 * @return Calibration with measured counts-per-radian values and zero homes.
 */
JiwyCalibration jiwy_default_calibration(void);

/**
 * @brief Convert a raw yaw encoder count to radians relative to yaw home.
 */
double jiwy_yaw_rad(const JiwyCalibration *calibration, int16_t count);

/**
 * @brief Convert a raw pitch encoder count to radians relative to pitch home.
 */
double jiwy_pitch_rad(const JiwyCalibration *calibration, int16_t count);

/**
 * @brief Clamp a yaw target to configured software travel limits.
 */
double jiwy_clamp_yaw_target(const JiwyCalibration *calibration,
                             double target_rad);

/**
 * @brief Clamp a pitch target to configured software travel limits.
 */
double jiwy_clamp_pitch_target(const JiwyCalibration *calibration,
                               double target_rad);

#endif
