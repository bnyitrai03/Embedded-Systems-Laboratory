#ifndef JIWY_CALIBRATION_H
#define JIWY_CALIBRATION_H

#include <stdint.h>

/*
 * Calibration values are deliberately runtime data, even though defaults come
 * from jiwy_config.h. Homing updates the *_home_count fields, and future setup
 * tools can load these values from a file without touching generated 20-sim C.
 */
typedef struct {
    double yaw_counts_per_rad;
    double pitch_counts_per_rad;
    int16_t yaw_home_count;
    int16_t pitch_home_count;
    double yaw_min_rad;
    double yaw_max_rad;
    double pitch_min_rad;
    double pitch_max_rad;
} JiwyCalibration;

JiwyCalibration jiwy_default_calibration(void);

double jiwy_yaw_rad(const JiwyCalibration *calibration, int16_t count);
double jiwy_pitch_rad(const JiwyCalibration *calibration, int16_t count);

double jiwy_clamp_yaw_target(const JiwyCalibration *calibration,
                             double target_rad);
double jiwy_clamp_pitch_target(const JiwyCalibration *calibration,
                               double target_rad);

#endif
