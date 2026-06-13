#include "jiwy_calibration.h"

#include "jiwy_config.h"

static double clamp(double value, double min_value, double max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static double degrees_to_rad(double degrees)
{
    return degrees * JIWY_PI / 180.0;
}

JiwyCalibration jiwy_default_calibration(void)
{
    JiwyCalibration calibration;

    calibration.yaw_counts_per_rad = 0.0;
    calibration.pitch_counts_per_rad = 0.0;
    calibration.yaw_home_count = 0;
    calibration.pitch_home_count = 0;
    calibration.yaw_min_rad = JIWY_YAW_MIN_RAD;
    calibration.yaw_max_rad = JIWY_YAW_MAX_RAD;
    calibration.pitch_min_rad = JIWY_PITCH_MIN_RAD;
    calibration.pitch_max_rad = JIWY_PITCH_MAX_RAD;
    return calibration;
}

void jiwy_set_yaw_travel_counts(JiwyCalibration *calibration,
                                unsigned travel_counts)
{
    calibration->yaw_counts_per_rad =
        (double)travel_counts / degrees_to_rad(JIWY_YAW_TRAVEL_DEGREES);
}

void jiwy_set_pitch_travel_counts(JiwyCalibration *calibration,
                                  unsigned travel_counts)
{
    calibration->pitch_counts_per_rad =
        (double)travel_counts / degrees_to_rad(JIWY_PITCH_TRAVEL_DEGREES);
}

double jiwy_yaw_rad(const JiwyCalibration *calibration, int16_t count)
{
    return ((double)count - (double)calibration->yaw_home_count) /
           calibration->yaw_counts_per_rad;
}

double jiwy_pitch_rad(const JiwyCalibration *calibration, int16_t count)
{
    return ((double)count - (double)calibration->pitch_home_count) /
           calibration->pitch_counts_per_rad;
}

double jiwy_clamp_yaw_target(const JiwyCalibration *calibration,
                             double target_rad)
{
    return clamp(target_rad,
                 calibration->yaw_min_rad,
                 calibration->yaw_max_rad);
}

double jiwy_clamp_pitch_target(const JiwyCalibration *calibration,
                               double target_rad)
{
    return clamp(target_rad,
                 calibration->pitch_min_rad,
                 calibration->pitch_max_rad);
}
