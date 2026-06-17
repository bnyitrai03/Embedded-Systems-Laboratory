#include "vision_geometry.h"

#include <math.h>

#include "jiwy_config.h"

void vision_pixel_to_camera_error(double object_x,
                                  double object_y,
                                  int width,
                                  int height,
                                  double *yaw_error_rad,
                                  double *pitch_error_rad)
{
    double center_x = (double)width / 2.0;
    double center_y = (double)height / 2.0;
    double normalized_x = (object_x - center_x) / center_x;
    double normalized_y = (center_y - object_y) / center_y;
    double yaw = atan(normalized_x * tan(JIWY_VISION_HORIZONTAL_FOV_RAD / 2.0));
    double pitch = atan(normalized_y * tan(JIWY_VISION_VERTICAL_FOV_RAD / 2.0));

    if (fabs(yaw) < JIWY_VISION_YAW_DEADBAND_RAD) {
        yaw = 0.0;
    }
    if (fabs(pitch) < JIWY_VISION_PITCH_DEADBAND_RAD) {
        pitch = 0.0;
    }

    *yaw_error_rad = yaw;
    *pitch_error_rad = pitch;
}
