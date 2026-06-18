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

    /*
     * No deadband here: zeroing the raw error biases the world-frame estimate
     * toward the camera angle instead of the true ball angle. Limit-cycle
     * prevention is handled at the world-estimate update in control_loop.c,
     * where the deadband gates whether the estimate is folded in at all.
     */

    *yaw_error_rad = yaw;
    *pitch_error_rad = pitch;
}
