#ifndef CONTROLLER_ADAPTER_H
#define CONTROLLER_ADAPTER_H

#include "control_protocol.h"

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

typedef struct {
    double yaw;
    double pitch;
} ControllerOutput;

/* Initialize generated 20-sim models and overwrite their tunable parameters. */
JiwyCalibration jiwy_default_calibration(void);
void controller_20sim_init(double step_size_s);

/*
 * Run one controller sample.
 *
 * Targets and feedback are in radians. The returned outputs are normalized
 * controller commands in roughly [-1, 1], later converted to PWM.
 */
ControllerOutput controller_20sim_step(const JiwyCalibration *calibration,
                                       EncoderSample encoders,
                                       double yaw_target_rad,
                                       double pitch_target_rad,
                                       double pitch_correction_rad);
MotorCommand controller_output_to_command(ControllerOutput output);
double jiwy_yaw_rad(const JiwyCalibration *calibration, int16_t encoder_count);
double jiwy_pitch_rad(const JiwyCalibration *calibration, int16_t encoder_count);

#endif
