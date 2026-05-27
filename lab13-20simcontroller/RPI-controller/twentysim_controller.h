#ifndef TWENTYSIM_CONTROLLER_H
#define TWENTYSIM_CONTROLLER_H

#include "control_protocol.h"
#include "controller/pitch_controller/pitch_submod.h"
#include "controller/yaw_controller/yaw_submod.h"
#include "jiwy_calibration.h"

typedef struct {
    double yaw;
    double pitch;
} ControllerOutput;

typedef struct {
    YawDouble yaw_u[2];
    YawDouble yaw_y[2];
    PitchDouble pitch_u[3];
    PitchDouble pitch_y[1];
} TwentySimController;

void twentysim_controller_init(TwentySimController *controller,
                               double step_size_s,
                               const JiwyCalibration *calibration,
                               EncoderSample initial_encoders,
                               double initial_yaw_target_rad,
                               double initial_pitch_target_rad);

ControllerOutput twentysim_controller_step(TwentySimController *controller,
                                           const JiwyCalibration *calibration,
                                           EncoderSample encoders,
                                           double yaw_target_rad,
                                           double pitch_target_rad);

MotorCommand controller_output_to_command(ControllerOutput output);

#endif
