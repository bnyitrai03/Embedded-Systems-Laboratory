#ifndef TWENTYSIM_CONTROLLER_H
#define TWENTYSIM_CONTROLLER_H

#include "control_protocol.h"
#include "controller/pitch_controller/pitch_submod.h"
#include "controller/yaw_controller/yaw_submod.h"
#include "jiwy_calibration.h"

/**
 * @file twentysim_controller.h
 * @brief Thin wrapper around generated yaw and pitch 20-sim submodels.
 */

/** @brief Normalized yaw/pitch controller output before PWM conversion. */
typedef struct {
    /** Yaw command, normally limited by generated 20-sim output clamps. */
    double yaw;
    /** Pitch command, normally limited by generated 20-sim output clamps. */
    double pitch;
} ControllerOutput;

/**
 * @brief Runtime storage for generated submodel input and output vectors.
 *
 * The generated 20-sim code keeps model state in globals, so only one active
 * controller instance should be used at a time.
 */
typedef struct {
    /** Yaw submodel inputs: target angle, measured position. */
    YawDouble yaw_u[2];
    /** Yaw submodel outputs: generated correction signal, motor output. */
    YawDouble yaw_y[2];
    /** Pitch submodel inputs: generated correction slot, target, position. */
    PitchDouble pitch_u[3];
    /** Pitch submodel output: motor output. */
    PitchDouble pitch_y[1];
} TwentySimController;

/**
 * @brief Initialize generated 20-sim yaw and pitch submodels.
 *
 * This function sets the generated sample time, seeds initial inputs from the
 * current encoder feedback, calls generated submodel initializers, then copies
 * PID tuning constants from jiwy_config.h into generated parameter arrays.
 */
void twentysim_controller_init(TwentySimController *controller,
                               double step_size_s,
                               const JiwyCalibration *calibration,
                               EncoderSample initial_encoders,
                               double initial_yaw_target_rad,
                               double initial_pitch_target_rad);

/**
 * @brief Run one generated 20-sim control sample.
 *
 * Targets are clamped by JiwyCalibration before entering the generated
 * submodels. Encoder counts are converted to radians relative to software home.
 */
ControllerOutput twentysim_controller_step(TwentySimController *controller,
                                           const JiwyCalibration *calibration,
                                           EncoderSample encoders,
                                           double yaw_target_rad,
                                           double pitch_target_rad);

/**
 * @brief Convert normalized controller output to FPGA motor command fields.
 */
MotorCommand controller_output_to_command(ControllerOutput output);

#endif
