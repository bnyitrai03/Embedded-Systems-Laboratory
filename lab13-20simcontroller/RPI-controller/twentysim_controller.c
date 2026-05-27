#include "twentysim_controller.h"

#include <math.h>
#include <stdint.h>

#include "jiwy_config.h"

enum {
    /* Generated parameter order from yaw_model.c and pitch_model.c. */
    PARAM_CORR_GAIN = 0,
    PARAM_KP = 1,
    PARAM_TAU_D = 2,
    PARAM_BETA = 3,
    PARAM_TAU_I = 4,
    PARAM_OUTPUT_MIN = 5,
    PARAM_OUTPUT_MAX = 6
};

static void set_yaw_inputs(TwentySimController *controller,
                           const JiwyCalibration *calibration,
                           EncoderSample encoders,
                           double target_rad)
{
    controller->yaw_u[0] = jiwy_clamp_yaw_target(calibration, target_rad);
    controller->yaw_u[1] = jiwy_yaw_rad(calibration, encoders.yaw);
}

static void set_pitch_inputs(TwentySimController *controller,
                             const JiwyCalibration *calibration,
                             EncoderSample encoders,
                             double target_rad)
{
    controller->pitch_u[0] = 0.0;
    controller->pitch_u[1] = jiwy_clamp_pitch_target(calibration, target_rad);
    controller->pitch_u[2] = jiwy_pitch_rad(calibration, encoders.pitch);
}

static void override_yaw_parameters(void)
{
    yaw_P[PARAM_CORR_GAIN] = JIWY_YAW_CORR_GAIN;
    yaw_P[PARAM_KP] = JIWY_YAW_KP;
    yaw_P[PARAM_TAU_D] = JIWY_YAW_TAU_D;
    yaw_P[PARAM_BETA] = JIWY_YAW_BETA;
    yaw_P[PARAM_TAU_I] = JIWY_YAW_TAU_I;
    yaw_P[PARAM_OUTPUT_MIN] = JIWY_YAW_OUTPUT_MIN;
    yaw_P[PARAM_OUTPUT_MAX] = JIWY_YAW_OUTPUT_MAX;
}

static void override_pitch_parameters(void)
{
    pitch_P[PARAM_CORR_GAIN] = JIWY_PITCH_CORR_GAIN;
    pitch_P[PARAM_KP] = JIWY_PITCH_KP;
    pitch_P[PARAM_TAU_D] = JIWY_PITCH_TAU_D;
    pitch_P[PARAM_BETA] = JIWY_PITCH_BETA;
    pitch_P[PARAM_TAU_I] = JIWY_PITCH_TAU_I;
    pitch_P[PARAM_OUTPUT_MIN] = JIWY_PITCH_OUTPUT_MIN;
    pitch_P[PARAM_OUTPUT_MAX] = JIWY_PITCH_OUTPUT_MAX;
}

void twentysim_controller_init(TwentySimController *controller,
                               double step_size_s,
                               const JiwyCalibration *calibration,
                               EncoderSample initial_encoders,
                               double initial_yaw_target_rad,
                               double initial_pitch_target_rad)
{
    yaw_step_size = step_size_s;
    pitch_step_size = step_size_s;

    /*
     * Match the generated example mains: fill u/y arrays, initialize the
     * submodels, then call CalculateSubmodel on each loop sample. The generated
     * integrators maintain yaw_time and pitch_time globally.
     */
    set_yaw_inputs(controller,
                   calibration,
                   initial_encoders,
                   initial_yaw_target_rad);
    set_pitch_inputs(controller,
                     calibration,
                     initial_encoders,
                     initial_pitch_target_rad);

    YawInitializeSubmodel(controller->yaw_u, controller->yaw_y, 0.0);
    PitchInitializeSubmodel(controller->pitch_u, controller->pitch_y, 0.0);

    override_yaw_parameters();
    override_pitch_parameters();
}

ControllerOutput twentysim_controller_step(TwentySimController *controller,
                                           const JiwyCalibration *calibration,
                                           EncoderSample encoders,
                                           double yaw_target_rad,
                                           double pitch_target_rad)
{
    ControllerOutput output;

    set_yaw_inputs(controller, calibration, encoders, yaw_target_rad);
    YawCalculateSubmodel(controller->yaw_u,
                         controller->yaw_y,
                         yaw_time);

    /*
     * The generated pitch submodel still has a correction input at u[0].
     * The wrapper intentionally keeps that hidden and fixed at zero so the
     * application-level control target remains yaw/pitch position only.
     */
    set_pitch_inputs(controller,
                     calibration,
                     encoders,
                     pitch_target_rad);
    PitchCalculateSubmodel(controller->pitch_u,
                           controller->pitch_y,
                           pitch_time);

    /* Yaw exposes {corr, out}; pitch exposes {out}. */
    output.yaw = controller->yaw_y[1];
    output.pitch = controller->pitch_y[0];
    return output;
}

static AxisCommand output_to_axis_command(double output)
{
    AxisCommand command;
    double magnitude = fabs(output);
    double pwm;

    command.direction = (output >= 0.0) ? MOTOR_DIR_POSITIVE : MOTOR_DIR_NEGATIVE;
    command.enable = 0;
    command.pwm = 0;

    if (!(magnitude > 0.0)) {
        return command;
    }

    pwm = magnitude * (double)MOTOR_PWM_MAX;
    if (pwm > (double)MOTOR_PWM_MAX) {
        pwm = (double)MOTOR_PWM_MAX;
    }

    command.pwm = (uint16_t)lround(pwm);
    command.enable = 1;
    return command;
}

MotorCommand controller_output_to_command(ControllerOutput output)
{
    MotorCommand command;

    command.yaw = output_to_axis_command(output.yaw);
    command.pitch = output_to_axis_command(output.pitch);
    return command;
}
