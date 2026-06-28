#include "twentysim_controller.h"

#include <math.h>
#include <stdint.h>

static void set_yaw_inputs(TwentySimController *controller, const JiwyCalibration *calibration, EncoderSample encoders, double target_rad)
{
    controller->yaw_u[0] = jiwy_clamp_yaw_target(calibration, target_rad);
    controller->yaw_u[1] = jiwy_yaw_rad(calibration, encoders.yaw);
}

static void set_pitch_inputs(TwentySimController *controller, const JiwyCalibration *calibration, EncoderSample encoders, double target_rad)
{
    controller->pitch_u[0] = 0.0;
    controller->pitch_u[1] = jiwy_clamp_pitch_target(calibration, target_rad);
    controller->pitch_u[2] = jiwy_pitch_rad(calibration, encoders.pitch);
}

void twentysim_controller_init(TwentySimController *controller, double step_size_s, const JiwyCalibration *calibration, EncoderSample initial_encoders, double initial_yaw_target_rad, double initial_pitch_target_rad)
{
    yaw_step_size = step_size_s;
    pitch_step_size = step_size_s;

    /*
     * Fill u/y arrays, initialize the submodels, then call CalculateSubmodel on each loop sample.
     */
    set_yaw_inputs(controller, calibration, initial_encoders, initial_yaw_target_rad);
    set_pitch_inputs(controller, calibration, initial_encoders, initial_pitch_target_rad);

    YawInitializeSubmodel(controller->yaw_u, controller->yaw_y, 0.0);
    PitchInitializeSubmodel(controller->pitch_u, controller->pitch_y, 0.0);
}

ControllerOutput twentysim_controller_step(TwentySimController *controller, const JiwyCalibration *calibration, EncoderSample encoders, double yaw_target_rad, double pitch_target_rad)
{
    ControllerOutput output;

    set_yaw_inputs(controller, calibration, encoders, yaw_target_rad);
    YawCalculateSubmodel(controller->yaw_u, controller->yaw_y, yaw_time);

    set_pitch_inputs(controller, calibration, encoders, pitch_target_rad);
    PitchCalculateSubmodel(controller->pitch_u, controller->pitch_y, pitch_time);

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