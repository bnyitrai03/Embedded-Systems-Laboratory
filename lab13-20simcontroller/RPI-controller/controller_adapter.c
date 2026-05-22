#include "controller_adapter.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>

#include "controller/pitch_controller.h"
#include "controller/yaw_controller.h"
#include "jiwy_config.h"

enum {
    /* Generated yaw_controller.c variable indices. */
    YAW_IN = 7,
    YAW_POSITION = 8,
    YAW_OUT = 9
};

enum {
    /* Generated pitch_controller.c variable indices. */
    PITCH_CORR = 8,
    PITCH_IN = 9,
    PITCH_POSITION = 10,
    PITCH_OUT = 11
};

enum {
    /* Same parameter order in both generated 20-sim controllers. */
    PARAM_CORR_GAIN = 0,
    PARAM_KP = 1,
    PARAM_TAU_D = 2,
    PARAM_BETA = 3,
    PARAM_TAU_I = 4,
    PARAM_OUTPUT_MIN = 5,
    PARAM_OUTPUT_MAX = 6
};

JiwyCalibration jiwy_default_calibration(void)
{
    JiwyCalibration calibration;

    calibration.yaw_counts_per_rad =
        JIWY_YAW_TRAVEL_COUNTS / (JIWY_YAW_TRAVEL_DEGREES * JIWY_PI / 180.0);
    calibration.pitch_counts_per_rad =
        JIWY_PITCH_TRAVEL_COUNTS / (JIWY_PITCH_TRAVEL_DEGREES * JIWY_PI / 180.0);
    calibration.yaw_home_count = 0;
    calibration.pitch_home_count = 0;
    calibration.yaw_min_rad = JIWY_YAW_MIN_RAD;
    calibration.yaw_max_rad = JIWY_YAW_MAX_RAD;
    calibration.pitch_min_rad = JIWY_PITCH_MIN_RAD;
    calibration.pitch_max_rad = JIWY_PITCH_MAX_RAD;
    return calibration;
}

void controller_20sim_init(double step_size_s)
{
    YawModelInitialize();
    XXModelInitialize();

    /*
     * Keep the generated C files replaceable: tune by changing jiwy_config.h,
     * then copy those values into the generated parameter arrays here.
     */
    yaw_P[PARAM_CORR_GAIN] = JIWY_YAW_CORR_GAIN;
    yaw_P[PARAM_KP] = JIWY_YAW_KP;
    yaw_P[PARAM_TAU_D] = JIWY_YAW_TAU_D;
    yaw_P[PARAM_BETA] = JIWY_YAW_BETA;
    yaw_P[PARAM_TAU_I] = JIWY_YAW_TAU_I;
    yaw_P[PARAM_OUTPUT_MIN] = JIWY_YAW_OUTPUT_MIN;
    yaw_P[PARAM_OUTPUT_MAX] = JIWY_YAW_OUTPUT_MAX;

    pitch_P[PARAM_CORR_GAIN] = JIWY_PITCH_CORR_GAIN;
    pitch_P[PARAM_KP] = JIWY_PITCH_KP;
    pitch_P[PARAM_TAU_D] = JIWY_PITCH_TAU_D;
    pitch_P[PARAM_BETA] = JIWY_PITCH_BETA;
    pitch_P[PARAM_TAU_I] = JIWY_PITCH_TAU_I;
    pitch_P[PARAM_OUTPUT_MIN] = JIWY_PITCH_OUTPUT_MIN;
    pitch_P[PARAM_OUTPUT_MAX] = JIWY_PITCH_OUTPUT_MAX;

    yaw_step_size = step_size_s;
    pitch_step_size = step_size_s;
    yaw_time = 0.0;
    pitch_time = 0.0;
    yaw_steps = 0;
    pitch_steps = 0;
}

double jiwy_yaw_rad(const JiwyCalibration *calibration, int16_t encoder_count)
{
    return ((double)encoder_count - (double)calibration->yaw_home_count) /
           calibration->yaw_counts_per_rad;
}

double jiwy_pitch_rad(const JiwyCalibration *calibration, int16_t encoder_count)
{
    return ((double)encoder_count - (double)calibration->pitch_home_count) /
           calibration->pitch_counts_per_rad;
}

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

static void commit_yaw_states(void)
{
    int i;

    /*
     * The generated models store "next state" values in R[]. For this discrete
     * controller we commit them once per sample after CalculateDynamic/Output.
     */
    for (i = 0; i < yaw_states_size; ++i) {
        yaw_s[i] = yaw_R[i];
    }
    yaw_time += yaw_step_size;
}

static void commit_pitch_states(void)
{
    int i;

    /* See commit_yaw_states; pitch uses the same generated-state convention. */
    for (i = 0; i < pitch_states_size; ++i) {
        pitch_s[i] = pitch_R[i];
    }
    pitch_time += pitch_step_size;
}

ControllerOutput controller_20sim_step(const JiwyCalibration *calibration,
                                       EncoderSample encoders,
                                       double yaw_target_rad,
                                       double pitch_target_rad,
                                       double pitch_correction_rad)
{
    ControllerOutput output;

    yaw_V[YAW_IN] = clamp(yaw_target_rad,
                          calibration->yaw_min_rad,
                          calibration->yaw_max_rad);
    yaw_V[YAW_POSITION] = jiwy_yaw_rad(calibration, encoders.yaw);
    YawCalculateDynamic();
    YawCalculateOutput();
    commit_yaw_states();

    pitch_V[PITCH_CORR] = pitch_correction_rad;
    pitch_V[PITCH_IN] = clamp(pitch_target_rad,
                              calibration->pitch_min_rad,
                              calibration->pitch_max_rad);
    pitch_V[PITCH_POSITION] = jiwy_pitch_rad(calibration, encoders.pitch);
    XXCalculateDynamic();
    XXCalculateOutput();
    commit_pitch_states();

    output.yaw = yaw_V[YAW_OUT];
    output.pitch = pitch_V[PITCH_OUT];
    return output;
}

static AxisCommand output_to_axis_command(double output)
{
    AxisCommand command;
    double magnitude = fabs(output);

    if (magnitude > 1.0) {
        magnitude = 1.0;
    }

    /* Positive output means "move toward increasing encoder counts". */
    command.direction = (output >= 0.0) ? MOTOR_DIR_POSITIVE : MOTOR_DIR_NEGATIVE;
    command.enable = magnitude > 0.0;
    command.pwm = (uint16_t)lround(magnitude * MOTOR_PWM_MAX);
    return command;
}

MotorCommand controller_output_to_command(ControllerOutput output)
{
    MotorCommand command;

    command.yaw = output_to_axis_command(output.yaw);
    command.pitch = output_to_axis_command(output.pitch);
    return command;
}
