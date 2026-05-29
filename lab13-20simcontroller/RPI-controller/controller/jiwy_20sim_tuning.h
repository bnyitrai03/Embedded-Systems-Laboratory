#ifndef JIWY_20SIM_TUNING_H
#define JIWY_20SIM_TUNING_H

/**
 * @file jiwy_20sim_tuning.h
 * @brief PID and output-limit constants used by generated 20-sim controllers.
 *
 * This header is hand-owned lab tuning code. The generated yaw_model.c and
 * pitch_model.c files include it from their parameter initializer functions so
 * tuning values are active during generated submodel initialization.
 *
 * If the 20-sim controller code is regenerated, reapply the small generated
 * changes that include this header and use these macros in
 * YawModelInitialize_parameters() and PitchModelInitialize_parameters().
 */

#define JIWY_YAW_CORR_GAIN 0.0
#define JIWY_YAW_KP 0.30
#define JIWY_YAW_TAU_D 0.01
#define JIWY_YAW_BETA 0.0
#define JIWY_YAW_TAU_I 4.0
#define JIWY_YAW_OUTPUT_MIN -0.5
#define JIWY_YAW_OUTPUT_MAX 0.5


#define JIWY_PITCH_CORR_GAIN 0.0
#define JIWY_PITCH_KP 0.35
#define JIWY_PITCH_TAU_D 0.01
#define JIWY_PITCH_BETA 0.0
#define JIWY_PITCH_TAU_I 8.0
#define JIWY_PITCH_OUTPUT_MIN -0.5
#define JIWY_PITCH_OUTPUT_MAX 0.5

#endif
