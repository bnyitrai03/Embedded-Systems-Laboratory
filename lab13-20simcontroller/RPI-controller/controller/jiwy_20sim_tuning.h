#ifndef JIWY_20SIM_TUNING_H
#define JIWY_20SIM_TUNING_H

/**
 * @brief PID and output-limit constants used by generated 20-sim controllers.
*/

#define JIWY_YAW_CORR_GAIN 0.0
#define JIWY_YAW_KP 0.28
#define JIWY_YAW_TAU_D 0.1
#define JIWY_YAW_BETA 0.0
#define JIWY_YAW_TAU_I 0.2
#define JIWY_YAW_OUTPUT_MIN -1.0
#define JIWY_YAW_OUTPUT_MAX 1.0


#define JIWY_PITCH_CORR_GAIN 0.0
#define JIWY_PITCH_KP 0.31
#define JIWY_PITCH_TAU_D 0.21
#define JIWY_PITCH_BETA 0.0
#define JIWY_PITCH_TAU_I 0.2
#define JIWY_PITCH_OUTPUT_MIN -1.0
#define JIWY_PITCH_OUTPUT_MAX 1.0

#endif
