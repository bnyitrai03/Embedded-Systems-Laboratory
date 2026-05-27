#ifndef JIWY_CONFIG_H
#define JIWY_CONFIG_H

/**
 * @file jiwy_config.h
 * @brief Lab-tunable calibration, travel limit, and PID constants.
 *
 * Edit this file when remeasuring the hardware or retuning the generated
 * 20-sim controllers. Generated 20-sim source files should remain unchanged.
 */

/** @brief Local pi constant used by preprocessor travel-limit expressions. */
#define JIWY_PI 3.14159265358979323846

/**
 * @name Encoder travel calibration
 *
 * Measured by driving each axis from one mechanical limit to the other.
 * Tune these values per physical JIWY setup.
 * @{
 */
#define JIWY_YAW_TRAVEL_COUNTS 2453.0
#define JIWY_YAW_TRAVEL_DEGREES 180.0

#define JIWY_PITCH_TRAVEL_COUNTS 684.0
#define JIWY_PITCH_TRAVEL_DEGREES 175.0
/** @} */

/**
 * @name Generated 20-sim PID tuning
 *
 * 20-sim controller parameters.
 * These are copied into the generated yaw_P[] and pitch_P[] arrays after
 * model initialization, so the generated controller files can stay untouched.
 * @{
 */
#define JIWY_YAW_CORR_GAIN 0.0
#define JIWY_YAW_KP 2.6
#define JIWY_YAW_TAU_D 0.05
#define JIWY_YAW_BETA 0.17
#define JIWY_YAW_TAU_I 9.0
#define JIWY_YAW_OUTPUT_MIN -0.99
#define JIWY_YAW_OUTPUT_MAX 0.99

#define JIWY_PITCH_CORR_GAIN 0.0
#define JIWY_PITCH_KP 1.6
#define JIWY_PITCH_TAU_D 0.05
#define JIWY_PITCH_BETA 0.001
#define JIWY_PITCH_TAU_I 10.5
#define JIWY_PITCH_OUTPUT_MIN -0.99
#define JIWY_PITCH_OUTPUT_MAX 0.99
/** @} */

/**
 * @name Software travel limits
 *
 * Software limits are relative to the software home found during homing.
 * The default assumes homing moves to the negative mechanical end stop.
 * @{
 */
#define JIWY_YAW_MIN_RAD 0.0
#define JIWY_YAW_MAX_RAD \
    (JIWY_YAW_TRAVEL_DEGREES * JIWY_PI / 180.0)

#define JIWY_PITCH_MIN_RAD 0.0
#define JIWY_PITCH_MAX_RAD \
    (JIWY_PITCH_TRAVEL_DEGREES * JIWY_PI / 180.0)
/** @} */

#endif
