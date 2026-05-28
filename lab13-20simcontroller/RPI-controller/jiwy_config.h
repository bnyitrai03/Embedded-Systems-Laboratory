#ifndef JIWY_CONFIG_H
#define JIWY_CONFIG_H




#define JIWY_PI 3.14159265358979323846


/**
 * @brief Maximum PWM duty value sent by the C controller.
 *
 * The protocol field is 14 bits, but the current FPGA PWM period is 2500 ticks
 * at 50 MHz / 20 kHz. Clamp commands to that hardware duty range on the C side.
 */
#define MOTOR_PWM_MAX 250u


/*
 * Measured by driving each axis from one mechanical limit to the other.
 * Tune these values per physical JIWY setup.
 */
#define JIWY_YAW_TRAVEL_COUNTS 10861.0
#define JIWY_YAW_TRAVEL_DEGREES 240.0

#define JIWY_PITCH_TRAVEL_COUNTS 13569.0
#define JIWY_PITCH_TRAVEL_DEGREES 240.0


/*
 * Software limits are relative to the software home found during homing.
 * The default assumes homing moves to the negative mechanical end stop.
 */
#define JIWY_YAW_MIN_RAD 0.0
#define JIWY_YAW_MAX_RAD \
    (JIWY_YAW_TRAVEL_DEGREES * JIWY_PI / 180.0)

#define JIWY_PITCH_MIN_RAD 0.0
#define JIWY_PITCH_MAX_RAD \
    (JIWY_PITCH_TRAVEL_DEGREES * JIWY_PI / 180.0)

#endif
