#ifndef JIWY_CONFIG_H
#define JIWY_CONFIG_H




#define JIWY_PI 3.14159265358979323846


/**
 * @brief Maximum PWM duty value sent by the C controller.
 *
 * The FPGA PWM period is 2500 ticks at 50 MHz / 20 kHz, but the controller
 * intentionally limits commands to 250 ticks for safer lab bring-up.
 */
#define MOTOR_PWM_MAX 512u

/**
 * @brief Control loop period amount. Unit is microseconds
 */
#define CONTROLLER_SAMPLE_PERIOD 3000u



#define HUE_LOWER_LIMIT 100

#define HUE_UPPER_LIMIT 200
/*
 * Physical angular travel between the two mechanical stops.
 * Encoder count travel is measured during homing.
 */
#define JIWY_YAW_TRAVEL_DEGREES 240.0

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
