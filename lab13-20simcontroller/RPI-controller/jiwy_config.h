#ifndef JIWY_CONFIG_H
#define JIWY_CONFIG_H

#define JIWY_PI 3.14159265358979323846

/*
 * Raspberry Pi SPI backend defaults.
 */
#define JIWY_SPI_DEFAULT_SPEED_HZ 100000u
#define JIWY_SPI_DEFAULT_CHANNEL 1u
#define JIWY_SPI_DEFAULT_FLAGS 0u

/*
 * Control loop defaults.
 */
#define JIWY_CONTROL_LOOP_SAMPLE_PERIOD_US 5000u
#define JIWY_CONTROL_LOOP_STATUS_EVERY 0u
#define JIWY_CONTROL_LOOP_DEFAULT_CSV_LOG_PATH ((const char *)0)

/**
 * @brief Maximum PWM duty value sent by the C controller.
 *
 * The FPGA PWM period is 2500 ticks at 50 MHz / 20 kHz, but the controller
 * intentionally limits commands to 250 ticks for safer lab bring-up.
 */
#define MOTOR_PWM_MAX 512u

/*
 * Homing defaults.
 */
#define JIWY_HOMING_PWM (MOTOR_PWM_MAX/4)
#define JIWY_HOMING_YAW_HOME_DIRECTION 0u
#define JIWY_HOMING_PITCH_HOME_DIRECTION 0u
#define JIWY_HOMING_SAMPLE_PERIOD_US 10000u
#define JIWY_HOMING_STOP_WINDOW_SAMPLES 50u
#define JIWY_HOMING_MOVEMENT_THRESHOLD_COUNTS 2u
#define JIWY_HOMING_MAX_SAMPLES_PER_AXIS 1000u
#define JIWY_HOMING_SETTLE_SAMPLES 20u

/*
 * Vision defaults.
 */
#define JIWY_VISION_DEFAULT_CAMERA "/dev/video0"
#define JIWY_VISION_DEBUG_ENABLED 0
#define JIWY_VISION_STREAM_ENABLED 0
#define JIWY_VISION_STREAM_PORT 8080
#define JIWY_VISION_FRAME_WIDTH 320
#define JIWY_VISION_FRAME_HEIGHT 240
#define JIWY_VISION_FRAME_RATE 30
#define JIWY_VISION_MIN_GREEN_PIXELS 80
#define JIWY_VISION_BLOB_MIN_WIDTH 6
#define JIWY_VISION_BLOB_MIN_HEIGHT 6
#define JIWY_VISION_BLOB_MIN_FILL_PERCENT 20
#define JIWY_VISION_BLOB_MAX_ASPECT_PERCENT 250
#define JIWY_VISION_TRACK_SMOOTHING_ALPHA 0.35
#define JIWY_VISION_TRACK_MAX_JUMP_PIXELS 80.0
#define JIWY_VISION_TRACK_RESET_LOST_FRAMES 5u
#define JIWY_VISION_FOV_RAD (60.0 * JIWY_PI / 180.0)
#define JIWY_VISION_STREAM_FPS_DIVISOR 3
#define JIWY_VISION_DEBUG_EVERY_FRAMES 30
#define JIWY_VISION_GREEN_MIN_CHANNEL 70
#define JIWY_VISION_GREEN_MIN_DELTA 20
#define JIWY_VISION_GREEN_MIN_WHITENESS_PERCENT 20
#define JIWY_VISION_GREEN_MAX_WHITENESS_PERCENT 45
#define JIWY_VISION_GREEN_MIN_BLACKNESS_PERCENT 35
#define JIWY_VISION_GREEN_MAX_BLACKNESS_PERCENT 65
#define JIWY_VISION_BMP_HEADER_SIZE 54
#define JIWY_VISION_STREAM_BOUNDARY "jiwyframe"
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
#define JIWY_YAW_MAX_RAD   (JIWY_YAW_TRAVEL_DEGREES * JIWY_PI / 180.0)

#define JIWY_PITCH_MIN_RAD 0.0
#define JIWY_PITCH_MAX_RAD (JIWY_PITCH_TRAVEL_DEGREES * JIWY_PI / 180.0)

/*
 * Hold-mode PID calibration schedule.
 *
 * --hold alternates between these two configured targets.
 */
#define JIWY_HOLD_TARGET1_YAW_RAD   (JIWY_YAW_MAX_RAD * 0.25)
#define JIWY_HOLD_TARGET1_PITCH_RAD (JIWY_PITCH_MAX_RAD * 0.25)
#define JIWY_HOLD_TARGET1_DURATION_S 10.0

#define JIWY_HOLD_TARGET2_YAW_RAD   (JIWY_YAW_MAX_RAD * 0.75)
#define JIWY_HOLD_TARGET2_PITCH_RAD (JIWY_PITCH_MAX_RAD * 0.75)
#define JIWY_HOLD_TARGET2_DURATION_S 10.0

#define JIWY_HOLD_SCHEDULE_REPEAT 1

#endif
