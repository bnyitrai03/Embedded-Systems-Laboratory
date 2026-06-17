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
 *
 * These constants are the single tuning surface for the green-ball tracker.
 * The tracker scans the full RGB frame, keeps the largest valid connected
 * green blob, then converts its center offset into yaw/pitch camera errors.
 */
#define JIWY_VISION_DEFAULT_CAMERA "/dev/video0"
#define JIWY_VISION_DEBUG_ENABLED 1
#define JIWY_VISION_STREAM_ENABLED 1
#define JIWY_VISION_STREAM_PORT 8080
#define JIWY_VISION_FRAME_WIDTH 640
#define JIWY_VISION_FRAME_HEIGHT 480
#define JIWY_VISION_FRAME_RATE 30

/*
 * Reject selected blobs smaller than this many green pixels. Increase this if
 * small green noise is still accepted as the ball. Decrease it if the real ball
 * is far away or partly hidden and disappears.
 */
#define JIWY_VISION_MIN_GREEN_PIXELS 150

/*
 * Minimum selected blob dimensions. Increase to ignore thin reflections or
 * dots. Decrease only if the ball becomes very small in the camera image.
 */
#define JIWY_VISION_BLOB_MIN_WIDTH 30
#define JIWY_VISION_BLOB_MIN_HEIGHT 30

/*
 * Minimum percent of the bounding box filled by green pixels. Increase this to
 * reject sparse speckles. Decrease it if shadows/highlights make the ball mask
 * hollow or incomplete.
 */
#define JIWY_VISION_BLOB_MIN_FILL_PERCENT 25

/*
 * Maximum width/height ratio in percent. 220 accepts moderately elliptical
 * perspective views while rejecting long strips.
 */
#define JIWY_VISION_BLOB_MAX_ASPECT_PERCENT 220

/*
 * Exponential smoothing alpha for the chosen blob center. Larger follows fast
 * motion more closely but is noisier. Smaller is steadier but lags the ball.
 */
#define JIWY_VISION_TRACK_SMOOTHING_ALPHA 0.70

/*
 * If lock-to-previous is enabled in code, reject sudden blob jumps larger than
 * this many pixels until the tracker has been lost long enough to reset.
 */
#define JIWY_VISION_TRACK_MAX_JUMP_PIXELS 200.0
#define JIWY_VISION_TRACK_RESET_LOST_FRAMES 10u
/*
 * Control-loop samples to keep using the last valid camera frame before
 * holding current position. At 5 ms control period, 25 samples is 125 ms.
 */
#define JIWY_VISION_MAX_STALE_CONTROL_SAMPLES 25u

/*
 * Logitech C270 is commonly advertised as 60 deg diagonal. For a 4:3 640x480
 * frame, that corresponds to about 49.6 deg horizontal and 39.7 deg vertical.
 * Increase these if the robot under-rotates for a measured pixel offset.
 * Decrease them if it over-rotates.
 */
#define JIWY_VISION_HORIZONTAL_FOV_RAD (49.6 * JIWY_PI / 180.0)
#define JIWY_VISION_VERTICAL_FOV_RAD   (39.7 * JIWY_PI / 180.0)

/*
 * Camera-error deadband. Errors smaller than this become zero so the robot does
 * not twitch when the ball is visually centered. Increase if it still hunts at
 * center. Decrease if it stops before the ball is centered accurately enough.
 */
#define JIWY_VISION_YAW_DEADBAND_RAD   0.01
#define JIWY_VISION_PITCH_DEADBAND_RAD 0.01

/*
 * Publish only every Nth camera frame to the browser debug stream. With a
 * 30 fps camera, 6 gives about 5 fps and keeps HTTP debug work cheap.
 */
#define JIWY_VISION_STREAM_FPS_DIVISOR 6
#define JIWY_VISION_DEBUG_EVERY_FRAMES 30

/*
 * Pixel-level green threshold.
 *
 * GREEN_MIN_CHANNEL: minimum dominant green channel. Increase to ignore dark
 * pixels. Decrease if the ball is detected poorly in dim light.
 *
 * GREEN_MIN_DELTA: minimum separation between strongest and weakest RGB
 * channels. Increase to reject gray/white glare. Decrease if real green pixels
 * are muted by lighting or camera exposure.
 */
#define JIWY_VISION_GREEN_MIN_CHANNEL 20
#define JIWY_VISION_GREEN_MIN_DELTA 12

/*
 * Whiteness is min(R,G,B) as percent of 255. Low values allow saturated colors;
 * high values allow pale glare. Keep min at 0 for the lab ball unless black
 * background noise starts matching.
 */
#define JIWY_VISION_GREEN_MIN_WHITENESS_PERCENT 0
#define JIWY_VISION_GREEN_MAX_WHITENESS_PERCENT 60

/*
 * Blackness is (255 - max(R,G,B)) as percent of 255. Raising the max allows
 * darker green pixels, useful under shadows. Lower it if dark noise is accepted.
 */
#define JIWY_VISION_GREEN_MIN_BLACKNESS_PERCENT 0
#define JIWY_VISION_GREEN_MAX_BLACKNESS_PERCENT 95
#define JIWY_VISION_BMP_HEADER_SIZE 54
#define JIWY_VISION_STREAM_BOUNDARY "jiwyframe"

/*
 * Hue window in degrees around green. Widen if the ball changes color with
 * lighting; narrow if yellow/cyan background objects are selected.
 */
#define HUE_LOWER_LIMIT 80
#define HUE_UPPER_LIMIT 180

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
