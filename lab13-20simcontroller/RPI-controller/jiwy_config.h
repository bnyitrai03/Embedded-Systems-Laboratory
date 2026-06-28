#ifndef JIWY_CONFIG_H
#define JIWY_CONFIG_H

#define JIWY_PI 3.14159265358979323846

/*
 * Raspberry Pi SPI defaults.
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
 * The FPGA PWM period is 2500 ticks at 50 MHz / 20 kHz. The controller limits
 * commands to MOTOR_PWM_MAX ticks for lab safety.
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
 * Reject selected blobs smaller than this many green pixels.
 */
#define JIWY_VISION_MIN_GREEN_PIXELS 100

/*
 * Minimum selected blob dimensions.
 */
#define JIWY_VISION_BLOB_MIN_WIDTH 30
#define JIWY_VISION_BLOB_MIN_HEIGHT 30

/*
 * Minimum percent of the bounding box filled by green pixels.
 */
#define JIWY_VISION_BLOB_MIN_FILL_PERCENT 40

/*
 * Maximum width/height ratio in percent. 220 accepts moderately elliptical
 * perspective views while rejecting long strips.
 */
#define JIWY_VISION_BLOB_MAX_ASPECT_PERCENT 220

/*
 * Exponential smoothing alpha for the chosen blob center. Larger follows fast
 * motion more closely but is noisier. Smaller is steadier but lags the ball.
 */
#define JIWY_VISION_TRACK_SMOOTHING_ALPHA 0.60

/*
 * If lock-to-previous is enabled in code, reject sudden blob jumps larger than
 * this many pixels until the tracker has been lost long enough to reset.
 */
#define JIWY_VISION_TRACK_MAX_JUMP_PIXELS 200.0
#define JIWY_VISION_TRACK_RESET_LOST_FRAMES 10u
/*
 * Scale camera error before creating the robot setpoint.
 */
#define JIWY_VISION_TARGET_GAIN 1.0

/*
 * Maximum vision target movement per control sample. This turns camera
 * frame-rate target jumps into a smooth ramp for the PID.
 */
#define JIWY_VISION_TARGET_SLEW_RAD_PER_SAMPLE 0.01

#define JIWY_VISION_HORIZONTAL_FOV_RAD (40.0 * JIWY_PI / 180.0)
#define JIWY_VISION_VERTICAL_FOV_RAD   (30.0 * JIWY_PI / 180.0)

/*
 * Camera-error deadband. Errors smaller than this become zero so the robot does
 * not twitch when the ball is visually centered.
 */
#define JIWY_VISION_YAW_DEADBAND_RAD   0.0
#define JIWY_VISION_PITCH_DEADBAND_RAD 0.06

/*
 * Publish only every Nth camera frame to the browser debug stream. With a
 * 30 fps camera, 3 gives about 10 fps and keeps HTTP debug work cheap.
 */
#define JIWY_VISION_STREAM_FPS_DIVISOR 3
#define JIWY_VISION_DEBUG_EVERY_FRAMES 30

/*
 * A frame is flagged "late" when its interval exceeds this percentage of the
 * nominal frame period. 150 corresponds to 1.5x the configured frame rate.
 */
#define JIWY_VISION_LATE_FRAME_THRESHOLD_PCT 150

/*
 * Seconds to wait for the GStreamer camera pipeline to reach the PLAYING state
 * before reporting a startup failure.
 */
#define JIWY_VISION_PIPELINE_READY_TIMEOUT_S 5

/*
 * Pixel-level green threshold.
 *
 * GREEN_MIN_CHANNEL: minimum dominant green channel.
 * GREEN_MIN_DELTA: minimum separation between strongest and weakest RGB
 * channels.
 */
#define JIWY_VISION_GREEN_MIN_CHANNEL 20
#define JIWY_VISION_GREEN_MIN_DELTA 35

/*
 * Whiteness is min(R,G,B) as percent of 255. Low values allow saturated colors;
 * high values allow pale glare.
 */
#define JIWY_VISION_GREEN_MIN_WHITENESS_PERCENT 0
#define JIWY_VISION_GREEN_MAX_WHITENESS_PERCENT 60

/*
 * Blackness is (255 - max(R,G,B)) as percent of 255.
 */
#define JIWY_VISION_GREEN_MIN_BLACKNESS_PERCENT 0
#define JIWY_VISION_GREEN_MAX_BLACKNESS_PERCENT 95
#define JIWY_VISION_BMP_HEADER_SIZE 54
#define JIWY_VISION_STREAM_BOUNDARY "jiwyframe"

/*
 * Hue window in degrees around green.
 */
#define HUE_LOWER_LIMIT 100
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
