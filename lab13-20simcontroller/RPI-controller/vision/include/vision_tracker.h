#ifndef VISION_TRACKER_H
#define VISION_TRACKER_H

#include <pthread.h>
#include <stdint.h>
#include <stddef.h>

typedef struct _GMainLoop GMainLoop;
typedef struct _GstElement GstElement;

/**
 * @file vision_tracker.h
 * @brief Lightweight green-object tracker for live yaw/pitch target updates.
 *
 * The tracker runs the GStreamer camera pipeline on a separate thread. The
 * control loop reads a small mutex-protected snapshot containing the latest
 * angular error between the camera centerline and the green object centroid.
 */

typedef struct VisionTargetSnapshot {
    /** Non-zero when the current frame contains enough green pixels. */
    int valid;
    /** Positive when the green object is right of image center. */
    double yaw_error_rad;
    /** Positive when the green object is above image center. */
    double pitch_error_rad;
    /** Camera frame number associated with this snapshot. */
    uint64_t frame_count;
    /** Milliseconds since the previous processed frame, or 0 for the first. */
    double frame_interval_ms;
    /** Microseconds spent processing this frame. */
    double process_us;
    /** Non-zero when the frame interval is unusually late. */
    int late_frame;
} VisionTargetSnapshot;

#include "vision_stream.h"

typedef struct {
    pthread_t thread;
    pthread_mutex_t lock;
    pthread_mutex_t state_lock;
    pthread_cond_t state_cond;
    VisionTargetSnapshot latest;
    GMainLoop *loop;
    GstElement *pipeline;
    const char *camera_device;
    VisionStream stream;
    int debug_enabled;
    int thread_started;
    int start_done;
    int start_result;
} VisionTracker;

/**
 * @brief Initialize tracker storage before starting the camera thread.
 */
void vision_tracker_init(VisionTracker *tracker);

/**
 * @brief Start the MJPEG camera tracker thread.
 *
 * @param camera_device V4L2 device path, or NULL for the configured default.
 * @param debug_enabled Non-zero prints low-rate camera diagnostics.
 * @return 0 on successful thread/pipeline start, negative errno-style value.
 */
int vision_tracker_start(VisionTracker *tracker,
                         const char *camera_device,
                         int debug_enabled,
                         int stream_enabled,
                         int stream_port);

/**
 * @brief Stop the camera thread and release GStreamer resources.
 */
void vision_tracker_stop(VisionTracker *tracker);

/**
 * @brief Copy the latest camera target snapshot.
 *
 * @return 0 on success or -EINVAL for invalid arguments.
 */
int vision_tracker_read_latest(VisionTracker *tracker,
                               VisionTargetSnapshot *snapshot);

#endif
