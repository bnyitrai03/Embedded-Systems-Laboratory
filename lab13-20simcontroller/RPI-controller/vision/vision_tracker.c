#include "vision_tracker.h"

#include <errno.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <gst/app/gstappsink.h>
#include <gst/video/video.h>
#include "jiwy_config.h"
#include "vision_blob.h"
#include "vision_geometry.h"

typedef struct {
    VisionTracker *tracker;
    VisionBlobTracker blob_tracker;
    uint64_t frame_count;
    struct timespec last_frame_time;
    int have_last_frame_time;
} VisionPipelineData;

static long timespec_diff_us(struct timespec end, struct timespec start)
{
    time_t sec = end.tv_sec - start.tv_sec;
    long nsec = end.tv_nsec - start.tv_nsec;

    return (long)sec * 1000000L + nsec / 1000L;
}

static void update_snapshot(VisionTracker *tracker, const VisionTargetSnapshot *snapshot)
{
    pthread_mutex_lock(&tracker->lock);
    tracker->latest = *snapshot;
    pthread_mutex_unlock(&tracker->lock);
}

static void publish_start_result(VisionTracker *tracker, int result)
{
    pthread_mutex_lock(&tracker->state_lock);
    tracker->start_result = result;
    tracker->start_done = 1;
    pthread_cond_signal(&tracker->state_cond);
    pthread_mutex_unlock(&tracker->state_lock);
}

/* ---- frame processing helpers (called from on_new_sample) ---- */

static int acquire_frame_buffer(GstSample *sample, GstBuffer **buffer_out, GstVideoInfo *info)
{
    GstBuffer *buffer = gst_sample_get_buffer(sample);
    GstCaps *caps = gst_sample_get_caps(sample);

    if (buffer == NULL || caps == NULL || !gst_video_info_from_caps(info, caps)) {
        return -1;
    }
    if (GST_VIDEO_INFO_FORMAT(info) != GST_VIDEO_FORMAT_RGB) {
        fprintf(stderr,
                "Vision tracker expected RGB frames, got %s\n",
                GST_VIDEO_INFO_NAME(info));
        return -1;
    }
    *buffer_out = buffer;
    return 0;
}

static int compute_frame_geometry(const GstVideoInfo *info, int *width_out, int *height_out, int *stride_out)
{
    int width = GST_VIDEO_INFO_WIDTH(info);
    int height = GST_VIDEO_INFO_HEIGHT(info);
    int stride = GST_VIDEO_INFO_PLANE_STRIDE(info, 0);

    if (width <= 0 || height <= 0 || (size_t)height > (size_t)INT_MAX / (size_t)width) {
        fprintf(stderr, "Vision tracker got invalid frame dimensions\n");
        return -1;
    }
    if (stride <= 0) {
        return -1;
    }
    *width_out = width;
    *height_out = height;
    *stride_out = stride;
    return 0;
}

static int map_buffer_for_read(GstBuffer *buffer, GstMapInfo *map, int width, int height, int stride)
{
    if (!gst_buffer_map(buffer, map, GST_MAP_READ)) {
        return -1;
    }
    if (map->size < (gsize)((height - 1) * stride + width * 3)) {
        fprintf(stderr, "Vision tracker frame buffer is smaller than expected\n");
        gst_buffer_unmap(buffer, map);
        return -1;
    }
    return 0;
}

static int build_snapshot(VisionPipelineData *data, const VisionBlobDetection *detection, int width, int height, const struct timespec *frame_start, VisionTargetSnapshot *snapshot)
{
    const double nominal_frame_ms = 1000.0 / (double)JIWY_VISION_FRAME_RATE;
    double frame_interval_ms = 0.0;
    struct timespec frame_end;

    memset(snapshot, 0, sizeof(*snapshot));
    snapshot->frame_count = data->frame_count;
    if (data->have_last_frame_time) {
        frame_interval_ms = (double)timespec_diff_us(*frame_start, data->last_frame_time) / 1000.0;
    }
    snapshot->frame_interval_ms = frame_interval_ms;

    if (detection->valid) {
        snapshot->valid = 1;
        vision_pixel_to_camera_error(detection->object_x, detection->object_y, width, height, &snapshot->yaw_error_rad, &snapshot->pitch_error_rad);
    }

    if (clock_gettime(CLOCK_MONOTONIC, &frame_end) < 0) {
        return -1;
    }
    snapshot->process_us = (double)timespec_diff_us(frame_end, *frame_start);
    snapshot->late_frame = data->have_last_frame_time && frame_interval_ms > nominal_frame_ms * JIWY_VISION_LATE_FRAME_THRESHOLD_PCT / 100.0;
    data->last_frame_time = *frame_start;
    data->have_last_frame_time = 1;
    return 0;
}

static void publish_debug_frame(const VisionPipelineData *data, const VisionBlobDetection *detection, const VisionTargetSnapshot *snapshot)
{
    if (snapshot->valid) {
        printf("vision frame=%" G_GUINT64_FORMAT
               " yaw_err=%.4f pitch_err=%.4f blob=%" G_GUINT64_FORMAT
               " green=%" G_GUINT64_FORMAT
               " box=%dx%d"
               " dt=%.2fms proc=%.0fus late=%d\n",
               (guint64)data->frame_count,
               snapshot->yaw_error_rad,
               snapshot->pitch_error_rad,
               (guint64)detection->blob_pixels,
               (guint64)detection->total_green_pixels,
               detection->max_x - detection->min_x + 1,
               detection->max_y - detection->min_y + 1,
               snapshot->frame_interval_ms,
               snapshot->process_us,
               snapshot->late_frame);
    } else {
        printf("vision frame=%" G_GUINT64_FORMAT
               " no green blob green=%" G_GUINT64_FORMAT
               " dt=%.2fms proc=%.0fus late=%d\n",
               (guint64)data->frame_count,
               (guint64)detection->total_green_pixels,
               snapshot->frame_interval_ms,
               snapshot->process_us,
               snapshot->late_frame);
    }
}

static GstFlowReturn on_new_sample(GstAppSink *appsink, gpointer user_data)
{
    VisionPipelineData *data = (VisionPipelineData *)user_data;
    VisionTracker *tracker = data->tracker;
    GstSample *sample = NULL;
    GstBuffer *buffer = NULL;
    GstVideoInfo info;
    GstMapInfo map;
    int mapped = 0;
    int width = 0;
    int height = 0;
    int stride = 0;
    struct timespec frame_start;
    VisionBlobDetection detection;
    VisionTargetSnapshot snapshot;
    GstFlowReturn result = GST_FLOW_ERROR;

    sample = gst_app_sink_pull_sample(appsink);
    if (sample == NULL) {
        return GST_FLOW_ERROR;
    }

    if (clock_gettime(CLOCK_MONOTONIC, &frame_start) < 0) {
        goto cleanup;
    }
    if (acquire_frame_buffer(sample, &buffer, &info) < 0) {
        goto cleanup;
    }
    if (compute_frame_geometry(&info, &width, &height, &stride) < 0) {
        goto cleanup;
    }
    if (map_buffer_for_read(buffer, &map, width, height, stride) < 0) {
        goto cleanup;
    }
    mapped = 1;

    if (vision_blob_tracker_process_rgb(&data->blob_tracker, map.data, width, height, stride, &detection) < 0) {
        goto cleanup;
    }

    if (build_snapshot(data, &detection, width, height, &frame_start, &snapshot) < 0) {
        goto cleanup;
    }

    update_snapshot(tracker, &snapshot);

    if (tracker->stream.enabled &&
        data->frame_count % JIWY_VISION_STREAM_FPS_DIVISOR == 0) {
        vision_stream_publish_rgb(&tracker->stream, map.data, width, height, stride, &detection, &snapshot);
    }

    if (tracker->debug_enabled &&
        data->frame_count % JIWY_VISION_DEBUG_EVERY_FRAMES == 0) {
        publish_debug_frame(data, &detection, &snapshot);
    }

    ++data->frame_count;
    result = GST_FLOW_OK;

cleanup:
    if (mapped) {
        gst_buffer_unmap(buffer, &map);
    }
    if (sample != NULL) {
        gst_sample_unref(sample);
    }
    return result;
}

static gboolean on_bus_message(GstBus *bus, GstMessage *message, gpointer user_data)
{
    VisionPipelineData *data = (VisionPipelineData *)user_data;

    (void)bus;

    switch (GST_MESSAGE_TYPE(message)) {
    case GST_MESSAGE_ERROR: {
        GError *error = NULL;
        gchar *debug = NULL;

        gst_message_parse_error(message, &error, &debug);
        fprintf(stderr,
                "Vision tracker GStreamer error: %s\n",
                error != NULL ? error->message : "unknown");
        fprintf(stderr,
                "Vision tracker debug info: %s\n",
                debug != NULL ? debug : "none");
        g_clear_error(&error);
        g_free(debug);
        g_main_loop_quit(data->tracker->loop);
        break;
    }
    case GST_MESSAGE_EOS:
        g_main_loop_quit(data->tracker->loop);
        break;
    default:
        break;
    }

    return TRUE;
}

/* ---- pipeline construction helpers ---- */

static void prepare_pipeline_data(VisionPipelineData *data, VisionTracker *tracker)
{
    memset(data, 0, sizeof(*data));
    data->tracker = tracker;
    vision_blob_tracker_init(&data->blob_tracker);
}

static GstCaps *create_rgb_caps(void)
{
    return gst_caps_new_simple("video/x-raw",
                               "format", G_TYPE_STRING, "RGB",
                               "width", G_TYPE_INT, JIWY_VISION_FRAME_WIDTH,
                               "height", G_TYPE_INT, JIWY_VISION_FRAME_HEIGHT,
                               "framerate", GST_TYPE_FRACTION,
                               JIWY_VISION_FRAME_RATE, 1,
                               NULL);
}

static void configure_sink(GstElement *sink, VisionPipelineData *data)
{
    g_object_set(sink, "emit-signals", TRUE, "sync", FALSE, "max-buffers", 1, "drop", TRUE, NULL);
    g_signal_connect(sink, "new-sample", G_CALLBACK(on_new_sample), data);
}

/* Runs the built pipeline to completion: bus watch, PLAYING, ready check,
 * main loop, and teardown. tracker->pipeline must already be set on
 * entry; on return tracker->pipeline and tracker->loop are NULL and the
 * pipeline has been unreferenced. The blob tracker is destroyed by the caller.
 * Publishes the start result to the waiting controller thread.
 */
static void run_pipeline_until_quit(VisionTracker *tracker, VisionPipelineData *data)
{
    GstElement *pipeline = tracker->pipeline;
    GstBus *bus;
    guint bus_watch_id;
    GstStateChangeReturn state_result;
    GstStateChangeReturn ready_result;

    tracker->loop = g_main_loop_new(NULL, FALSE);

    bus = gst_pipeline_get_bus(GST_PIPELINE(pipeline));
    bus_watch_id = gst_bus_add_watch(bus, on_bus_message, data);
    gst_object_unref(bus);

    state_result = gst_element_set_state(pipeline, GST_STATE_PLAYING);
    if (state_result == GST_STATE_CHANGE_FAILURE) {
        fprintf(stderr, "Vision tracker failed to start camera pipeline\n");
        g_source_remove(bus_watch_id);
        g_main_loop_unref(tracker->loop);
        tracker->loop = NULL;
        gst_object_unref(pipeline);
        tracker->pipeline = NULL;
        publish_start_result(tracker, -EIO);
        return;
    }

    ready_result = gst_element_get_state(pipeline, NULL, NULL, JIWY_VISION_PIPELINE_READY_TIMEOUT_S * GST_SECOND);
    if (ready_result == GST_STATE_CHANGE_FAILURE) {
        fprintf(stderr, "Vision tracker camera pipeline did not reach PLAYING\n");
        gst_element_set_state(pipeline, GST_STATE_NULL);
        g_source_remove(bus_watch_id);
        g_main_loop_unref(tracker->loop);
        tracker->loop = NULL;
        gst_object_unref(pipeline);
        tracker->pipeline = NULL;
        publish_start_result(tracker, -EIO);
        return;
    }

    publish_start_result(tracker, 0);
    g_main_loop_run(tracker->loop);

    gst_element_set_state(pipeline, GST_STATE_NULL);
    g_source_remove(bus_watch_id);
    g_main_loop_unref(tracker->loop);
    tracker->loop = NULL;
    gst_object_unref(pipeline);
    tracker->pipeline = NULL;
}

/* ---- platform specific thread entry points ---- */

#ifndef JIWY_VISION_USE_AVFVIDEOSRC
static void *vision_thread_main_v4l2(void *arg)
{
    VisionTracker *tracker = (VisionTracker *)arg;
    VisionPipelineData data;
    GstElement *pipeline;
    GstElement *source;
    GstElement *source_capsfilter;
    GstElement *jpegdec;
    GstElement *convert;
    GstElement *scale;
    GstElement *rgb_capsfilter;
    GstElement *sink;
    GstCaps *source_caps;
    GstCaps *rgb_caps;

    gst_init(NULL, NULL);
    prepare_pipeline_data(&data, tracker);

    pipeline = gst_pipeline_new("jiwy-vision-tracker");
    source = gst_element_factory_make("v4l2src", "source");
    source_capsfilter = gst_element_factory_make("capsfilter", "source-caps");
    jpegdec = gst_element_factory_make("jpegdec", "jpeg-decoder");
    convert = gst_element_factory_make("videoconvert", "rgb-convert");
    scale = gst_element_factory_make("videoscale", "video-scale");
    rgb_capsfilter = gst_element_factory_make("capsfilter", "rgb-caps");
    sink = gst_element_factory_make("appsink", "sink");

    if (pipeline == NULL || source == NULL || source_capsfilter == NULL || jpegdec == NULL || convert == NULL || scale == NULL || rgb_capsfilter == NULL || sink == NULL) {
        fprintf(stderr, "Vision tracker could not create GStreamer elements\n");
        if (pipeline != NULL) {
            gst_object_unref(pipeline);
        }
        publish_start_result(tracker, -ENODEV);
        return NULL;
    }

    g_object_set(source, "device", tracker->camera_device, NULL);
    source_caps = gst_caps_new_simple("image/jpeg", "width", G_TYPE_INT, JIWY_VISION_FRAME_WIDTH, "height", G_TYPE_INT, JIWY_VISION_FRAME_HEIGHT, "framerate", GST_TYPE_FRACTION, JIWY_VISION_FRAME_RATE, 1, NULL);
    g_object_set(source_capsfilter, "caps", source_caps, NULL);
    gst_caps_unref(source_caps);

    rgb_caps = create_rgb_caps();
    g_object_set(rgb_capsfilter, "caps", rgb_caps, NULL);
    gst_caps_unref(rgb_caps);

    configure_sink(sink, &data);

    gst_bin_add_many(GST_BIN(pipeline), source, source_capsfilter, jpegdec, convert, scale, rgb_capsfilter, sink, NULL);

    if (!gst_element_link_many(source, source_capsfilter, jpegdec, convert, scale, rgb_capsfilter, sink, NULL)) {
        fprintf(stderr, "Vision tracker could not link MJPEG camera pipeline\n");
        gst_object_unref(pipeline);
        publish_start_result(tracker, -EINVAL);
        return NULL;
    }

    tracker->pipeline = pipeline;
    run_pipeline_until_quit(tracker, &data);
    vision_blob_tracker_destroy(&data.blob_tracker);
    return NULL;
}
#else
static void *vision_thread_main_avf(void *arg)
{
    VisionTracker *tracker = (VisionTracker *)arg;
    VisionPipelineData data;
    GstElement *pipeline;
    GstElement *source;
    GstElement *source_capsfilter;
    GstElement *convert;
    GstElement *scale;
    GstElement *rgb_capsfilter;
    GstElement *sink;
    GstCaps *source_caps;
    GstCaps *rgb_caps;

    gst_init(NULL, NULL);
    prepare_pipeline_data(&data, tracker);

    pipeline = gst_pipeline_new("jiwy-vision-tracker");
    source = gst_element_factory_make("avfvideosrc", "source");
    source_capsfilter = gst_element_factory_make("capsfilter", "source-caps");
    convert = gst_element_factory_make("videoconvert", "rgb-convert");
    scale = gst_element_factory_make("videoscale", "video-scale");
    rgb_capsfilter = gst_element_factory_make("capsfilter", "rgb-caps");
    sink = gst_element_factory_make("appsink", "sink");

    if (pipeline == NULL || source == NULL || source_capsfilter == NULL ||
        convert == NULL || scale == NULL || rgb_capsfilter == NULL ||
        sink == NULL) {
        fprintf(stderr, "Vision tracker could not create GStreamer elements\n");
        if (pipeline != NULL) {
            gst_object_unref(pipeline);
        }
        publish_start_result(tracker, -ENODEV);
        return NULL;
    }

    if (tracker->camera_device != NULL) {
        char *end = NULL;
        long device_index = strtol(tracker->camera_device, &end, 10);

        if (end != tracker->camera_device && *end == '\0' &&
            device_index >= 0 && device_index <= G_MAXINT) {
            g_object_set(source, "device-index", (gint)device_index, NULL);
        }
    }

    source_caps = gst_caps_new_simple("video/x-raw",
                                      "width", G_TYPE_INT, JIWY_VISION_FRAME_WIDTH,
                                      "height", G_TYPE_INT, JIWY_VISION_FRAME_HEIGHT,
                                      "framerate", GST_TYPE_FRACTION,
                                      JIWY_VISION_FRAME_RATE, 1,
                                      NULL);
    g_object_set(source_capsfilter, "caps", source_caps, NULL);
    gst_caps_unref(source_caps);

    rgb_caps = create_rgb_caps();
    g_object_set(rgb_capsfilter, "caps", rgb_caps, NULL);
    gst_caps_unref(rgb_caps);

    configure_sink(sink, &data);

    gst_bin_add_many(GST_BIN(pipeline),
                     source,
                     source_capsfilter,
                     convert,
                     scale,
                     rgb_capsfilter,
                     sink,
                     NULL);

    if (!gst_element_link_many(source,
                               source_capsfilter,
                               convert,
                               scale,
                               rgb_capsfilter,
                               sink,
                               NULL)) {
        fprintf(stderr,
                "Vision tracker could not link macOS camera pipeline\n");
        gst_object_unref(pipeline);
        publish_start_result(tracker, -EINVAL);
        return NULL;
    }

    tracker->pipeline = pipeline;
    run_pipeline_until_quit(tracker, &data);
    vision_blob_tracker_destroy(&data.blob_tracker);
    return NULL;
}
#endif

static void *vision_thread_main(void *arg)
{
#ifdef JIWY_VISION_USE_AVFVIDEOSRC
    return vision_thread_main_avf(arg);
#else
    return vision_thread_main_v4l2(arg);
#endif
}

/* ---- public API (unchanged) ---- */

void vision_tracker_init(VisionTracker *tracker)
{
    memset(tracker, 0, sizeof(*tracker));
    pthread_mutex_init(&tracker->lock, NULL);
    pthread_mutex_init(&tracker->state_lock, NULL);
    pthread_cond_init(&tracker->state_cond, NULL);
    vision_stream_init(&tracker->stream);
}

int vision_tracker_start(VisionTracker *tracker, const char *camera_device, int debug_enabled, int stream_enabled, int stream_port)
{
    int result;

    if (tracker == NULL) {
        return -EINVAL;
    }

    tracker->camera_device = camera_device != NULL ? camera_device : JIWY_VISION_DEFAULT_CAMERA;
    tracker->debug_enabled = debug_enabled;
    tracker->start_done = 0;
    tracker->start_result = 0;

    result = vision_stream_start(&tracker->stream, stream_enabled, stream_port);
    if (result < 0) {
        return result;
    }

    result = pthread_create(&tracker->thread, NULL, vision_thread_main, tracker);
    if (result != 0) {
        vision_stream_stop(&tracker->stream);
        return -result;
    }
    tracker->thread_started = 1;

    pthread_mutex_lock(&tracker->state_lock);
    while (!tracker->start_done) {
        pthread_cond_wait(&tracker->state_cond, &tracker->state_lock);
    }
    result = tracker->start_result;
    pthread_mutex_unlock(&tracker->state_lock);

    if (result < 0) {
        pthread_join(tracker->thread, NULL);
        tracker->thread_started = 0;
        vision_stream_stop(&tracker->stream);
    }

    return result;
}

void vision_tracker_stop(VisionTracker *tracker)
{
    if (tracker == NULL || !tracker->thread_started) {
        return;
    }

    if (tracker->loop != NULL) {
        g_main_loop_quit(tracker->loop);
    }

    pthread_join(tracker->thread, NULL);
    tracker->thread_started = 0;

    vision_stream_stop(&tracker->stream);
}

int vision_tracker_read_latest(VisionTracker *tracker, VisionTargetSnapshot *snapshot)
{
    if (tracker == NULL || snapshot == NULL) {
        return -EINVAL;
    }

    pthread_mutex_lock(&tracker->lock);
    *snapshot = tracker->latest;
    pthread_mutex_unlock(&tracker->lock);

    return 0;
}