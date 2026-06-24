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

static void update_snapshot(VisionTracker *tracker,
                            const VisionTargetSnapshot *snapshot)
{
    pthread_mutex_lock(&tracker->lock);
    tracker->latest = *snapshot;
    pthread_mutex_unlock(&tracker->lock);
}

static GstFlowReturn on_new_sample(GstAppSink *appsink, gpointer user_data)
{
    VisionPipelineData *data = (VisionPipelineData *)user_data;
    VisionTracker *tracker = data->tracker;
    GstSample *sample = gst_app_sink_pull_sample(appsink);
    GstBuffer *buffer;
    GstCaps *caps;
    GstVideoInfo info;
    GstMapInfo map;
    VisionTargetSnapshot snapshot;
    int width;
    int height;
    int stride;
    VisionBlobDetection detection;
    struct timespec frame_start;
    struct timespec frame_end;
    long process_us = 0;
    double frame_interval_ms = 0.0;
    const double nominal_frame_ms = 1000.0 / (double)JIWY_VISION_FRAME_RATE;

    if (sample == NULL) {
        return GST_FLOW_ERROR;
    }

    if (clock_gettime(CLOCK_MONOTONIC, &frame_start) < 0) {
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }

    buffer = gst_sample_get_buffer(sample);
    caps = gst_sample_get_caps(sample);

    if (buffer == NULL || caps == NULL ||
        !gst_video_info_from_caps(&info, caps)) {
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }

    if (GST_VIDEO_INFO_FORMAT(&info) != GST_VIDEO_FORMAT_RGB) {
        fprintf(stderr,
                "Vision tracker expected RGB frames, got %s\n",
                GST_VIDEO_INFO_NAME(&info));
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }

    width = GST_VIDEO_INFO_WIDTH(&info);
    height = GST_VIDEO_INFO_HEIGHT(&info);
    stride = GST_VIDEO_INFO_PLANE_STRIDE(&info, 0);

    if (width <= 0 || height <= 0 ||
        (size_t)height > (size_t)INT_MAX / (size_t)width) {
        fprintf(stderr, "Vision tracker got invalid frame dimensions\n");
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }

    if (stride <= 0 || !gst_buffer_map(buffer, &map, GST_MAP_READ)) {
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }

    if (map.size < (gsize)((height - 1) * stride + width * 3)) {
        fprintf(stderr,
                "Vision tracker frame buffer is smaller than expected\n");
        gst_buffer_unmap(buffer, &map);
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }

    if (vision_blob_tracker_process_rgb(&data->blob_tracker,
                                        map.data,
                                        width,
                                        height,
                                        stride,
                                        &detection) < 0) {
        gst_buffer_unmap(buffer, &map);
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }

    memset(&snapshot, 0, sizeof(snapshot));
    snapshot.frame_count = data->frame_count;
    if (data->have_last_frame_time) {
        frame_interval_ms =
            (double)timespec_diff_us(frame_start, data->last_frame_time) / 1000.0;
    }
    snapshot.frame_interval_ms = frame_interval_ms;

    if (detection.valid) {
        snapshot.valid = 1;
        vision_pixel_to_camera_error(detection.object_x,
                                     detection.object_y,
                                     width,
                                     height,
                                     &snapshot.yaw_error_rad,
                                     &snapshot.pitch_error_rad);
    }

    if (clock_gettime(CLOCK_MONOTONIC, &frame_end) < 0) {
        gst_buffer_unmap(buffer, &map);
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }
    process_us = timespec_diff_us(frame_end, frame_start);
    snapshot.process_us = (double)process_us;
    snapshot.late_frame =
        data->have_last_frame_time &&
        frame_interval_ms > nominal_frame_ms * 1.5;
    data->last_frame_time = frame_start;
    data->have_last_frame_time = 1;

    update_snapshot(tracker, &snapshot);

    if (tracker->stream.enabled &&
        data->frame_count % JIWY_VISION_STREAM_FPS_DIVISOR == 0) {
        vision_stream_publish_rgb(&tracker->stream,
                                  map.data,
                                  width,
                                  height,
                                  stride,
                                  &detection,
                                  &snapshot);
    }

    if (tracker->debug_enabled &&
        data->frame_count % JIWY_VISION_DEBUG_EVERY_FRAMES == 0) {
        if (snapshot.valid) {
            printf("vision frame=%" G_GUINT64_FORMAT
                   " yaw_err=%.4f pitch_err=%.4f blob=%" G_GUINT64_FORMAT
                   " green=%" G_GUINT64_FORMAT
                   " box=%dx%d"
                   " dt=%.2fms proc=%.0fus late=%d\n",
                   (guint64)data->frame_count,
                   snapshot.yaw_error_rad,
                   snapshot.pitch_error_rad,
                   (guint64)detection.blob_pixels,
                   (guint64)detection.total_green_pixels,
                   detection.max_x - detection.min_x + 1,
                   detection.max_y - detection.min_y + 1,
                   snapshot.frame_interval_ms,
                   snapshot.process_us,
                   snapshot.late_frame);
        } else {
            printf("vision frame=%" G_GUINT64_FORMAT
                   " no green blob green=%" G_GUINT64_FORMAT
                   " dt=%.2fms proc=%.0fus late=%d\n",
                   (guint64)data->frame_count,
                   (guint64)detection.total_green_pixels,
                   snapshot.frame_interval_ms,
                   snapshot.process_us,
                   snapshot.late_frame);
        }
    }

    ++data->frame_count;
    gst_buffer_unmap(buffer, &map);
    gst_sample_unref(sample);

    return GST_FLOW_OK;
}

static gboolean on_bus_message(GstBus *bus,
                               GstMessage *message,
                               gpointer user_data)
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

static void publish_start_result(VisionTracker *tracker, int result)
{
    pthread_mutex_lock(&tracker->state_lock);
    tracker->start_result = result;
    tracker->start_done = 1;
    pthread_cond_signal(&tracker->state_cond);
    pthread_mutex_unlock(&tracker->state_lock);
}

static void *vision_thread_main(void *arg)
{
    VisionTracker *tracker = (VisionTracker *)arg;
    VisionPipelineData data;
    GstElement *pipeline;
    GstElement *source;
    GstElement *source_capsfilter;
#ifndef JIWY_VISION_USE_AVFVIDEOSRC
    GstElement *jpegdec;
#endif
    GstElement *convert;
    GstElement *scale;
    GstElement *rgb_capsfilter;
    GstElement *sink;
    GstCaps *source_caps;
    GstCaps *rgb_caps;
    GstBus *bus;
    guint bus_watch_id;
    GstStateChangeReturn state_result;
    GstStateChangeReturn ready_result;

    gst_init(NULL, NULL);

    memset(&data, 0, sizeof(data));
    data.tracker = tracker;
    vision_blob_tracker_init(&data.blob_tracker);

    pipeline = gst_pipeline_new("jiwy-vision-tracker");
#ifdef JIWY_VISION_USE_AVFVIDEOSRC
    source = gst_element_factory_make("avfvideosrc", "source");
#else
    source = gst_element_factory_make("v4l2src", "source");
#endif
    source_capsfilter = gst_element_factory_make("capsfilter", "source-caps");
#ifndef JIWY_VISION_USE_AVFVIDEOSRC
    jpegdec = gst_element_factory_make("jpegdec", "jpeg-decoder");
#endif
    convert = gst_element_factory_make("videoconvert", "rgb-convert");
    scale = gst_element_factory_make("videoscale", "video-scale");
    rgb_capsfilter = gst_element_factory_make("capsfilter", "rgb-caps");
    sink = gst_element_factory_make("appsink", "sink");

    if (pipeline == NULL || source == NULL || source_capsfilter == NULL ||
#ifndef JIWY_VISION_USE_AVFVIDEOSRC
        jpegdec == NULL ||
#endif
        convert == NULL || scale == NULL || rgb_capsfilter == NULL ||
        sink == NULL) {
        fprintf(stderr, "Vision tracker could not create GStreamer elements\n");
        if (pipeline != NULL) {
            gst_object_unref(pipeline);
        }
        publish_start_result(tracker, -ENODEV);
        return NULL;
    }

#ifdef JIWY_VISION_USE_AVFVIDEOSRC
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
#else
    g_object_set(source, "device", tracker->camera_device, NULL);

    source_caps = gst_caps_new_simple("image/jpeg",
                                      "width", G_TYPE_INT, JIWY_VISION_FRAME_WIDTH,
                                      "height", G_TYPE_INT, JIWY_VISION_FRAME_HEIGHT,
                                      "framerate", GST_TYPE_FRACTION,
                                      JIWY_VISION_FRAME_RATE, 1,
                                      NULL);
#endif
    g_object_set(source_capsfilter, "caps", source_caps, NULL);
    gst_caps_unref(source_caps);

    rgb_caps = gst_caps_new_simple("video/x-raw",
                                   "format", G_TYPE_STRING, "RGB",
                                   "width", G_TYPE_INT, JIWY_VISION_FRAME_WIDTH,
                                   "height", G_TYPE_INT, JIWY_VISION_FRAME_HEIGHT,
                                   "framerate", GST_TYPE_FRACTION,
                                   JIWY_VISION_FRAME_RATE, 1,
                                   NULL);
    g_object_set(rgb_capsfilter, "caps", rgb_caps, NULL);
    gst_caps_unref(rgb_caps);

    g_object_set(sink,
                 "emit-signals", TRUE,
                 "sync", FALSE,
                 "max-buffers", 1,
                 "drop", TRUE,
                 NULL);
    g_signal_connect(sink, "new-sample", G_CALLBACK(on_new_sample), &data);

    gst_bin_add_many(GST_BIN(pipeline),
                     source,
                     source_capsfilter,
#ifndef JIWY_VISION_USE_AVFVIDEOSRC
                     jpegdec,
#endif
                     convert,
                     scale,
                     rgb_capsfilter,
                     sink,
                     NULL);

#ifdef JIWY_VISION_USE_AVFVIDEOSRC
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
#else
    if (!gst_element_link_many(source,
                               source_capsfilter,
                               jpegdec,
                               convert,
                               scale,
                               rgb_capsfilter,
                               sink,
                               NULL)) {
        fprintf(stderr,
                "Vision tracker could not link MJPEG camera pipeline\n");
        gst_object_unref(pipeline);
        publish_start_result(tracker, -EINVAL);
        return NULL;
    }
#endif

    tracker->loop = g_main_loop_new(NULL, FALSE);
    tracker->pipeline = pipeline;

    bus = gst_pipeline_get_bus(GST_PIPELINE(pipeline));
    bus_watch_id = gst_bus_add_watch(bus, on_bus_message, &data);
    gst_object_unref(bus);

    state_result = gst_element_set_state(pipeline, GST_STATE_PLAYING);
    if (state_result == GST_STATE_CHANGE_FAILURE) {
        fprintf(stderr, "Vision tracker failed to start camera pipeline\n");
        g_source_remove(bus_watch_id);
        g_main_loop_unref(tracker->loop);
        tracker->loop = NULL;
        tracker->pipeline = NULL;
        gst_object_unref(pipeline);
        publish_start_result(tracker, -EIO);
        return NULL;
    }

    ready_result = gst_element_get_state(pipeline,
                                         NULL,
                                         NULL,
                                         5 * GST_SECOND);
    if (ready_result == GST_STATE_CHANGE_FAILURE) {
        fprintf(stderr, "Vision tracker camera pipeline did not reach PLAYING\n");
        gst_element_set_state(pipeline, GST_STATE_NULL);
        g_source_remove(bus_watch_id);
        g_main_loop_unref(tracker->loop);
        tracker->loop = NULL;
        tracker->pipeline = NULL;
        gst_object_unref(pipeline);
        publish_start_result(tracker, -EIO);
        return NULL;
    }

    publish_start_result(tracker, 0);
    g_main_loop_run(tracker->loop);

    gst_element_set_state(pipeline, GST_STATE_NULL);
    g_source_remove(bus_watch_id);
    g_main_loop_unref(tracker->loop);
    tracker->loop = NULL;
    tracker->pipeline = NULL;
    gst_object_unref(pipeline);
    vision_blob_tracker_destroy(&data.blob_tracker);

    return NULL;
}

void vision_tracker_init(VisionTracker *tracker)
{
    memset(tracker, 0, sizeof(*tracker));
    pthread_mutex_init(&tracker->lock, NULL);
    pthread_mutex_init(&tracker->state_lock, NULL);
    pthread_cond_init(&tracker->state_cond, NULL);
    vision_stream_init(&tracker->stream);
}

int vision_tracker_start(VisionTracker *tracker,
                         const char *camera_device,
                         int debug_enabled,
                         int stream_enabled,
                         int stream_port)
{
    int result;

    if (tracker == NULL) {
        return -EINVAL;
    }

    tracker->camera_device =
        camera_device != NULL ? camera_device : JIWY_VISION_DEFAULT_CAMERA;
    tracker->debug_enabled = debug_enabled;
    tracker->start_done = 0;
    tracker->start_result = 0;

    result = vision_stream_start(&tracker->stream,
                                 stream_enabled,
                                 stream_port);
    if (result < 0) {
        return result;
    }

    result = pthread_create(&tracker->thread,
                            NULL,
                            vision_thread_main,
                            tracker);
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

int vision_tracker_read_latest(VisionTracker *tracker,
                               VisionTargetSnapshot *snapshot)
{
    if (tracker == NULL || snapshot == NULL) {
        return -EINVAL;
    }

    pthread_mutex_lock(&tracker->lock);
    *snapshot = tracker->latest;
    pthread_mutex_unlock(&tracker->lock);

    return 0;
}
