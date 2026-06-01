#include "vision_tracker.h"

#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include <gst/app/gstappsink.h>
#include <gst/video/video.h>

#define VISION_FRAME_WIDTH 640
#define VISION_FRAME_HEIGHT 480
#define VISION_FRAME_RATE 30
#define VISION_MIN_GREEN_PIXELS 80
#define VISION_FOV_RAD (60.0 * M_PI / 180.0)
#define VISION_DEFAULT_CAMERA "/dev/video0"

typedef struct {
    VisionTracker *tracker;
    uint64_t frame_count;
} VisionPipelineData;

static gboolean is_green_pixel(guint8 r, guint8 g, guint8 b)
{
    guint8 max_value = MAX(r, MAX(g, b));
    guint8 min_value = MIN(r, MIN(g, b));
    int delta = max_value - min_value;
    int hue;
    int saturation_percent;

    if (max_value < 50 || delta < 30 || g != max_value) {
        return FALSE;
    }

    hue = 120 + (60 * ((int)b - (int)r)) / delta;
    saturation_percent = (delta * 100) / max_value;

    return hue >= 60 && hue <= 170 && saturation_percent >= 35;
}

static void pixel_to_camera_error(double object_x,
                                  double object_y,
                                  int width,
                                  int height,
                                  double *yaw_error_rad,
                                  double *pitch_error_rad)
{
    double center_x = (double)width / 2.0;
    double center_y = (double)height / 2.0;

    *yaw_error_rad =
        ((object_x - center_x) / center_x) * (VISION_FOV_RAD / 2.0);
    *pitch_error_rad =
        ((center_y - object_y) / center_y) * (VISION_FOV_RAD / 2.0);
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
    uint64_t green_pixels = 0;
    uint64_t sum_x = 0;
    uint64_t sum_y = 0;

    if (sample == NULL) {
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

    for (int y = 0; y < height; ++y) {
        const guint8 *row = map.data + y * stride;

        for (int x = 0; x < width; ++x) {
            const guint8 *pixel = row + x * 3;

            if (is_green_pixel(pixel[0], pixel[1], pixel[2])) {
                ++green_pixels;
                sum_x += (uint64_t)x;
                sum_y += (uint64_t)y;
            }
        }
    }

    memset(&snapshot, 0, sizeof(snapshot));
    snapshot.frame_count = data->frame_count;

    if (green_pixels >= VISION_MIN_GREEN_PIXELS) {
        double object_x = (double)sum_x / (double)green_pixels;
        double object_y = (double)sum_y / (double)green_pixels;

        snapshot.valid = 1;
        pixel_to_camera_error(object_x,
                              object_y,
                              width,
                              height,
                              &snapshot.yaw_error_rad,
                              &snapshot.pitch_error_rad);
    }

    update_snapshot(tracker, &snapshot);

    if (tracker->debug_enabled && data->frame_count % 30 == 0) {
        if (snapshot.valid) {
            printf("vision frame=%" G_GUINT64_FORMAT
                   " yaw_err=%.4f pitch_err=%.4f pixels=%" G_GUINT64_FORMAT "\n",
                   (guint64)data->frame_count,
                   snapshot.yaw_error_rad,
                   snapshot.pitch_error_rad,
                   (guint64)green_pixels);
        } else {
            printf("vision frame=%" G_GUINT64_FORMAT
                   " no green object pixels=%" G_GUINT64_FORMAT "\n",
                   (guint64)data->frame_count,
                   (guint64)green_pixels);
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
    GstElement *jpegdec;
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

    pipeline = gst_pipeline_new("jiwy-vision-tracker");
    source = gst_element_factory_make("v4l2src", "source");
    source_capsfilter = gst_element_factory_make("capsfilter", "source-caps");
    jpegdec = gst_element_factory_make("jpegdec", "jpeg-decoder");
    convert = gst_element_factory_make("videoconvert", "rgb-convert");
    scale = gst_element_factory_make("videoscale", "video-scale");
    rgb_capsfilter = gst_element_factory_make("capsfilter", "rgb-caps");
    sink = gst_element_factory_make("appsink", "sink");

    if (pipeline == NULL || source == NULL || source_capsfilter == NULL ||
        jpegdec == NULL || convert == NULL || scale == NULL ||
        rgb_capsfilter == NULL || sink == NULL) {
        fprintf(stderr, "Vision tracker could not create GStreamer elements\n");
        if (pipeline != NULL) {
            gst_object_unref(pipeline);
        }
        publish_start_result(tracker, -ENODEV);
        return NULL;
    }

    g_object_set(source, "device", tracker->camera_device, NULL);

    source_caps = gst_caps_new_simple("image/jpeg",
                                      "width", G_TYPE_INT, VISION_FRAME_WIDTH,
                                      "height", G_TYPE_INT, VISION_FRAME_HEIGHT,
                                      "framerate", GST_TYPE_FRACTION,
                                      VISION_FRAME_RATE, 1,
                                      NULL);
    g_object_set(source_capsfilter, "caps", source_caps, NULL);
    gst_caps_unref(source_caps);

    rgb_caps = gst_caps_new_simple("video/x-raw",
                                   "format", G_TYPE_STRING, "RGB",
                                   "width", G_TYPE_INT, VISION_FRAME_WIDTH,
                                   "height", G_TYPE_INT, VISION_FRAME_HEIGHT,
                                   "framerate", GST_TYPE_FRACTION,
                                   VISION_FRAME_RATE, 1,
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
                     jpegdec,
                     convert,
                     scale,
                     rgb_capsfilter,
                     sink,
                     NULL);

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

    return NULL;
}

void vision_tracker_init(VisionTracker *tracker)
{
    memset(tracker, 0, sizeof(*tracker));
    pthread_mutex_init(&tracker->lock, NULL);
    pthread_mutex_init(&tracker->state_lock, NULL);
    pthread_cond_init(&tracker->state_cond, NULL);
}

int vision_tracker_start(VisionTracker *tracker,
                         const char *camera_device,
                         int debug_enabled)
{
    int result;

    if (tracker == NULL) {
        return -EINVAL;
    }

    tracker->camera_device =
        camera_device != NULL ? camera_device : VISION_DEFAULT_CAMERA;
    tracker->debug_enabled = debug_enabled;
    tracker->start_done = 0;
    tracker->start_result = 0;

    result = pthread_create(&tracker->thread,
                            NULL,
                            vision_thread_main,
                            tracker);
    if (result != 0) {
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
    } else {
        tracker->running = 1;
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
    tracker->running = 0;
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
