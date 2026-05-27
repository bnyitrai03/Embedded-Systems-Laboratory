#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/video/video.h>
#include <glib-unix.h>

#include <signal.h>

#define FRAME_WIDTH 640
#define FRAME_HEIGHT 480
#define FRAME_RATE 30
#define MIN_GREEN_PIXELS 80
#define NUM_BUFFERS 0

typedef struct {
    GMainLoop *loop;
    guint64 frame_count;
} AppData;

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

static GstFlowReturn on_new_sample(GstAppSink *appsink, gpointer user_data)
{
    AppData *data = (AppData *)user_data;
    GstSample *sample = gst_app_sink_pull_sample(appsink);
    GstBuffer *buffer;
    GstCaps *caps;
    GstVideoInfo info;
    GstMapInfo map;
    int width;
    int height;
    int stride;
    guint64 green_pixels = 0;
    guint64 sum_x = 0;
    guint64 sum_y = 0;
    int min_x;
    int min_y;
    int max_x = -1;
    int max_y = -1;

    if (!sample) {
        return GST_FLOW_ERROR;
    }

    buffer = gst_sample_get_buffer(sample);
    caps = gst_sample_get_caps(sample);

    if (!buffer || !caps || !gst_video_info_from_caps(&info, caps)) {
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }

    if (GST_VIDEO_INFO_FORMAT(&info) != GST_VIDEO_FORMAT_RGB) {
        g_printerr("Expected RGB frames, got %s\n", GST_VIDEO_INFO_NAME(&info));
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }

    width = GST_VIDEO_INFO_WIDTH(&info);
    height = GST_VIDEO_INFO_HEIGHT(&info);
    stride = GST_VIDEO_INFO_PLANE_STRIDE(&info, 0);
    min_x = width;
    min_y = height;

    if (stride <= 0 || !gst_buffer_map(buffer, &map, GST_MAP_READ)) {
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }

    if (map.size < (gsize)((height - 1) * stride + width * 3)) {
        g_printerr("Frame buffer is smaller than expected for %dx%d RGB\n", width, height);
        gst_buffer_unmap(buffer, &map);
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }

    for (int y = 0; y < height; y++) {
        const guint8 *row = map.data + y * stride;

        for (int x = 0; x < width; x++) {
            const guint8 *pixel = row + x * 3;

            if (is_green_pixel(pixel[0], pixel[1], pixel[2])) {
                green_pixels++;
                sum_x += (guint64)x;
                sum_y += (guint64)y;

                if (x < min_x) {
                    min_x = x;
                }
                if (x > max_x) {
                    max_x = x;
                }
                if (y < min_y) {
                    min_y = y;
                }
                if (y > max_y) {
                    max_y = y;
                }
            }
        }
    }

    if (green_pixels >= MIN_GREEN_PIXELS) {
        double object_x = (double)sum_x / (double)green_pixels;
        double object_y = (double)sum_y / (double)green_pixels;
        int bbox_w = max_x - min_x + 1;
        int bbox_h = max_y - min_y + 1;

        g_print(
            "Frame %5" G_GUINT64_FORMAT
            " | green object x=%6.1f y=%6.1f"
            " | bbox x=%d y=%d w=%d h=%d"
            " | pixels=%" G_GUINT64_FORMAT "\n",
            data->frame_count,
            object_x,
            object_y,
            min_x,
            min_y,
            bbox_w,
            bbox_h,
            green_pixels
        );
    } else {
        g_print(
            "Frame %5" G_GUINT64_FORMAT
            " | no green object | green pixels=%" G_GUINT64_FORMAT "\n",
            data->frame_count,
            green_pixels
        );
    }

    data->frame_count++;

    gst_buffer_unmap(buffer, &map);
    gst_sample_unref(sample);

    return GST_FLOW_OK;
}

static gboolean on_bus_message(GstBus *bus, GstMessage *message, gpointer user_data)
{
    AppData *data = (AppData *)user_data;

    (void)bus;

    switch (GST_MESSAGE_TYPE(message)) {
    case GST_MESSAGE_ERROR: {
        GError *error = NULL;
        gchar *debug = NULL;

        gst_message_parse_error(message, &error, &debug);
        g_printerr("GStreamer error: %s\n", error ? error->message : "unknown");
        g_printerr("Debug info: %s\n", debug ? debug : "none");
        g_clear_error(&error);
        g_free(debug);
        g_main_loop_quit(data->loop);
        break;
    }
    case GST_MESSAGE_EOS:
        g_main_loop_quit(data->loop);
        break;
    default:
        break;
    }

    return TRUE;
}

static gboolean on_sigint(gpointer user_data)
{
    AppData *data = (AppData *)user_data;

    g_print("\nStopping...\n");
    g_main_loop_quit(data->loop);
    return G_SOURCE_CONTINUE;
}

int main(int argc, char *argv[])
{
    const char *camera_device = "/dev/video0";
    AppData data;
    GstElement *pipeline;
    GstElement *source;
    GstElement *jpegdec;
    GstElement *convert;
    GstElement *scale;
    GstElement *capsfilter;
    GstElement *sink;
    GstCaps *caps;
    GstBus *bus;
    guint bus_watch_id;
    guint sigint_watch_id;
    GstStateChangeReturn state_result;
    int exit_code = 0;

    if (argc > 2) {
        g_printerr("Usage: %s [camera-device]\n", argv[0]);
        return 1;
    }
    if (argc == 2) {
        camera_device = argv[1];
    }

    gst_init(&argc, &argv);

    data.loop = g_main_loop_new(NULL, FALSE);
    data.frame_count = 0;

    pipeline = gst_pipeline_new("green-ball-rpi-headless");
    source = gst_element_factory_make("v4l2src", "source");
    jpegdec = gst_element_factory_make("jpegdec", "jpeg-decoder");
    convert = gst_element_factory_make("videoconvert", "rgb-convert");
    scale = gst_element_factory_make("videoscale", "video-scale");
    capsfilter = gst_element_factory_make("capsfilter", "rgb-caps");
    sink = gst_element_factory_make("appsink", "sink");

    if (!pipeline || !source || !jpegdec || !convert || !scale || !capsfilter || !sink) {
        g_printerr("Could not create all GStreamer elements.\n");
        exit_code = 1;
        goto cleanup_before_add;
    }

    g_object_set(source, "device", camera_device, NULL);
    if (NUM_BUFFERS > 0) {
        g_object_set(source, "num-buffers", NUM_BUFFERS, NULL);
    }

    caps = gst_caps_new_simple(
        "video/x-raw",
        "format", G_TYPE_STRING, "RGB",
        "width", G_TYPE_INT, FRAME_WIDTH,
        "height", G_TYPE_INT, FRAME_HEIGHT,
        "framerate", GST_TYPE_FRACTION, FRAME_RATE, 1,
        NULL
    );
    g_object_set(capsfilter, "caps", caps, NULL);
    gst_caps_unref(caps);

    g_object_set(
        sink,
        "emit-signals", TRUE,
        "sync", FALSE,
        "max-buffers", 1,
        "drop", TRUE,
        NULL
    );
    g_signal_connect(sink, "new-sample", G_CALLBACK(on_new_sample), &data);

    gst_bin_add_many(
        GST_BIN(pipeline),
        source,
        jpegdec,
        convert,
        scale,
        capsfilter,
        sink,
        NULL
    );

    if (!gst_element_link_many(source, jpegdec, convert, scale, capsfilter, sink, NULL)) {
        g_printerr("Could not link camera pipeline.\n");
        g_printerr("This version expects an MJPEG v4l2 camera.\n");
        exit_code = 1;
        goto cleanup_pipeline;
    }

    bus = gst_pipeline_get_bus(GST_PIPELINE(pipeline));
    bus_watch_id = gst_bus_add_watch(bus, on_bus_message, &data);
    gst_object_unref(bus);

    sigint_watch_id = g_unix_signal_add(SIGINT, on_sigint, &data);

    g_print("rpi headless | camera device: %s\n", camera_device);
    g_print("Detecting green object in %dx%d RGB frames\n", FRAME_WIDTH, FRAME_HEIGHT);

    state_result = gst_element_set_state(pipeline, GST_STATE_PLAYING);
    if (state_result == GST_STATE_CHANGE_FAILURE) {
        g_printerr("Failed to start pipeline.\n");
        exit_code = 1;
        g_source_remove(sigint_watch_id);
        g_source_remove(bus_watch_id);
        goto cleanup_pipeline;
    }

    g_main_loop_run(data.loop);

    g_source_remove(sigint_watch_id);
    g_source_remove(bus_watch_id);

cleanup_pipeline:
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);

cleanup_before_add:
    g_main_loop_unref(data.loop);

    return exit_code;
}
