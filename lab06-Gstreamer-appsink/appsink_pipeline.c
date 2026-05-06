#include <gst/gst.h>
#include <gst/app/gstappsink.h>

/*
 * The program will stop automatically after this many frames.
 * This is useful for testing, because otherwise the camera source would run forever.
 */
#define NUM_BUFFERS 300

/*
 * Brightness threshold.
 * Average Y value is between 0 and 255.
 * Lower values mean darker image, higher values mean brighter image.
 */
#define BRIGHTNESS_THRESHOLD 100

/*
 * Calculates the average brightness of an I420 frame.
 *
 * I420 is a YUV format.
 * In I420, the first width * height bytes are the Y plane.
 * The Y plane represents luminance, which means brightness.
 *
 * Therefore, for a simple brightness check, we only need to average
 * the first width * height bytes.
 */
static int average_brightness_i420(const guint8 *data, gsize size, int width, int height)
{
    int y_size = width * height;

    /*
     * Basic safety check.
     * If width/height are invalid, or the buffer is smaller than the expected
     * Y plane size, we should not read from it.
     */
    if (width <= 0 || height <= 0 || size < (gsize)y_size) {
        return -1;
    }

    unsigned long long total = 0;

    /*
     * Sum all Y values.
     * Each Y value is one byte and represents the brightness of one pixel.
     */
    for (int i = 0; i < y_size; i++) {
        total += data[i];
    }

    /*
     * Return the average brightness.
     */
    return (int)(total / y_size);
}

/*
 * This callback is called every time appsink receives a new decoded frame.
 *
 * The pipeline is:
 *
 *   v4l2src -> jpegdec -> appsink
 *
 * v4l2src receives MJPEG frames from the camera.
 * jpegdec decodes those JPEG frames into raw video frames.
 * appsink passes those raw frames to this C function.
 */
static GstFlowReturn on_new_sample(GstAppSink *appsink, gpointer user_data)
{
    /*
     * Static means the value is kept between function calls.
     * This lets us count frames without using a global variable.
     */
    static int frame_count = 0;

    /*
     * Pull the newest sample from appsink.
     * A GstSample usually contains:
     *   - GstBuffer: the actual frame data
     *   - GstCaps: metadata such as format, width, height, framerate
     */
    GstSample *sample = gst_app_sink_pull_sample(appsink);

    if (!sample) {
        return GST_FLOW_ERROR;
    }

    /*
     * Get the actual frame buffer and the caps from the sample.
     */
    GstBuffer *buffer = gst_sample_get_buffer(sample);
    GstCaps *caps = gst_sample_get_caps(sample);

    if (!buffer || !caps) {
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }

    /*
     * Caps contain information about the decoded frame.
     * For example:
     *   video/x-raw, format=I420, width=160, height=120
     */
    GstStructure *structure = gst_caps_get_structure(caps, 0);

    /*
     * Read the pixel format from the caps.
     * We expect I420, because this code calculates brightness from I420 Y plane.
     */
    const gchar *format = gst_structure_get_string(structure, "format");

    int width = 0;
    int height = 0;

    /*
     * Read frame dimensions from the caps.
     * This is better than hardcoding width and height inside the callback.
     */
    gst_structure_get_int(structure, "width", &width);
    gst_structure_get_int(structure, "height", &height);

    GstMapInfo map;

    /*
     * Map the GstBuffer so we can access the raw frame bytes.
     * map.data points to the frame data.
     * map.size is the number of bytes in the buffer.
     */
    if (gst_buffer_map(buffer, &map, GST_MAP_READ)) {

        /*
         * This brightness function only supports I420.
         * If jpegdec outputs another raw format, we print a message instead
         * of reading the buffer incorrectly.
         */
        if (g_strcmp0(format, "I420") == 0) {
            int avg = average_brightness_i420(map.data, map.size, width, height);

            g_print(
                "Frame %d | %s | %dx%d | avg=%d | %s\n",
                frame_count,
                format,
                width,
                height,
                avg,
                avg > BRIGHTNESS_THRESHOLD ? "bright" : "dark"
            );
        } else {
            g_print(
                "Frame %d | decoded format is %s, expected I420\n",
                frame_count,
                format ? format : "unknown"
            );
        }

        /*
         * Always unmap the buffer after accessing it.
         */
        gst_buffer_unmap(buffer, &map);
    }

    frame_count++;

    /*
     * Release the sample after we are done with it.
     * The sample owns references to the buffer and caps.
     */
    gst_sample_unref(sample);

    /*
     * Tell GStreamer that processing this sample was successful.
     */
    return GST_FLOW_OK;
}

int main(int argc, char *argv[])
{
    GstElement *pipeline;
    GstElement *source;
    GstElement *decoder;
    GstElement *sink;

    GstBus *bus;
    GstMessage *msg;

    /*
     * Initialize GStreamer.
     * This must be called before creating GStreamer elements.
     */
    gst_init(&argc, &argv);

    /*
     * Create an empty pipeline.
     * A pipeline is the container that holds and runs all elements.
     */
    pipeline = gst_pipeline_new("lab-appsink-pipeline");

    /*
     * Create the elements.
     *
     * v4l2src:
     *   Reads frames from a Linux video device, for example /dev/video7.
     *
     * jpegdec:
     *   Decodes MJPEG/JPEG frames into raw video frames.
     *
     * appsink:
     *   Gives the decoded raw frames to our C code.
     */
    source  = gst_element_factory_make("v4l2src", "source");
    decoder = gst_element_factory_make("jpegdec", "decoder");
    sink    = gst_element_factory_make("appsink", "sink");

    /*
     * Check that all elements were created successfully.
     * If one of them is NULL, the plugin may be missing or the element name is wrong.
     */
    if (!pipeline || !source || !decoder || !sink) {
        g_printerr("Could not create all elements.\n");
        return -1;
    }

    /*
     * Select the camera device.
     * In the lab tutorial this is /dev/video0.
     */
    g_object_set(source, "device", "/dev/video0", NULL);

    /*
     * Stop after NUM_BUFFERS frames.
     * Without this, the camera would keep streaming until the program is interrupted.
     */
    g_object_set(source, "num-buffers", NUM_BUFFERS, NULL);

    /*
     * Configure appsink.
     *
     * emit-signals = TRUE:
     *   appsink emits the "new-sample" signal when a new frame arrives.
     *
     * sync = FALSE:
     *   Do not synchronize to the clock. For processing frames in code, this is simpler.
     *
     * max-buffers = 1:
     *   Keep at most one frame queued inside appsink.
     *
     * drop = TRUE:
     *   If our code is too slow, drop old frames instead of building a large queue.
     */
    g_object_set(
        sink,
        "emit-signals", TRUE,
        "sync", FALSE,
        "max-buffers", 1,
        "drop", TRUE,
        NULL
    );

    /*
     * Connect the appsink "new-sample" signal to our callback function.
     * Every time a decoded frame reaches appsink, on_new_sample() is called.
     */
    g_signal_connect(sink, "new-sample", G_CALLBACK(on_new_sample), NULL);

    /*
     * Add all elements to the pipeline.
     * Elements must be inside the pipeline before they can be linked and run.
     */
    gst_bin_add_many(GST_BIN(pipeline), source, decoder, sink, NULL);

    /*
     * Link the elements in order:
     *
     *   v4l2src -> jpegdec -> appsink
     *
     * This assumes that the camera source produces MJPEG/JPEG frames.
     */
    if (!gst_element_link_many(source, decoder, sink, NULL)) {
        g_printerr("Could not link elements.\n");
        gst_object_unref(pipeline);
        return -1;
    }

    g_print("Running: v4l2src ! jpegdec ! appsink\n");

    /*
     * Start the pipeline.
     * After this, the camera starts producing frames.
     */
    gst_element_set_state(pipeline, GST_STATE_PLAYING);

    /*
     * Get the bus from the pipeline.
     * The bus is used to receive messages such as errors and EOS.
     */
    bus = gst_element_get_bus(pipeline);

    /*
     * Wait until either:
     *   - an error occurs
     *   - EOS is received
     *
     * EOS should happen automatically after NUM_BUFFERS frames.
     */
    msg = gst_bus_timed_pop_filtered(
        bus,
        GST_CLOCK_TIME_NONE,
        GST_MESSAGE_ERROR | GST_MESSAGE_EOS
    );

    /*
     * Handle the received bus message.
     */
    if (msg) {
        if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_ERROR) {
            GError *err;
            gchar *debug;

            gst_message_parse_error(msg, &err, &debug);

            g_printerr("Error: %s\n", err->message);
            g_printerr("Debug: %s\n", debug ? debug : "none");

            g_clear_error(&err);
            g_free(debug);
        } else if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_EOS) {
            g_print("Finished.\n");
        }

        gst_message_unref(msg);
    }

    /*
     * Clean up:
     *   1. Release the bus reference.
     *   2. Stop the pipeline.
     *   3. Release the pipeline.
     *
     * Releasing the pipeline also releases the elements inside it.
     */
    gst_object_unref(bus);
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);

    return 0;
}