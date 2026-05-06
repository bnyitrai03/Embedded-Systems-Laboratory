#include <gst/gst.h>
#include <glib.h>



static gboolean bus_call(GstBus *bus, GstMessage *msg, gpointer data)
{
  GMainLoop *loop = (GMainLoop *) data;


  switch (GST_MESSAGE_TYPE (msg)) {


    case GST_MESSAGE_EOS:
      g_print ("End of stream\n");
      g_main_loop_quit (loop);
      break;


    case GST_MESSAGE_ERROR: {
      gchar  *debug;
      GError *error;


      gst_message_parse_error (msg, &error, &debug);
      g_free (debug);


      g_printerr ("Error: %s\n", error->message);
      g_error_free (error);


      g_main_loop_quit (loop);
      break;
    }
    default:
      break;
  }


  return TRUE;
}



int main ()
{
  GMainLoop *loop;


  GstElement *pipeline, *source, *capsfilter, *decoder, *sink;
  GstCaps *caps;
  GstBus *bus;
  guint bus_watch_id;


  /* Initialisation */
  gst_init ();
  loop = g_main_loop_new (NULL, FALSE);



// gst-launch-1.0 -v -e v4l2src device=/dev/video0 ! image/jpeg,width=320,height=240,framerate=30/1 ! jpegdec ! filesink location=file.yuv
/* Create gstreamer elements */
pipeline   = gst_pipeline_new ("camera-capture");


source     = gst_element_factory_make ("v4l2src",    "camera-source");
capsfilter = gst_element_factory_make ("capsfilter", "camera-caps");
decoder    = gst_element_factory_make ("jpegdec",    "jpeg-decoder");
sink       = gst_element_factory_make ("filesink",   "file-output");


if (!pipeline || !source || !capsfilter || !decoder || !sink) {
  g_printerr ("One element could not be created. Exiting.\n");
  return -1;
}


/* Set camera device */
g_object_set (G_OBJECT (source),
              "device", "/dev/video0",
              "num-buffers", 100,
              NULL);


/* Set requested MJPEG camera format */
caps = gst_caps_new_simple ("image/jpeg",
                            "width", G_TYPE_INT, 320,
                            "height", G_TYPE_INT, 240,
                            "framerate", GST_TYPE_FRACTION, 30, 1,
                            NULL);


g_object_set (G_OBJECT (capsfilter),
              "caps", caps,
              NULL);


gst_caps_unref (caps);


/* Set output file */
g_object_set (G_OBJECT (sink),
              "location", "file.yuv",
              NULL);


/* Add elements to pipeline */
gst_bin_add_many (GST_BIN (pipeline),
                  source, capsfilter, decoder, sink,
                  NULL);


/* Link static pipeline */
if (!gst_element_link_many (source, capsfilter, decoder, sink, NULL)) {
  g_printerr ("Elements could not be linked.\n");
  gst_object_unref (pipeline);
  return -1;
}


/* Add bus watch */
/* Add bus watch before starting */
bus = gst_pipeline_get_bus (GST_PIPELINE (pipeline));
bus_watch_id = gst_bus_add_watch (bus, bus_call, loop);
gst_object_unref (bus);


/* Start pipeline */
g_print ("Starting camera capture\n");


GstStateChangeReturn ret;
ret = gst_element_set_state (pipeline, GST_STATE_PLAYING);


if (ret == GST_STATE_CHANGE_FAILURE) {
  g_printerr ("Failed to set pipeline to PLAYING.\n");
  gst_object_unref (pipeline);
  return -1;
}


/* Keep program alive */
g_print ("Running main loop...\n");
g_main_loop_run (loop);


/* Cleanup only after EOS/error/Ctrl+C handling */
g_print ("Returned from main loop, stopping pipeline\n");
gst_element_set_state (pipeline, GST_STATE_NULL);


g_print ("Deleting pipeline\n");
gst_object_unref (GST_OBJECT (pipeline));
g_source_remove (bus_watch_id);
g_main_loop_unref (loop);
return 0;
} 