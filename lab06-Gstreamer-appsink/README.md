````markdown
# Compile and Run

This program uses GStreamer with `appsink`.

## 1. Check required elements

```bash
gst-inspect-1.0 v4l2src
gst-inspect-1.0 jpegdec
gst-inspect-1.0 appsink
````

## 2. Compile

```bash
gcc appsink_pipeline.c -o appsink_pipeline \
  $(pkg-config --cflags --libs gstreamer-1.0 gstreamer-app-1.0)
```

## 3. Run

```bash
./appsink_pipeline
```

## 4. Camera device

The code currently uses:

```c
/dev/video0
```

If needed, check available cameras:

```bash
ls /dev/video*
```

Then update this line in the code:

```c
g_object_set(source, "device", "/dev/video0", NULL);
```

## 5. Expected output

```text
Running: v4l2src ! jpegdec ! appsink
Frame 0 | I420 | 160x120 | avg=95 | dark
Frame 1 | I420 | 160x120 | avg=130 | bright
Finished.
```

```
```
