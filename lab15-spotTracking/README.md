# Lab 15: Green Ball Appsink Demo

This folder has four fixed versions of the same green object tracker:

```text
green_ball_mac_preview.c
green_ball_mac_headless.c
green_ball_rpi_preview.c
green_ball_rpi_headless.c
```

Each program creates its GStreamer elements directly in C, following the style of `lab06-Gstreamer-appsink/appsink_pipeline.c`. The `appsink` callback prints the detected object position in image coordinates. Coordinates use the top-left image corner as `(0, 0)`.

## Build

Pick exactly the version you want:

```bash
make mac-preview
make mac-headless
make rpi-preview
make rpi-headless
```

The binaries are:

```text
green_ball_mac_preview
green_ball_mac_headless
green_ball_rpi_preview
green_ball_rpi_headless
```

## Run

macOS:

```bash
./green_ball_mac_preview
./green_ball_mac_preview 1
./green_ball_mac_headless 0
```

Raspberry Pi/Linux:

```bash
./green_ball_rpi_preview /dev/video0
./green_ball_rpi_headless /dev/video0
```

## Pipelines

mac headless:

```text
avfvideosrc -> videoconvert -> videoscale -> RGB caps -> appsink
```

mac preview:

```text
avfvideosrc -> videoconvert -> videoscale -> RGB caps -> tee
  tee -> queue -> appsink
  tee -> queue -> videoconvert -> cairooverlay -> autovideosink
```

rpi headless:

```text
v4l2src -> jpegdec -> videoconvert -> videoscale -> RGB caps -> appsink
```

rpi preview:

```text
v4l2src -> jpegdec -> videoconvert -> videoscale -> RGB caps -> tee
  tee -> queue -> appsink
  tee -> queue -> videoconvert -> cairooverlay -> autovideosink
```

Example output:

```text
Frame    42 | green object x= 312.4 y= 220.7 | bbox x=275 y=184 w=76 h=73 | pixels=2894
Frame    43 | no green object | green pixels=12
```

The Raspberry Pi versions currently expect an MJPEG `v4l2src` camera, matching the lab06 approach. If the final camera outputs raw frames, remove `jpegdec` from the RPi source chain.
