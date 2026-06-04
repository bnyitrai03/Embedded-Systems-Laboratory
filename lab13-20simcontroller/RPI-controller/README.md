# RPI controller architecture

This folder keeps the 20-sim generated controllers behind small local wrappers.
See `ARCHITECTURE.md` for the control/data-flow explanation.

- `control_protocol.*`: packs/unpacks the 32-bit motor/encoder frame.
- `motor_comm.h`: communication interface used by the controller application.
- `rpi_spi_master.*`: Raspberry Pi spidev implementation of that interface.
- `jiwy_calibration.*`: converts encoder counts to radians and owns measured travel spans, software home offsets, travel limits, and target clamps.
- `twentysim_controller.*`: initializes and steps the generated yaw/pitch 20-sim submodels and converts normalized controller output to PWM commands.
- `control_loop.*`: time-driven position-control loop for one yaw/pitch target.
- `vision_tracker.*`: optional GStreamer green-object tracker that publishes yaw/pitch camera error in radians.
- `homing.*`: sequential yaw/pitch software homing routine.
- `main.c`: starts SPI, runs homing, then optionally runs fixed hold with `--hold` or vision tracking with `--track`.
- `jiwy_config.h`: physical travel angles, PWM limit, and initial software limits.
- `controller/jiwy_20sim_tuning.h`: yaw/pitch PID tuning values used by the
  generated 20-sim model initializers.

The SPI frame is big-endian:

```text
TX byte 0..1: yaw command   bit15 direction, bit14 enable, bit13..0 PWM
TX byte 2..3: pitch command bit15 direction, bit14 enable, bit13..0 PWM
RX byte 0..1: yaw encoder   signed int16
RX byte 2..3: pitch encoder signed int16
```

The PWM field has 14 protocol bits, and the FPGA PWM period is 2500 ticks at
50 MHz / 20 kHz. The C side intentionally clamps commands to `0..250`.

Direction convention in the C code is `0 = negative/decreasing encoder`,
`1 = positive/increasing encoder`. If the FPGA uses the opposite polarity,
swap the direction handling in `control_protocol.c` or in the FPGA.

Build on the Raspberry Pi with Make:

```sh
sudo apt install pkg-config libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
make
```

Or build with CMake:

```sh
cmake -S . -B build
cmake --build build
```

Run homing with defaults:

```sh
./jiwy_controller
```

Optional arguments are SPI speed and homing PWM:

```sh
./jiwy_controller 100000 250
```

Run homing and then hold the home position with the 20-sim controllers:

```sh
./jiwy_controller 100000 250 --hold
```

Run homing and then track a green object with the camera:

```sh
./jiwy_controller 100000 250 --track
```

The default camera is `/dev/video0`. Choose another V4L2 camera with:

```sh
./jiwy_controller 100000 250 --track --camera /dev/video1
```

Build the standalone SPI smoke test:

```sh
make smoke-test
```


Usage
```sh
./test/spi_motor_smoke_test [speed_hz] [yaw_pwm] [yaw_dir] [pitch_pwm] [pitch_dir] [loops] [period_ms]
```

The homing routine has no limit-switch input. It moves one axis at a time with
low PWM, visits both mechanical stops, and treats a stable encoder count as each
stop. The measured stop-to-stop encoder span is used for count-to-radian
calibration. Keep the first test at low PWM and be ready to interrupt with
Ctrl-C.

Safety assumptions to verify on the real setup:

- homing direction moves toward the intended mechanical end stop
- positive controller output increases encoder counts
- configured travel angles and limits match the measured JIWY
- homing PWM is low enough that pushing into the end stop is acceptable

The default calibration uses the configured physical travel angles:

```text
yaw:   240 degrees
pitch: 240 degrees
```

Edit `jiwy_config.h` when you remeasure the angular travel or want to test
different limit assumptions. Encoder travel counts are measured during homing.

For controller tuning, edit the named PID constants in
`controller/jiwy_20sim_tuning.h`, rebuild, and test again. The generated
`yaw_model.c` and `pitch_model.c` files include this hand-owned header in their
parameter initializer functions. If you regenerate 20-sim code, reapply that
small include/macro change. The generated 20-sim parameters are:

```text
kp      proportional gain
tauD    derivative time
beta    derivative filter factor
tauI    integral time; larger means weaker/slower integral action
min/max normalized controller output clamp, later mapped to PWM
```

Vision tracking is opt-in with `--track`. The camera thread expects an MJPEG
V4L2 camera at `640x480` and decodes frames through GStreamer:

```text
v4l2src -> image/jpeg caps -> jpegdec -> videoconvert -> videoscale -> RGB -> appsink
```

The tracker scans RGB pixels for green, averages the green pixel coordinates,
and converts the centroid to camera error using a fixed 60 degree field of
view. The units are pixels until the final multiply by radians:

```c
yaw_error_rad =
    ((object_x - center_x) / center_x) * (camera_fov_rad / 2.0);

pitch_error_rad =
    ((center_y - object_y) / center_y) * (camera_fov_rad / 2.0);
```

`camera_fov_rad / 2.0` is used because 60 degrees is the full image width or
height: the image center is 0 degrees, and each edge is 30 degrees from the
centerline. Positive yaw means the object is right of center. Positive pitch
means the object is above center.

The 100 Hz motor loop reads the latest camera result each sample. If the ball is
detected, the setpoint is:

```text
target = current encoder angle + camera error
```

If no ball is detected, the setpoint is the current encoder angle, so the robot
stops chasing stale camera data.

# PID CSV logging

The controller is quiet by default. It does not write CSV logs or camera debug
logs unless requested.

```bash
./jiwy_controller 100000 500 --hold --log pid_log.csv
```

The same CSV logging is available while tracking:

```bash
./jiwy_controller 100000 500 --track --log yaw_pitch_test.csv
```

To check control-loop timing without full CSV logging, print one status line
every N samples. At the default 100 Hz control rate, `100` means once per
second:

```bash
./jiwy_controller 100000 500 --track --status-every 100
```

Enable low-rate camera diagnostics with:

```bash
./jiwy_controller 100000 500 --track --vision-debug
```

Stream the camera frames used by the tracker with a red marker at the detected
green-object centroid:

```bash
./jiwy_controller 100000 500 --track --vision-stream
```

The stream binds to `127.0.0.1:8080` on the Raspberry Pi. From your PC, open a
second terminal and forward that port:

```bash
ssh -L 8080:127.0.0.1:8080 esl@esl.local
```

Then open:

```text
http://127.0.0.1:8080/
```

Use another port if needed:

```bash
./jiwy_controller 100000 500 --track --vision-stream --vision-stream-port 8081
ssh -L 8081:127.0.0.1:8081 esl@esl.local
```

Plot after the run:

```bash
python3 plot_pid_log.py pid_log.csv
```

Install plotting dependencies if needed:

```bash
python3 -m pip install matplotlib
```
