# RPI controller architecture

This folder keeps the 20-sim generated controllers behind small local wrappers.
See `ARCHITECTURE.md` for the control/data-flow explanation.

- `control_protocol.*`: packs/unpacks the 32-bit motor/encoder frame.
- `motor_comm.h`: communication interface used by the controller application.
- `rpi_spi_master.*`: Raspberry Pi spidev implementation of that interface.
- `jiwy_calibration.*`: converts encoder counts to radians and owns measured travel spans, software home offsets, travel limits, and target clamps.
- `twentysim_controller.*`: initializes and steps the generated yaw/pitch 20-sim submodels and converts normalized controller output to PWM commands.
- `control_loop.*`: time-driven position-control loop for one yaw/pitch target.
- `vision_blob.*`: green thresholding, largest-blob selection, and centroid calculation.
- `vision_geometry.*`: centroid pixel position to yaw/pitch camera error in radians.
- `vision_stream.*`: low-rate HTTP debug stream and annotated-frame rendering.
- `vision_tracker.*`: optional Raspberry Pi GStreamer camera thread and snapshot publishing.
- `homing.*`: sequential yaw/pitch software homing routine.
- `main.c`: starts SPI, runs homing, then optionally runs scheduled hold calibration with `--hold` or vision tracking with `--track`.
- `jiwy_config.h`: central non-PID configuration for SPI, homing, control-loop, vision, and travel defaults.
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

Run homing and then alternate through the configured PID-calibration setpoints:

```sh
./jiwy_controller --hold
```

Run homing and then track a green object with the camera:

```sh
./jiwy_controller --track
```

Build the minimal vision smoke runner. It uses the same vision blob, geometry,
and tracker code as the robot path, but swaps only the camera source for macOS:

```sh
make mac-smoke
./vision/test/vision_tracker_smoke
```

The smoke runner prints the latest yaw/pitch snapshot once per second and, by
default, serves the same annotated frame at `http://127.0.0.1:8080/`. On the
Raspberry Pi, build the same runner against the normal V4L2 camera path:

```sh
make vision-smoke
./vision/test/vision_tracker_smoke
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

Edit `jiwy_config.h` to change controller behavior such as SPI defaults,
homing settings, vision settings, travel limits, and the two hold-calibration
targets and durations.
Encoder travel counts are still measured during homing.

The default hold schedule uses radian expressions derived in the header:

```c
#define JIWY_HOLD_TARGET1_YAW_RAD   (JIWY_YAW_MAX_RAD * 0.25)
#define JIWY_HOLD_TARGET1_PITCH_RAD (JIWY_PITCH_MAX_RAD * 0.25)
#define JIWY_HOLD_TARGET1_DURATION_S 10.0

#define JIWY_HOLD_TARGET2_YAW_RAD   (JIWY_YAW_MAX_RAD * 0.75)
#define JIWY_HOLD_TARGET2_PITCH_RAD (JIWY_PITCH_MAX_RAD * 0.75)
#define JIWY_HOLD_TARGET2_DURATION_S 10.0
```

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
V4L2 camera at the configured resolution in `jiwy_config.h` and decodes frames
through GStreamer:

```text
v4l2src -> image/jpeg caps -> jpegdec -> videoconvert -> videoscale -> RGB -> appsink
```

`vision_blob.c` scans RGB pixels into a green mask, finds connected green blobs,
rejects small or thin blobs, and uses the center of the largest valid blob as
the ball target. A short exponential smoother is applied after selection.
`vision_geometry.c` converts the filtered blob center to camera error using the
configured horizontal and vertical camera field of view:

```c
yaw_error_rad = atan(normalized_x * tan(horizontal_fov_rad / 2.0));
pitch_error_rad = atan(normalized_y * tan(vertical_fov_rad / 2.0));
```

Positive yaw means the object is right of center. Positive pitch means the
object is above center. The raw angular error is reported without a deadband;
limit-cycle prevention at center is handled by the world-estimate update gate
in the control loop (see below).

The control loop builds the setpoint in a fixed world frame so that slewing the
camera toward the ball does not perturb its own target (the sensor-latency
positive feedback that makes the camera oscillate). When a new valid camera
frame arrives, the ball world angle is reconstructed as:

```text
ball_world = camera_angle_at_capture + camera error
```

`camera_angle_at_capture` is looked up by linear interpolation in a short
encoder-history ring using the frame's `CLOCK_MONOTONIC` capture timestamp, not
the current camera angle. That world estimate is then:

1. Low-pass filtered (`JIWY_VISION_WORLD_FILTER_ALPHA`) to reject detection
   noise.
2. Deadband-gated at the update (`JIWY_VISION_YAW/PITCH_DEADBAND_RAD`): the
   estimate only folds in a new measurement when it moves beyond the deadband,
   so a centered ball produces a constant setpoint instead of a limit cycle.
3. Optionally led by a velocity feedforward
   (`JIWY_VISION_FEEDFORWARD_GAIN_*`) to reduce lag of a moving ball.
4. Rate-limited (`JIWY_VISION_SETPOINT_MAX_RATE_RAD_PER_S`) so the per-frame
   estimate change is handed to the PID as a smooth ramp instead of a step.

If no ball is detected, or the latest valid frame is not refreshed for
`JIWY_VISION_MAX_STALE_CONTROL_SAMPLES` control samples, the ramped target
freezes at the last known ball direction so the camera holds where the ball
was, instead of snapping to the current camera angle.

# PID CSV logging

The controller is quiet by default. It does not write CSV logs or camera debug
logs unless requested.

```bash
./jiwy_controller --hold --log pid_log.csv
```

The same CSV logging is available while tracking:

```bash
./jiwy_controller --track --log yaw_pitch_test.csv
```

Status logging, camera diagnostics, and vision streaming are configured through
`jiwy_config.h`.

The stream binds to `127.0.0.1:8080` on the Raspberry Pi and shows one
annotated raw camera frame: detection status, selected-blob circle, centroid
crosshair, image-center marker, yaw/pitch error, blob/green pixels, processing
time, and frame interval. From your PC, open a second terminal and forward that
port:

```bash
ssh -L 8080:127.0.0.1:8080 esl@esl.local
```

Then open:

```text
http://127.0.0.1:8080/
```

To use another port, change the stream port in `jiwy_config.h` and then forward that same port:

```bash
./jiwy_controller --track
ssh -L 8081:127.0.0.1:8081 esl@esl.local
```

Plot after the run:

```bash
python3 scripts/plot_pid_log.py pid_log.csv
```

Install plotting dependencies if needed:

```bash
python3 -m pip install matplotlib
```
