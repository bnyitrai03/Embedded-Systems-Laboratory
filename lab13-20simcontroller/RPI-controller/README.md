# RPI controller architecture

This folder keeps the 20-sim generated controllers behind small local wrappers.
See `ARCHITECTURE.md` for the control/data-flow explanation.

- `control_protocol.*`: packs/unpacks the 32-bit motor/encoder frame.
- `motor_comm.h`: communication interface used by the controller application.
- `rpi_spi_master.*`: Raspberry Pi spidev implementation of that interface.
- `jiwy_calibration.*`: converts encoder counts to radians and owns measured travel spans, software home offsets, travel limits, and target clamps.
- `twentysim_controller.*`: initializes and steps the generated yaw/pitch 20-sim submodels and converts normalized controller output to PWM commands.
- `control_loop.*`: time-driven position-control loop for one yaw/pitch target.
- `homing.*`: sequential yaw/pitch software homing routine.
- `main.c`: starts SPI, runs homing, initializes the controller after homing when `--hold` is requested, and prints the software home offsets.
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

To add vision later, replace the fixed `ControlTarget` passed to
`control_loop_run` with target values derived from the latest camera result.
The motor loop can stay at 100 Hz while the vision code reuses the latest
detected target between camera frames.
# PID CSV logging

When running the hold loop, the controller writes a CSV log by default:

```bash
./jiwy_controller 100000 500 --hold
```

This creates:

```text
pid_log.csv
```

To choose another path:

```bash
./jiwy_controller 100000 500 --hold --log yaw_pitch_test.csv
```

Plot after the run:

```bash
python3 plot_pid_log.py pid_log.csv
```

Install plotting dependencies if needed:

```bash
python3 -m pip install matplotlib
```
