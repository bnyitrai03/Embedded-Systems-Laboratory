# RPI controller architecture

This folder keeps the 20-sim generated controllers behind small local adapters.
See `ARCHITECTURE.md` for the control/data-flow explanation.

- `control_protocol.*`: packs/unpacks the 32-bit motor/encoder frame.
- `motor_comm.h`: communication interface used by the controller application.
- `rpi_spi_master.*`: Raspberry Pi spidev implementation of that interface.
- `controller_adapter.*`: converts encoder counts to radians, calls the generated yaw/pitch controllers, and converts normalized controller output to PWM commands.
- `control_loop.*`: time-driven position-control loop with a replaceable target provider.
- `homing.*`: sequential yaw/pitch software homing routine.
- `main.c`: starts SPI, runs homing, disables the motors, and prints the software home offsets.
- `jiwy_config.h`: measured encoder travel calibration and initial software limits.
- PID tuning values also live in `jiwy_config.h`; do not edit generated
  `controller/*_controller.c` files for tuning.

The SPI frame is big-endian:

```text
TX byte 0..1: yaw command   bit15 direction, bit14 enable, bit13..0 PWM
TX byte 2..3: pitch command bit15 direction, bit14 enable, bit13..0 PWM
RX byte 0..1: yaw encoder   signed int16
RX byte 2..3: pitch encoder signed int16
```

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
./jiwy_controller 100000 1500
```

Run homing and then hold the home position with the 20-sim controllers:

```sh
./jiwy_controller 100000 1500 --hold
```

Build the standalone SPI smoke test:

```sh
make smoke-test
```

The homing routine has no limit-switch input. It moves one axis at a time with
low PWM and treats a stable encoder count as the mechanical end stop. Keep the
first test at low PWM and be ready to interrupt with Ctrl-C.

Safety assumptions to verify on the real setup:

- homing direction moves toward the intended mechanical end stop
- positive controller output increases encoder counts
- configured travel limits match the measured JIWY
- homing PWM is low enough that pushing into the end stop is acceptable

The default calibration uses the measured travel values:

```text
yaw:   2453 counts over 180 degrees
pitch:  684 counts over 175 degrees
```

Edit `jiwy_config.h` when you remeasure the setup or want to test different
travel assumptions. These values affect count-to-radian conversion and the
target clamps used by the 20-sim controller adapter.

For controller tuning, edit the named PID constants in `jiwy_config.h`, rebuild,
and test again. The generated 20-sim parameters are:

```text
kp      proportional gain
tauD    derivative time
beta    derivative filter factor
tauI    integral time; larger means weaker/slower integral action
min/max normalized controller output clamp, later mapped to PWM
```

To add vision later, implement a new `ControlTargetProvider` and pass it to
`control_loop_run`. The motor loop can stay at 100 Hz while the vision provider
reuses the latest detected target between camera frames.
