# JIWY RPI controller architecture

This application is split so the generated 20-sim controller code, the motor
communication backend, and future target sources such as vision can change
independently.

## Data flow

```text
target provider
    -> ControlTarget in radians
    -> twentysim_controller_step()
    -> normalized yaw/pitch outputs
    -> controller_output_to_command()
    -> MotorCommand
    -> MotorComm backend
    -> FPGA motor driver

FPGA encoder counters
    -> MotorComm backend
    -> EncoderSample
    -> count-to-radian conversion
    -> generated 20-sim feedback input
```

## Startup sequence

1. `main.c` opens the selected `MotorComm` backend.
2. `homing_run()` moves yaw and pitch one at a time to the configured mechanical
   home direction and records the encoder counts as software zero.
3. If `--hold` is supplied, `main.c` reads the current encoder sample,
   initializes `TwentySimController` with that feedback and the initial target,
   then starts `control_loop_run()` at 100 Hz using a fixed target of yaw
   `0 rad`, pitch `0 rad`.

## Extension points

### Communication backend

`motor_comm.h` is the hardware boundary. A backend only needs to implement:

```c
int exchange(void *context, MotorCommand command, EncoderSample *sample);
```

The current backend is `rpi_spi_master.c`. A DE10/Avalon backend should keep the
same `MotorCommand` and `EncoderSample` structures so the controller and vision
code do not need to change.

### Target provider

`control_loop.h` defines `ControlTargetProvider`. This is where future behavior
should be plugged in:

- fixed-position hold
- manual joystick input
- scripted setpoints
- vision target tracking

A vision provider should avoid blocking the motor loop. The expected pattern is
to process camera frames elsewhere, store the latest valid image error, and let
the provider convert that latest value into yaw/pitch target radians.

## Calibration and tuning

`jiwy_config.h` contains the values that should change during lab testing:

- encoder travel counts and travel degrees
- software travel limits
- yaw and pitch PID constants
- controller output clamps

The generated files in `controller/yaw_controller/` and
`controller/pitch_controller/` should stay replaceable. If 20-sim code is
regenerated, copy the new files in and keep local tuning in `jiwy_config.h`.

## Timing assumptions

The generated controllers currently use a 10 ms sample time. `main.c` derives
the controller step size from `control_loop_default_config()` and passes it to:

```c
twentysim_controller_init(&controller, 0.01, ...);
```

and `control_loop_default_config()` uses `10000 us`. Keep these matched unless
you also change the model sample time in 20-sim.

The SPI loop sends the command computed during the previous sample while reading
the current encoder sample. This is a one-sample command delay, but it keeps the
communication to one full-duplex transaction per control cycle.
