# JIWY RPI controller architecture

This application is split so the generated 20-sim controller code, the motor
communication backend, and future target sources such as vision can change
independently.

## Data flow

```text
control target
    -> ControlTarget in radians
    -> twentysim_controller_step()
    -> normalized yaw/pitch outputs
    -> controller_output_to_command()
    -> MotorCommand
    -> MotorComm backend
    -> FPGA motor driver

optional camera tracker
    -> VisionTargetSnapshot yaw/pitch error in radians
    -> latest target selection in control_loop_run()

FPGA encoder counters
    -> MotorComm backend
    -> EncoderSample
    -> count-to-radian conversion
    -> generated 20-sim feedback input
```

## Startup sequence

1. `main.c` opens the selected `MotorComm` backend.
2. `homing_run()` moves yaw and pitch one at a time to both mechanical stops,
   measures encoder travel, and records the configured home-side stop as
   software zero.
3. If `--hold` or `--track` is supplied, `main.c` reads the current encoder sample,
   initializes `TwentySimController` with that feedback and the initial target,
   then starts `control_loop_run()` at 100 Hz.
4. `--hold` uses a fixed target of yaw `0 rad`, pitch `0 rad`. `--track`
   starts `vision_tracker` first and lets the control loop derive targets from
   the latest camera error.

## Extension points

### Communication backend

`motor_comm.h` is the hardware boundary. A backend only needs to implement:

```c
int exchange(void *context, MotorCommand command, EncoderSample *sample);
```

The current backend is `rpi_spi_master.c`. A DE10/Avalon backend should keep the
same `MotorCommand` and `EncoderSample` structures so the controller and vision
code do not need to change.

### Control target

`control_loop_run()` accepts a fixed `ControlTarget` and an optional
`VisionTracker`. A null tracker keeps the hold loop simple during motor
bring-up. A non-null tracker updates the target from the latest camera result.
Other target sources can follow the same pattern:

- fixed-position hold
- manual joystick input
- scripted setpoints
- vision target tracking with `--track`

Vision avoids blocking the motor loop by processing camera frames on its own
thread. The control loop only copies a small mutex-protected snapshot. When the
green object is detected, target radians are computed as current encoder angle
plus camera error. When it is not detected, the target is the current encoder
angle.

## Calibration and tuning

`jiwy_config.h` contains the values that should change during lab testing:

- physical travel degrees
- software travel limits

`controller/jiwy_20sim_tuning.h` contains yaw and pitch PID constants and
controller output clamps used by the generated model parameter initializers. If
20-sim code is regenerated, reapply the small include/macro edits in
`yaw_model.c` and `pitch_model.c` so the generated initializers continue using
that tuning header.

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
