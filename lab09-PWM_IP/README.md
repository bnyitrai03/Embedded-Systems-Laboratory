# Lab 09 - PWM IP notes

Goal: make a small PWM IP for one VNH2SP30-E motor driver.

The driver is an H-bridge. The FPGA should only drive the logic inputs, not the motor directly.

## Files

- `PWM.v`: pure PWM logic for one motor.
- `PWM_wrapper.v`: Avalon-MM wrapper around `PWM.v`.
- `PWM_tb.v`: simple simulation testbench for `PWM.v`.
- `pwm_hw.tcl`: Platform Designer component description for `PWM_wrapper`.
- `DE10_NANO_TOP.v`: top-level board wrapper.
- `soc_system.h`: generated software header for the HPS address map.
- `main.c`: small Linux `/dev/mem` test program.

## Required outputs

Assignment 9 asks for three outputs:

- `INA`: direction input A.
- `INB`: direction input B.
- `C`: PWM speed signal.

In the VNH2SP30-E datasheet and schematic, `C` is basically the `PWM` input.

In Platform Designer these conduits were exported as:

| IP conduit | Export name |
| --- | --- |
| `ina` | `in_a` |
| `inb` | `in_b` |
| `pwm_c` | `pwm` |

## Avalon registers

Current wrapper has one motor only.

| Address | Register | Use |
| --- | --- | --- |
| 0 | control | bit 0 = enable, bit 1 = direction |
| 1 | duty | raw PWM duty value |

Default clock assumptions:

```text
CLK_FREQ = 50 MHz
PWM_FREQ = 20 kHz
PWM_PERIOD = 2500 ticks
```

Useful duty values:

| Duty | Meaning |
| --- | --- |
| 0 | 0 percent |
| 1250 | about 50 percent |
| 2500 | 100 percent |

Keep duty between `0` and `2500` for the current settings.

## Direction table

From the VNH2SP30-E datasheet, normal operation with enable pins high:

| INA | INB | Mode |
| --- | --- | --- |
| 1 | 0 | Clockwise |
| 0 | 1 | Counter-clockwise |
| 1 | 1 | Brake to VCC |
| 0 | 0 | Brake to GND |

Current module uses:

```text
dir = 0 -> INA = 1, INB = 0
dir = 1 -> INA = 0, INB = 1
enable = 0 -> INA = 0, INB = 0, C = 0
```

If the motor turns the wrong physical direction, flip the software direction bit or swap the mapping later.

## Pin names on DE10/JIWY

The pinout file uses project names instead of datasheet names.

| Axis | Datasheet signal | Project signal | FPGA pin |
| --- | --- | --- | --- |
| Pitch | `INA` | `PITCH_DIRA` | `PIN_AH24` |
| Pitch | `INB` | `PITCH_DIRB` | `PIN_AG25` |
| Pitch | `PWM` / `C` | `PITCH_PWM_VAL` | `PIN_AG23` |
| Yaw | `INA` | `YAW_DIRA` | `PIN_AA15` |
| Yaw | `INB` | `YAW_DIRB` | `PIN_Y15` |
| Yaw | `PWM` / `C` | `YAW_PWM_VAL` | `PIN_AG28` |

This IP is one motor only. For pitch and yaw together, instantiate it twice or make a two-channel wrapper.

Current top-level wiring uses the pitch motor:

```verilog
.in_a_export(PITCH_DIRA),
.in_b_export(PITCH_DIRB),
.pwm_export(PITCH_PWM_VAL)
```

To test yaw instead, connect those exports to `YAW_DIRA`, `YAW_DIRB`, and `YAW_PWM_VAL`.

## Platform Designer notes

Connections:

- `clock_reset` connected to `clk_0`.
- `clock_reset_reset` connected to the reset from `clk_0`.
- `s0` connected to the HPS lightweight AXI master.
- `ina`, `inb`, and `pwm_c` exported.

Regenerate HDL after changes. This creates/updates `soc_system.sopcinfo`.

## Generating `soc_system.h`

Use the Windows tools directly:

```bat
cd /d G:\Quartus\Projects\ip
"G:\Quartus\Quartus\quartus\sopc_builder\bin\sopcinfo2swinfo.exe" --input=soc_system.sopcinfo --output=soc_system.swinfo
"G:\Quartus\Quartus\quartus\sopc_builder\bin\swinfo2header.exe" --swinfo soc_system.swinfo --sopc soc_system.sopcinfo --single soc_system.h --module hps_0_arm_a9_0
```

The header should contain:

```c
#define PWM_WRAPPER_0_BASE 0xff200000
#define PWM_WRAPPER_0_SPAN 1024
```

Use these macros in `main.c`.

## `main.c` test program

`main.c` maps the PWM IP through `/dev/mem` and writes two 32-bit registers:

```c
pwm_regs[0] = control;
pwm_regs[1] = duty;
```

Usage:

```bash
./main            # enable=1, direction=0, duty=1250
./main 1250 0 1   # 50 percent, direction 0, enabled
./main 2500 1 1   # 100 percent, direction 1, enabled
./main 0 0 0      # disabled
```

Run as root because it opens `/dev/mem`.


## What to show

- `C` is around `20 kHz`.
- Changing duty changes PWM high time.
- Changing direction changes `INA` and `INB`.
- Disabling the module drives everything low.

Oscilloscope check is enough for the first demo.
