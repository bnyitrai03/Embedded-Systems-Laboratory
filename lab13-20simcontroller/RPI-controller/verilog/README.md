# FPGA motor SPI interface

This Verilog is intended to match the Raspberry Pi smoke test and controller
protocol in `../control_protocol.*`.

## SPI frame

SPI mode: mode 0, 8-bit words from Linux/spidev, one 32-bit transaction.

```text
Raspberry Pi -> FPGA, big-endian:
  bits 31..16: yaw command
    bit 15    direction
    bit 14    enable
    bit 13..0 PWM duty, currently clamped by C code to 0..2500 ticks
  bits 15..0: pitch command
    bit 15    direction
    bit 14    enable
    bit 13..0 PWM duty, currently clamped by C code to 0..2500 ticks

FPGA -> Raspberry Pi, big-endian:
  bits 31..16: signed int16 yaw encoder count
  bits 15..0:  signed int16 pitch encoder count
```

The FPGA only commits motor commands after a complete 32-bit frame. Short
transfers leave the previous command active.

## Current modules

- `TopEntity.v`: wires SPI, two quadrature decoders, and two PWM drivers.
- `SPI.v`: 32-bit protocol parser and encoder response shifter.
- `QuadDecoder.v`: signed 16-bit quadrature counter.
- `PWM.v`: direction pins plus PWM output for one motor.

## Bring-up checklist

1. Build/program this FPGA design.
2. Run the RPI smoke test with both PWM values at zero and confirm stable encoder
   values:

   ```sh
   ./test/spi_motor_smoke_test 100000 0 1 0 1 20 50
   ```

3. Move only yaw with low PWM and check that yaw counts change while pitch stays
   near constant:

   ```sh
   ./test/spi_motor_smoke_test 100000 1500 1 0 1 200 10
   ```

4. Repeat with the opposite yaw direction bit. Counts should move the opposite
   way.
5. Repeat for pitch:

   ```sh
   ./test/spi_motor_smoke_test 100000 0 1 1500 1 200 10
   ```

If a direction bit moves the axis opposite to the C-side convention, flip that
mapping either in `PWM.v` or in the C protocol adapter, but keep it consistent.
