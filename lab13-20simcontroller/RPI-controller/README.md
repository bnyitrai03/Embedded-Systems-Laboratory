# Programming the FPGA

Run the FPGA build, place-and-route, bitstream packing, and programming flow with:

```sh
./scripts/program_ice40.sh
```

The script synthesizes `TopEntity`, programs the ICE40 through `icoprog`, rebuilds
the Raspberry Pi controller, and leaves the controller stopped.

If your `icoprog` checkout is not in `~/ws/icoprog`, set `ICOPROG_DIR` first:

```sh
ICOPROG_DIR=/path/to/icoprog ./scripts/program_ice40.sh
```

Run the Verilog testbench with:

```sh
./scripts/run_verilog_tb.sh
```

This compiles `TopEntity_tb.v`, runs the simulation, and opens `gtkwave`.


# Building and running the RPI demo

Build on the Raspberry Pi with Make:

```sh
make
```

Run homing and then track a green object with the camera:

```sh
./jiwy_controller --track
```

Run homing and then alternate through the configured PID calibration setpoints:

```sh
./jiwy_controller --hold
```

# Build and run

Use this script to build and run:

```sh
./scripts/make_quick.sh
```

# PID CSV logging

The controller is quiet by default. It does not write CSV logs or camera debug
logs unless requested.

```bash
./jiwy_controller --track --log yaw_pitch_test.csv
```

Status logging, camera diagnostics, and vision streaming are configured through `jiwy_config.h`.

The stream binds to `127.0.0.1:8080` on the Raspberry Pi and shows one
annotated raw camera frame: detection status, selected-blob circle, centroid
crosshair, image-center marker, yaw/pitch error, blob pixels, processing
time, and frame interval. From your PC, open a second terminal and forward that
port:

```bash
ssh -L 8080:127.0.0.1:8080 esl@esl.local
```

Then open:

```text
http://127.0.0.1:8080/
```

Plot after the run:

```bash
python scripts/plot_pid_log.py pid_log.csv
```

Copy the newest CSV log from the Raspberry Pi and open the plots:

```bash
./scripts/fetch_pi_csv.sh
```
