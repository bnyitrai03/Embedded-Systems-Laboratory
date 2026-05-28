#!/usr/bin/env bash
set -e

echo "[1/3] Compiling with iverilog..."
iverilog -o TopEntity_tb.vvp TopEntity_tb.v TopEntity.v SPI.v QuadDecoder.v PWM.v

echo "[2/3] Running simulation..."
vvp TopEntity_tb.vvp

echo "[3/3] Opening GTKWave..."
gtkwave TopEntity_tb.vcd