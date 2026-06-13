#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
VERILOG_DIR="${PROJECT_DIR}/verilog"

cd "${VERILOG_DIR}"

echo "[1/3] Compiling with iverilog..."
iverilog -o TopEntity_tb.vvp TopEntity_tb.v TopEntity.v SPI.v QuadDecoder.v PWM.v

echo "[2/3] Running simulation..."
vvp TopEntity_tb.vvp

echo "[3/3] Opening GTKWave..."
gtkwave TopEntity_tb.vcd
