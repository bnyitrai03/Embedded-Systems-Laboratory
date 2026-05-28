#!/usr/bin/env bash
set -euo pipefail

echo "[WARNING] DELIVERED AS IS. Understand the commands before running this script."

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
if [ -z "${ICOPROG_DIR:-}" ]; then
    ICOPROG_DIR=~/ws/icoprog
fi

JSON_FILE="${SCRIPT_DIR}/ice40.json"
ASC_FILE="${SCRIPT_DIR}/ice40.asc"
BIN_FILE="${SCRIPT_DIR}/ice40.bin"
ICOPROG_BIN="${ICOPROG_DIR}/ice40.bin"

SPI_DISABLED=0

cleanup() {
    if [ "${SPI_DISABLED}" -eq 1 ]; then
        sudo dtparam spi=on || true
    fi
}
trap cleanup EXIT

echo "[1/7] Synthesizing TopEntity..."
cd "${SCRIPT_DIR}"
yosys -p 'synth_ice40 -top TopEntity -json ice40.json' \
    TopEntity.v \
    SPI.v \
    QuadDecoder.v \
    PWM.v

echo "[2/7] Placing and routing..."
nextpnr-ice40 --hx8k --json ice40.json --pcf ico-jiwy.pcf --asc ice40.asc

echo "[3/7] Packing bitstream..."
icepack ice40.asc ice40.bin

echo "[4/7] Copying bitstream to ${ICOPROG_DIR}..."
cd "${ICOPROG_DIR}"
cp -f "${BIN_FILE}" "${ICOPROG_BIN}"

echo "[5/7] Programming FPGA..."
sudo dtparam spi=off
SPI_DISABLED=1
./icoprog -R
./icoprog -p < ice40.bin
sudo dtparam spi=on
SPI_DISABLED=0

echo "[6/7] Building Raspberry Pi controller..."
cd "${PROJECT_DIR}"
make

echo "[7/7] Cleaning generated FPGA build files..."
rm -f "${JSON_FILE}" "${ASC_FILE}" "${BIN_FILE}"

echo "ICE40 board programmed and controller rebuilt."
echo "[IMPORTANT] The controller program has NOT been executed."
