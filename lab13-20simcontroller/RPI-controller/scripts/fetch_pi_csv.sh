#!/usr/bin/env bash
set -euo pipefail

PI_HOST="${PI_HOST:-esl@esl.local}"
PI_DIR="${PI_DIR:-~/ws/Embedded-Systems-Laboratory/lab13-20simcontroller/RPI-controller}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
LOCAL_DIR="${LOCAL_DIR:-${PROJECT_DIR}}"
CSV_PATTERN="${CSV_PATTERN:-*.csv}"
PYTHON="${PYTHON:-python3}"
PLOT="${PLOT:-1}"

latest_csv="$(
    ssh "$PI_HOST" "cd $PI_DIR && ls -t $CSV_PATTERN 2>/dev/null | head -n 1"
)"

if [ -z "$latest_csv" ]; then
    echo "No CSV files found on $PI_HOST:$PI_DIR" >&2
    exit 1
fi

echo "Copying $latest_csv from $PI_HOST..."
scp "$PI_HOST:$PI_DIR/$latest_csv" "$LOCAL_DIR/"
echo "Saved to $LOCAL_DIR/$latest_csv"

if [ "$PLOT" != "0" ]; then
    echo "Opening plots..."
    "$PYTHON" "${SCRIPT_DIR}/plot_pid_log.py" "$LOCAL_DIR/$latest_csv"
fi
