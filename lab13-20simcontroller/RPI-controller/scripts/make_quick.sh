#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

cd "${PROJECT_DIR}"
make clean
make
#./jiwy_controller 100000 150 --track --status-every 100
#./jiwy_controller 100000 200 --track --status-every 100 --vision-stream --log yaw_pitch_test.csv
#./jiwy_controller 100000 200 --hold --status-every 100 --vision-stream --log yaw_pitch_test.csv
sudo ./jiwy_controller --hold --log yaw_pitch_test.csv
