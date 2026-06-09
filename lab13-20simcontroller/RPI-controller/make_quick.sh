#!/bin/env bash
make clean
make 
#./jiwy_controller 100000 150 --track --status-every 100
# ./jiwy_controller 100000 200 --track --status-every 100  --vision-stream --log yaw_pitch_test.csv
./jiwy_controller 100000 200 --hold --status-every 100  --vision-stream --log yaw_pitch_test.csv
