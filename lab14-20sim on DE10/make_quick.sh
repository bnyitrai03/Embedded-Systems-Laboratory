#!/bin/env bash
make clean
make 
./jiwy_controller 100 --hold --log yaw_pitch_test.csv