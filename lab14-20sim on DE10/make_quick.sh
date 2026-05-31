#!/bin/env bash
make clean
make 
./jiwy_controller 100000 100 --hold --log yaw_pitch_test.csv