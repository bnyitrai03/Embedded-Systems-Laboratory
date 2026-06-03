#!/bin/env bash
make clean
make 
./jiwy_controller 100000 150 --track --status-every 100