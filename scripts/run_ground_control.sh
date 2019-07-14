#!/bin/bash
source run_common.sh
PROJECT_PATH=/home/host/projects/RovVision2/
PYTHON=/miniconda/bin/python 

if [ ! -v SIM ]
then
tmux kill-session -t dronelab
tmux new-session -d -s dronelab
PYTHON=/bin/python3
PROJECT_PATH=../
else
tmux new-window
fi 

new_4_win
run 0 ground_control joy_rov.py
run 1 ground_control viewer.py

if [ ! -v SIM ]
then 
tmux att
fi
