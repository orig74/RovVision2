#!/bin/bash

#GAME_PATH=/DroneLab/baked_games/Ocean1_packed/LinuxNoEditor/
GAME_PATH=/project_files/Ocean1_packed/LinuxNoEditor/
PROJECT_PATH=/home/host/projects/RovVision2/
DRONESIMLAB_PATH=../../DroneSimLab/

source run_common.sh

tmux kill-session -t dronelab
tmux new-session -d -s dronelab

#tmux send-keys "cd ../../dockers/unreal_engine_4 && ./attach.sh" ENTER

################## sim part
new_4_win
run 0 sim ue4_bridge.py
run 1 sim run_dynamic_sim.py
tmux select-pane -t 2
init_docker_image
tmux send-keys "cd $GAME_PATH;PATH=/miniconda/bin/:$PATH ./run.sh" ENTER

sleep 1

################## onboard part
tmux new-window 
new_6_win
run 0 onboard controller.py
run 1 onboard sensors_gate.py

sleep 2

################## ground control part
tmux new-window 
new_4_win
run 0 ground_control joy_rov.py
run 1 ground_control viewer.py


tmux select-window -t 0
#tmux set -g mouse on
tmux att

