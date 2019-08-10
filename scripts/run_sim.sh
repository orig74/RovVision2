#!/bin/bash
echo 1
echo $HEADLESS
echo 2
#GAME_PATH=/DroneLab/baked_games/Ocean1_packed/LinuxNoEditor/
GAME_PATH=/project_files/Ocean2_packed/LinuxNoEditor/
PROJECT_PATH=/home/host/projects/RovVision2/
DRONESIMLAB_PATH=../../DroneSimLab/
SIM=
PYTHON=/miniconda/bin/python 

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
source run_onboard.sh
sleep 2

################## ground control part
if [ ! -v HEADLESS ]; then
    source run_ground_control.sh
fi

tmux select-window -t 0
#tmux set -g mouse on
tmux att

