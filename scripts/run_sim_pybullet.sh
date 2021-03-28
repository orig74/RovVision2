#!/bin/bash
echo 1
echo $HEADLESS
echo 2
if [ ! -v ROV_TYPE ]; then
    export ROV_TYPE=1
else
    export ROV_TYPE=$ROV_TYPE
fi

SIM=PB
PYTHON=python 
PROJECT_PATH=`git rev-parse --show-toplevel`
source run_common.sh

tmux kill-session -t dronelab
tmux new-session -d -s dronelab
tmux set -g pane-border-status top
#tmux send-keys "cd ../../dockers/unreal_engine_4 && ./attach.sh" ENTER

################## sim part
new_4_win
run 1 sim pybullet_bridge.py
tmux select-pane -t 2

sleep 1

################## onboard part
source run_onboard.sh
sleep 2

################## ground control part
if [ ! -v HEADLESS ]; then
    source run_ground_control.sh
fi
tmux select-window -t 0
run 3 sim depth_image_view.py
#tmux set -g mouse on
tmux att

