#!/bin/bash
echo 1
echo $HEADLESS
echo 2
if [ ! -v ROV_TYPE ]; then
    export ROV_TYPE=4
else
    export ROV_TYPE=$ROV_TYPE
fi

export SIM=PB
if  [ ! -v SIM_STREAM_TYPE ]; then
    export SIM_STREAM_TYPE=ZMQ
fi

export SIM_STREAM_TYPE=${SIM_STREAM_TYPE:-$ZMQ}
echo running sim with $SIM_STREAM_TYPE
#export SIM_STREAM_TYPE=GST
sleep 1 
PYTHON=python 
export RESIZE_VIEWER=1800
PROJECT_PATH=`git rev-parse --show-toplevel`
source run_common.sh

tmux kill-session -t dronelab
tmux new-session -d -s dronelab
tmux set -g pane-border-status top
#tmux send-keys "cd ../../dockers/unreal_engine_4 && ./attach.sh" ENTER

################## sim part
new_4_win
run 1 sim "pybullet_bridge2.py --show_gui"
#run 1 sim "pybullet_bridge.py"
tmux select-pane -t 2

sleep 1

wmctrl -r "Bullet Phys" -t 1
################## onboard part
source run_onboard.sh
sleep 1

################## ground control part
if [ ! -v HEADLESS ]; then
    source run_ground_control.sh
fi
tmux select-window -t 0
#run 3 sim depth_image_view.py
#tmux set -g mouse on
tmux att

