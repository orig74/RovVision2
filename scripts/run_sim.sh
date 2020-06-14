#!/bin/bash
echo 1
echo $HEADLESS
echo 2

if [ ! -v ROV_TYPE ]; then
    export ROV_TYPE=1
else
    export ROV_TYPE=$ROV_TYPE
fi
#GAME_PATH=/DroneLab/baked_games/Ocean1_packed/LinuxNoEditor/
#GAME_PATH=/project_files/Ocean2_packed/LinuxNoEditor/
GAME_PATH=/project_files/Ocean2_packed_2.24.3/LinuxNoEditor/
PACKED_NAME=Ocean2
PROJECT_PATH=/home/host/projects/RovVision2
DRONESIMLAB_PATH=../../DroneSimLab/
SIM=
PYTHON=python 

source run_common.sh

tmux kill-session -t dronelab
tmux new-session -d -s dronelab
tmux set -g pane-border-status top
#tmux send-keys "cd ../../dockers/unreal_engine_4 && ./attach.sh" ENTER

################## sim part
new_4_win
run 0 sim ue4_bridge.py
run 1 sim run_dynamic_sim.py
tmux select-pane -t 2
init_docker_image
#tmux send-keys "unset DISPLAY;cd $GAME_PATH;PATH=/miniconda/bin/:$PATH ./run.sh" ENTER
ENTRY_POINT=unreal_proxy
ENTRY_PATH=unreal_proxy/

tmux send-keys "cd ${PROJECT_PATH}/sim" ENTER
tmux send-keys "python3 $DRONESIMLAB_PATH/UE4PyhtonBridge/set_path.py --entry_point $ENTRY_POINT --entry_path $ENTRY_PATH --packed_game_name $PACKED_NAME --packed_game_path $GAME_PATH" ENTER
tmux send-keys "cd ${GAME_PATH}" ENTER
tmux send-keys "cd $GAME_PATH; ./run.sh" ENTER

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

