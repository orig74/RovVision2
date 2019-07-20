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

#common for sim and hw
new_6_win
run 0 onboard controller.py
run 1 onboard sensors_gate.py
sleep 1
run 2 plugins manual_plugin.py
run 3 plugins depth_hold_plugin.py
run 4 plugins att_hold_plugin.py

#only hw from here
if [ ! -v SIM ]
then 
tmux new-window
new_6_win
run 0 hw hw_gate.py
run 1 hw flircam_proxy.py

tmux att
fi
