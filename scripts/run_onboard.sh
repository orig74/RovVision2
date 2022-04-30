#!/bin/bash
source run_common.sh

if [ ! -v SIM ]
then
tmux kill-session -t dronelab
tmux new-session -d -s dronelab
PROJECT_PATH=../
else
#PROJECT_PATH=/home/host/projects/RovVision2/
PROJECT_PATH="${PROJECT_PATH:-/home/host/projects/RovVision2/}"
#PYTHON=/miniconda/bin/python 
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
run 5 plugins tracker_plugin.py
tmux new-window
new_6_win
run 0 plugins pos_hold_plugin.py
run 1 plugins sonar_hold_plugin.py
run 2 onboard hw_stats.py

#only hw from here
if [ ! -v SIM ]
then 
tmux new-window
new_6_win
run 0 utils detect_usb.py
sleep 3 
run 0 hw hw_gate.py
#run 1 hw flircam_proxy.py
run 1 hw alvium_proxy.py
run 2 hw periph_gate.py
run 3 hw vnav.py
run 4 hw sonar.py
run 5 hw dvl.py

tmux att
fi
