#!/bin/bash
source run_common.sh
PROJECT_PATH=/home/host/projects/RovVision2/
PYTHON=/miniconda/bin/python 

#[ $USER == 'uav' ] && export RESIZE_VIEWER=1500
[ $USER == 'uav' ] && export RESIZE_VIEWER=2300

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
run 2 web "--version && FLASK_APP=server.py flask run"
run 3 web "--version && sleep 3 && firefox http://127.0.0.1:5000/static/html/checklists.html --new-window"
if [ ! -v SIM ]
then 
tmux att
fi
