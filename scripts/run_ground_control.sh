#!/bin/bash
source run_common.sh
PROJECT_PATH="${PROJECT_PATH:-/home/host/projects/RovVision2/}"
#PYTHON=${1:-/miniconda/bin/python}

#[ $USER == 'uav' ] && export RESIZE_VIEWER=1500
# [ $USER == 'uav' ] && export RESIZE_VIEWER=2300

if [ ! -v SIM ]
then
tmux kill-session -t dronelab
tmux new-session -d -s dronelab
PYTHON=/bin/python3
PROJECT_PATH=../
else
tmux new-window
fi

new_6_win
run 0 ground_control joy_rov.py
run 1 ground_control "viewer.py --pub_data"
run 2 web "--version && FLASK_APP=server.py flask run"
run 3 ground_control gps_logger.py
#run 3 web "--version && sleep 3 && firefox http://127.0.0.1:5000/static/html/ropedive.html --new-window  --new-tab -url http://127.0.0.1:5000/static/html/checklists.html"
if [ ! -v SIM ]
then
tmux select-pane -t 4
#tmux send-keys "gst-launch-1.0 -v udpsrc port=17893 ! application/x-rtp, media=video, clock-rate=90000, encoding-name=H264, payload=96 ! rtph264depay ! decodebin ! videoconvert ! autovideosink" ENTER
tmux send-keys "gst-launch-1.0 -v -e udpsrc port=17893 ! application/x-rtp, media=video, clock-rate=90000, encoding-name=H264, payload=96 ! rtph264depay ! tee name=t ! queue ! decodebin ! videoconvert ! autovideosink t. ! queue ! h264parse ! qtmux ! filesink location=/home/uav/records_main_cam/$(date '+%y%m%d-%H%M%S').mov sync=false" ENTER
tmux att
fi
