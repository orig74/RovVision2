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
#run 5 plugins tracker_plugin.py
tmux new-window
new_6_win
#run 0 plugins pos_hold_plugin.py
run 0 plugins pos_hold_dvl_plugin.py
run 1 plugins tracker_main.py
#run 1 plugins sonar_hold_plugin.py
run 2 onboard hw_stats.py

#only hw from here
if [ ! -v SIM ]
then

tmux select-pane -t 3
#tmux send-keys "gst-launch-1.0 v4l2src device=/dev/video4 ! video/x-raw,width=640 ! videoconvert ! videoflip method=rotate-180 ! x264enc tune=zerolatency bitrate=500 key-int-max=30 ! rtph264pay ! udpsink host=169.254.0.2 port=17893" ENTER
#tmux send-keys "gst-launch-1.0 v4l2src device=/dev/video4 ! video/x-raw,width=640 ! videoconvert ! videoflip method=rotate-180 ! x264enc tune=zerolatency bitrate=500 speed-preset=superfast key-int-max=30 ! rtph264pay ! udpsink host=169.254.0.2 port=17893" ENTER
#tmux send-keys "gst-launch-1.0 -v -e udpsrc port=17893 ! videoflip method=vertical-flip ! application/x-rtp, media=video, clock-rate=90000, encoding-name=H264, payload=96 ! rtph264depay ! tee name=t ! queue ! decodebin ! videoconvert ! autovideosink t. ! queue ! h264parse ! qtmux ! filesink location=/home/uav/records_main_cam/$(date '+%y%m%d-%H%M%S').mov sync=false" ENTER

tmux new-window
new_6_win
run 0 utils detect_usb.py
sleep 3
run 0 hw ESP32_gate.py
#run 1 hw flircam_proxy.py
run 1 hw alvium_proxy.py
run 2 hw D405Driver.py
#run 2 hw periph_gate.py
run 3 hw vnav.py
run 4 hw sonar.py
run 5 hw dvl.py

tmux att
fi
