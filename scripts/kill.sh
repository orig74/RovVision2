#/bin/bash
tmux kill-session -t dronelab
pkill -2 -f "gst-launch-1.0"
