#/bin/bash
tmux kill-session -t dronelab

pkill -2 -f "gst-launch-1.0"
docker ps -a |grep python3_dev | awk -- '{ print $1 }' | xargs docker rm -f
