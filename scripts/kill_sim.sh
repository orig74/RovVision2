#/bin/bash
tmux kill-session -t dronelab
docker ps -a |grep python3_dev | awk -- '{ print $1 }' | xargs docker rm -f
