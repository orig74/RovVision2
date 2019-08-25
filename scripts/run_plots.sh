#!/bin/bash
PROJECT_PATH='../'
source run_common.sh
tmux kill-session -t rovplots
tmux new-session -d -s rovplots
new_6_win

function run { #pane number, path, script
tmux select-pane -t $1 
tmux send-keys "cd $PROJECT_PATH/$2" ENTER
tmux send-keys "$3" ENTER
}

run 0 ground_control "python plotter.py --topic topic_att_pitch_control --port 10052"
run 1 ground_control "python plotter.py --topic topic_att_roll_control --port 10052"
run 2 ground_control "python plotter.py --topic topic_att_yaw_control --port 10052"
run 3 ground_control "python plotter.py --topic topic_pos_hold_pid_0 --port 10053"
run 4 ground_control "python plotter.py --topic topic_pos_hold_pid_1 --port 10053"
run 5 ground_control "python plotter.py --topic topic_depth_control --port 9996"

tmux att



