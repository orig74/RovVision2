function init_docker_image {
tmux send-keys "cd $DRONESIMLAB_PATH/dockers/python3_dev && ./run_image.sh" ENTER
}

function init_pb {
tmux send-keys "source ${PB_ENV:-~/python_venvs/pybullet}/bin/activate" ENTER
}
function new_4_win {
tmux split-window -h
tmux select-pane -t 0
tmux split-window -v
tmux select-pane -t 2
tmux split-window -v
}


function new_6_win {
tmux split-window -h
tmux select-pane -t 0
tmux split-window -v
tmux select-pane -t 2
tmux split-window -v
tmux select-pane -t 0
tmux split-window -v
tmux select-pane -t 3
tmux split-window -v
}


if [ -v SIM ]
then 

function run { #pane number, path, script
tmux select-pane -t $1 
if [ "$SIM" == "PB" ]
then
init_pb
else
init_docker_image
tmux send-keys "bash" ENTER
fi
tmux send-keys "printf '\033]2;%s\033\\' '$3'" ENTER
tmux send-keys "cd $PROJECT_PATH/$2" ENTER
tmux send-keys "export ROV_TYPE=$ROV_TYPE" ENTER
tmux send-keys "$PYTHON $3" ENTER

}

else
function run { #pane number, path, script
tmux select-pane -t $1 
[ ! -z "$RESIZE_VIEWER" ] && tmux send-keys "export RESIZE_VIEWER=$RESIZE_VIEWER" ENTER
tmux send-keys "printf '\033]2;%s\033\\' '$3'" ENTER
tmux send-keys "conda activate pybullet" ENTER
tmux send-keys "cd $PROJECT_PATH/$2" ENTER
tmux send-keys "python $3" ENTER
}

fi
#tmux send-keys "python drone_main.py" ENTER


