#!/bin/bash
echo 1
#ssh -t $REMOTE_ROVSIM_PATH \""cd ~/projects/RovVision2/scripts &&  ./kill_sim.sh\""
ssh -t -p 2222 labuser@localhost "cd ~/projects/RovVision2/scripts &&  ./kill_sim.sh"
