#!/bin/bash
echo 1
ssh -t $REMOTE_ROVSIM_PATH \""cd ~/projects/RovVision2/scripts &&  ./kill_sim.sh\""
