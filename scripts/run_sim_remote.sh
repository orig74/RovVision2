#!/bin/bash
echo 1
#echo usage:
#echo REMOTE_ROVSIM_PATH="username@remote_computer" ./run_sim_remote.sh
#echo or
#echo REMOTE_ROVSIM_PATH="username@remote_computer ssh -t username2@labcomputer" ./run_sim_remote.sh
ssh -t -p 2222 labuser@localhost "cd ~/projects/RovVision2/scripts &&  ./kill_sim.sh ; DISPLAY=:0 HEADLESS= ./run_sim.sh"
