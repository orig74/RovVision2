#!/bin/bash
printf '\033]2;STEREO\033\\'
ssh -p 2222 stereo@localhost -t "cd RovVision2/scripts && ./run_onboard.sh"  

