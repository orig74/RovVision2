#!/bin/bash
printf '\033]2;STEREO\033\\'
ssh stereo@192.168.2.2 -t "cd RovVision2/scripts && ./kill.sh"  

