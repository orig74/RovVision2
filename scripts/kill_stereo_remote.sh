#!/bin/bash
printf '\033]2;STEREO\033\\'
ssh stereo@stereo.local -t "cd RovVision2/scripts && ./kill.sh"  

