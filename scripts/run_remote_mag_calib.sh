#!/bin/bash
CMD="cd RovVision2/scripts && source ./set_stereo_conda.sh && cd ../hw && python vnav.py --calib_mag"
ssh -t -p 2222 stereo@localhost $CMD 

