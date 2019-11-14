#!/bin/bash
CMD="cd RovVision2/scripts && ./set_stereo_conda.sh && cd ../hw && python vnav.py --reset"
ssh -t -p 2222 stereo@localhost $CMD 

