#!/bin/bash
#rsync -avzu -e "ssh -p 2222" --exclude="*.AppImage*" --exclude="*.mp4" --exclude="*.pyc" --exclude=".git/" . stereo@localhost:/home/stereo/bluerov/
#rsync -avzu -e "ssh -p 2222"  --exclude=".git" --include="*/" --include="*.c" --include="*.sh" --include="*.py" --include="*.ino" --exclude="*" $HOME/RovVision2/ stereo@localhost:/home/stereo/RovVision2/
rsync -avzu --exclude=".git" --include="*/" --include="*.c" --include="*.sh" --include="*.py" --include="*.ino" --exclude="*" $HOME/RovVision2/ stereo@stereo.local:/home/stereo/RovVision2/
sleep 2
