#!/bin/bash
#rsync -avzu -e "ssh -p 2222" --exclude="*.AppImage*" --exclude="*.mp4" --exclude="*.pyc" --exclude=".git/" . stereo@localhost:/home/stereo/bluerov/
rsync -avzu -e "ssh"  --exclude=".git" --include="*/" --include="*.sh" --include="*.py" --include="*.ino" --exclude="*" $HOME/RovVision2/ stereo@192.168.2.2:/home/stereo/RovVision2/
sleep 2
