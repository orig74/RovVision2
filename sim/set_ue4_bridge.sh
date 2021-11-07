#!/bin/bash
#for running docker game
#DRONESIMLAB_PATH=/DroneLab
#for running unral engine
DRONESIMLAB_PATH=/home/user/projects/DroneSimLab
ENTRY_POINT=unreal_proxy
ENTRY_PATH=unreal_proxy/
#PROJECT_PATH=/project_files/Oceantest1/
PROJECT_PATH=/project_files/Ocean3/
UE4PATH=/local/UnrealEngine
python3 $DRONESIMLAB_PATH/UE4PyhtonBridge/set_path.py --entry_point $ENTRY_POINT --entry_path $ENTRY_PATH --project_path $PROJECT_PATH --uepath $UE4PATH
