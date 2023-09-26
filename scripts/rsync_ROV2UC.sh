#!/bin/bash
read -p "Enter UC username: " UCusername
rsync -avz --chmod=Du=rw,Dg=rw,Do=r,Fu=rw,Fg=rw,Fo=r -P --remove-source-files ~/../../media/data/* $UCusername@cssecs7.canterbury.ac.nz:/csse/research/CVlab/bluerov_data
echo "Deleting empty source directories..."
find ~/../../media/data/* -type d -empty -delete
read -p "Press ENTER to exit..." _
# /media/uav/Expansion/data/*
