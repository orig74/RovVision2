#!/bin/bash
read -p "Enter UC username: " UCusername
rsync -avz --chmod=Du=rwx,Dg=rx,Do=rx,Fu=rw,Fg=r,Fo=r -P --remove-source-files ~/data/* $UCusername@cssecs7.canterbury.ac.nz:/csse/research/CVlab/bluerov_data
echo "Deleting empty source directories..."
find ~/data/* -type d -empty -delete
read -p "Press ENTER to exit..." _
# /media/uav/Expansion/data/*
