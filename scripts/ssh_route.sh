#!/bin/bash
LOCALS=""
REMOTES=""

## 6760 6761 - gstreamer , 5577 - for depth image
LP="17790 17789 7755 7787 7788 8890 8897 9302 9301 9996 9998 10101 10111 10102 10103 6760 6761 17894 17895 5577 10052 10053 13295"
RP="9303 9304 8899 13297 7445"

for i in $LP;do
  LOCALS="$LOCALS -L $i:127.0.0.1:$i"
done
for i in $RP;do
  REMOTES="$REMOTES -R $i:127.0.0.1:$i"
done

#PANDA_ADDR=stereo@192.168.2.2
PANDA_ADDR=stereo@stereo.local

if [[ $# -eq 0 ]] ; then
    CMD="ssh -t -N -L 2222:localhost:22 $LOCALS $REMOTES $PANDA_ADDR"
fi
if [[ $# -eq 2 ]] ; then
    CMD="ssh -t -L 2222:localhost:2222 $LOCALS $REMOTES $1 ssh -t -N -L 2222:localhost:22 $LOCALS $REMOTES $2"
fi
#if [[ $# -eq 2 ]] ; then
#    CMD="ssh -t $LOCALS $REMOTES $1 ssh -t $LOCALS $REMOTES $2 ssh -t -N $LOCALS $REMOTES $PANDA_ADDR"
#fi

#echo $LOCALS
echo $CMD
$CMD
