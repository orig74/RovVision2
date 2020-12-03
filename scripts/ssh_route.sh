#!/bin/bash
LOCALS=""
REMOTES=""

## 6760 6761 - gstreamer , 5577 - for depth image
LP="7755 7787 7788 8897 9302 9301 9996 9998 10101 10102 10103 10104 6760 6761 5577 10052 10053"
RP="9303 8899"

for i in $LP;do
  LOCALS="$LOCALS -L $i:127.0.0.1:$i"
done
for i in $RP;do
  REMOTES="$REMOTES -R $i:127.0.0.1:$i"
done

PANDA_ADDR=stereo@192.168.2.2

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
