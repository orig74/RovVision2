#!/bin/bash
LOCALS=""
REMOTES=""

LP="7788 8897 9302 9301 7789 6760 6761"
RP="9303 8899"

for i in $LP;do
  LOCALS="$LOCALS -L $i:127.0.0.1:$i"
done
for i in $RP;do
  REMOTES="$REMOTES -R $i:127.0.0.1:$i"
done

PANDA_ADDR=user@10.0.0.1

#echo $LOCALS
CMD="ssh -t -N $LOCALS $REMOTES $RASPPI_IP"
echo $CMD
$CMD
