#!/bin/bash
cd /home/takara/work/dp/articara/log/claude_g1jump_20261021
./jb76scr.sh base
./jb76scr.sh kd200 KD_TRUNK_LAND_BOX=200
./jb76scr.sh kd250 KD_TRUNK_LAND_BOX=250
./jb76scr.sh soles1 LAND_REF_SOLES=1.0
./jb76scr.sh kd200s1 KD_TRUNK_LAND_BOX=200 LAND_REF_SOLES=1.0
./jb76scr.sh rise2 LAND_RISE=2.0
echo "#### SCR76_DONE"
