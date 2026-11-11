#!/bin/bash
cd /home/takara/work/dp/articara/log/claude_g1jump_20261021
./jb75scr.sh base
./jb75scr.sh clr03 JUMP_BOX_CLEAR=0.03
./jb75scr.sh clr05 JUMP_BOX_CLEAR=0.05
./jb75scr.sh tt04 FLIGHT_FEET_TUCK_T=0.04
./jb75scr.sh tt03 FLIGHT_FEET_TUCK_T=0.03
echo "#### SCR75_DONE"
