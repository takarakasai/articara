#!/bin/bash
# 左右交互の片足跳び（jb135、§55）: HVX＋HOP_ALT＋HOP_ALT_VY 0.3＋HOP_ALT_LEAN_Y 0.02＋HOP_ALT_W 0.22、横の捕獲 HOP_FLIGHT_Y_CAPTURE 0.95／1.0／1.1（HA095_* など）。2 つの組、5 回跳び、初速 1.5、N=16、j102。
D=/home/takara/work/dp/articara/log/claude_g1jump_20261021; B=$D/g1_flight_probe_j102
eval "$(sed -n '/^X=/p;/^H=/p;/^HP=/p' $D/jb112.sh)"
cd $D
hr() { # TAG SEED LEG EXTRA
  TAG=$1 SEED=$2 BIN=$B CONDS=1.5 N=16 W=2 EXTRA="{$X$H$HP,\"HOP_LEG\":\"$3\",\"JUMP_REPEAT\":\"4\",\"T\":\"20\",\"HOP_MOM_LAND_W\":\"0.1\",\"HOP_CROUCH_DY\":\"0.02\",\"FLIGHT_FEET_YAW\":\"1\",\"HOP_FEET_YAW_FROM\":\"2\",\"FLIGHT_TRUNK_YAW\":\"1\",\"JUMP_VX\":\"0.09\",\"HOP_ALT\":\"1\",\"HOP_ALT_VY\":\"0.3\",\"HOP_ALT_LEAN_Y\":\"0.02\",\"HOP_ALT_W\":\"0.22\"$4}" python3 jrand.py
}
for c in 095 100 110; do
hr HA${c}_5L 27182 L ",\"HOP_FLIGHT_Y_CAPTURE\":\"${c:0:1}.${c:1}\"" &
hr HA${c}_5Rn 31415 R ",\"HOP_FLIGHT_Y_CAPTURE\":\"${c:0:1}.${c:1}\"" &
done
wait
echo "#### JB135_DONE"
