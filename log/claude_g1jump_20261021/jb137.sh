#!/bin/bash
# 左右交互の片足跳び（jb137、§55）: 2 回目のしゃがみで HOP_CROUCH_DY と HOP_ALT_LEAN_Y が重なり 4 cm 内へ寄って崩れていた。HOP_CROUCH_DY 0、HOP_ALT_W 0.18、HOP_ALT_LEAN_Y 0／0.01 × HOP_ALT_VY 0.15／0.22（HAL0V15_* など）。2 つの組、5 回跳び、初速 1.5、N=16、j103。
D=/home/takara/work/dp/articara/log/claude_g1jump_20261021; B=$D/g1_flight_probe_j103
eval "$(sed -n '/^X=/p;/^H=/p;/^HP=/p' $D/jb112.sh)"
cd $D
hr() { # TAG SEED LEG EXTRA
  TAG=$1 SEED=$2 BIN=$B CONDS=1.5 N=16 W=2 EXTRA="{$X$H$HP,\"HOP_LEG\":\"$3\",\"JUMP_REPEAT\":\"4\",\"T\":\"20\",\"HOP_MOM_LAND_W\":\"0.1\",\"HOP_CROUCH_DY\":\"0\",\"FLIGHT_FEET_YAW\":\"1\",\"HOP_FEET_YAW_FROM\":\"2\",\"FLIGHT_TRUNK_YAW\":\"1\",\"JUMP_VX\":\"0.09\",\"HOP_ALT\":\"1\",\"HOP_FLIGHT_Y_CAPTURE\":\"1.0\"$4}" python3 jrand.py
}
for l in 0 1; do for v in 15 22; do
hr HAL${l}V${v}_5L 27182 L ",\"HOP_ALT_W\":\"0.18\",\"HOP_ALT_LEAN_Y\":\"0.0$l\",\"HOP_ALT_VY\":\"0.$v\"" &
hr HAL${l}V${v}_5Rn 31415 R ",\"HOP_ALT_W\":\"0.18\",\"HOP_ALT_LEAN_Y\":\"0.0$l\",\"HOP_ALT_VY\":\"0.$v\"" &
done; done
wait
echo "#### JB137_DONE"
