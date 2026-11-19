#!/bin/bash
# 左右交互の片足跳び（jb136、§55）: 足の幅を詰めて横の移動を減らす。HVX＋HOP_ALT＋LEAN_Y 0.02＋横の捕獲 1.0、HOP_ALT_W 0.14／0.18（最初の持ち上げにも、j103）× HOP_ALT_VY 0.15／0.22（HAW14V15_* など）。2 つの組、5 回跳び、初速 1.5、N=16。
D=/home/takara/work/dp/articara/log/claude_g1jump_20261021; B=$D/g1_flight_probe_j103
eval "$(sed -n '/^X=/p;/^H=/p;/^HP=/p' $D/jb112.sh)"
cd $D
hr() { # TAG SEED LEG EXTRA
  TAG=$1 SEED=$2 BIN=$B CONDS=1.5 N=16 W=2 EXTRA="{$X$H$HP,\"HOP_LEG\":\"$3\",\"JUMP_REPEAT\":\"4\",\"T\":\"20\",\"HOP_MOM_LAND_W\":\"0.1\",\"HOP_CROUCH_DY\":\"0.02\",\"FLIGHT_FEET_YAW\":\"1\",\"HOP_FEET_YAW_FROM\":\"2\",\"FLIGHT_TRUNK_YAW\":\"1\",\"JUMP_VX\":\"0.09\",\"HOP_ALT\":\"1\",\"HOP_ALT_LEAN_Y\":\"0.02\",\"HOP_FLIGHT_Y_CAPTURE\":\"1.0\"$4}" python3 jrand.py
}
for w in 14 18; do for v in 15 22; do
hr HAW${w}V${v}_5L 27182 L ",\"HOP_ALT_W\":\"0.$w\",\"HOP_ALT_VY\":\"0.$v\"" &
hr HAW${w}V${v}_5Rn 31415 R ",\"HOP_ALT_W\":\"0.$w\",\"HOP_ALT_VY\":\"0.$v\"" &
done; done
wait
echo "#### JB136_DONE"
