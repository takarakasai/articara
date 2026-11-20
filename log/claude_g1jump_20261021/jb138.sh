#!/bin/bash
# 止まらない左右交互の片足跳び（jb138、§56）: HOP_RUN（j104）。着地 0.04 s 後に再始動、しゃがみ 0.25 s、離地の DCM 0.07、捕獲点の真上へ置く（HOP_RUN_DCM 0）、前後の捕獲 0.8（RUN08_*）／1.0（RUN10_*）。2 つの組、10 回跳び、初速 1.5、N=16。
D=/home/takara/work/dp/articara/log/claude_g1jump_20261021; B=$D/g1_flight_probe_j104
eval "$(sed -n '/^X=/p;/^H=/p;/^HP=/p' $D/jb112.sh)"
cd $D
hr() { # TAG SEED LEG EXTRA
  TAG=$1 SEED=$2 BIN=$B CONDS=1.5 N=16 W=4 EXTRA="{$X$H$HP,\"HOP_LEG\":\"$3\",\"JUMP_REPEAT\":\"9\",\"T\":\"20\",\"HOP_MOM_LAND_W\":\"0.1\",\"HOP_CROUCH_DY\":\"0\",\"FLIGHT_FEET_YAW\":\"1\",\"HOP_FEET_YAW_FROM\":\"2\",\"FLIGHT_TRUNK_YAW\":\"1\",\"JUMP_VX\":\"0.09\",\"HOP_ALT\":\"1\",\"HOP_FLIGHT_Y_CAPTURE\":\"1.0\",\"HOP_ALT_W\":\"0.18\",\"HOP_ALT_LEAN_Y\":\"0\",\"HOP_ALT_VY\":\"0.15\",\"HOP_RUN\":\"1\",\"HOP_RUN_T_CROUCH\":\"0.25\",\"HOP_RUN_XI\":\"0.07\",\"HOP_RUN_DCM\":\"0\"$4}" python3 jrand.py
}
for k in 08 10; do
hr RUN${k}_L 27182 L ",\"HOP_RUN_CAPTURE_X\":\"${k:0:1}.${k:1}\"" &
hr RUN${k}_Rn 31415 R ",\"HOP_RUN_CAPTURE_X\":\"${k:0:1}.${k:1}\"" &
done
wait
echo "#### JB138_DONE"
