#!/bin/bash
# 止まらない左右交互の片足跳びの空中の胴（jb139、§57）: RUN08（jb138）＋FLIGHT_TRUNK 1（FT08_*）／＋WBCW_TRUNK_PUSH 0.3（TF08_*）。j105（HOP_FREE_X_W は未設定）。2 つの組、10 回跳び、初速 1.5、N=16。
D=/home/takara/work/dp/articara/log/claude_g1jump_20261021; B=$D/g1_flight_probe_j105
eval "$(sed -n '/^X=/p;/^H=/p;/^HP=/p' $D/jb112.sh)"
cd $D
hr() { # TAG SEED LEG EXTRA
  TAG=$1 SEED=$2 BIN=$B CONDS=1.5 N=16 W=4 EXTRA="{$X$H$HP,\"HOP_LEG\":\"$3\",\"JUMP_REPEAT\":\"9\",\"T\":\"20\",\"HOP_MOM_LAND_W\":\"0.1\",\"HOP_CROUCH_DY\":\"0\",\"FLIGHT_FEET_YAW\":\"1\",\"HOP_FEET_YAW_FROM\":\"2\",\"FLIGHT_TRUNK_YAW\":\"1\",\"JUMP_VX\":\"0.09\",\"HOP_ALT\":\"1\",\"HOP_FLIGHT_Y_CAPTURE\":\"1.0\",\"HOP_ALT_W\":\"0.18\",\"HOP_ALT_LEAN_Y\":\"0\",\"HOP_ALT_VY\":\"0.15\",\"HOP_RUN\":\"1\",\"HOP_RUN_T_CROUCH\":\"0.25\",\"HOP_RUN_XI\":\"0.07\",\"HOP_RUN_DCM\":\"0\"$4}" python3 jrand.py
}
C=",\"HOP_RUN_CAPTURE_X\":\"0.8\",\"FLIGHT_TRUNK\":\"1\""
hr FT08_L 27182 L "$C" &
hr FT08_Rn 31415 R "$C" &
hr TF08_L 27182 L "$C,\"WBCW_TRUNK_PUSH\":\"0.3\"" &
hr TF08_Rn 31415 R "$C,\"WBCW_TRUNK_PUSH\":\"0.3\"" &
wait
echo "#### JB139_DONE"
