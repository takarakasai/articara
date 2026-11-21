#!/bin/bash
# 止まらない交互の片足跳びの基準の確認（jb142、§57 の続き）: FV20（jb141）＋HOP_RUN_STOP_DY 0.01（最後の着地だけ基準を遊脚側へ、FS01_*）を 5 つの組で、新しい 3 つの組では FV20 も。j106。10 回跳び、初速 1.5、N=16。
D=/home/takara/work/dp/articara/log/claude_g1jump_20261021; B=$D/g1_flight_probe_j106
eval "$(sed -n '/^X=/p;/^H=/p;/^HP=/p' $D/jb112.sh)"
cd $D
hr() { # TAG SEED LEG EXTRA
  TAG=$1 SEED=$2 BIN=$B CONDS=1.5 N=16 W=2 EXTRA="{$X$H$HP,\"HOP_LEG\":\"$3\",\"JUMP_REPEAT\":\"9\",\"T\":\"20\",\"HOP_MOM_LAND_W\":\"0.1\",\"HOP_CROUCH_DY\":\"0\",\"FLIGHT_FEET_YAW\":\"1\",\"HOP_FEET_YAW_FROM\":\"2\",\"FLIGHT_TRUNK_YAW\":\"1\",\"JUMP_VX\":\"0.2\",\"HOP_ALT\":\"1\",\"HOP_FLIGHT_Y_CAPTURE\":\"1.0\",\"HOP_ALT_W\":\"0.18\",\"HOP_ALT_LEAN_Y\":\"0\",\"HOP_ALT_VY\":\"0.15\",\"HOP_RUN\":\"1\",\"HOP_RUN_T_CROUCH\":\"0.25\",\"HOP_RUN_XI\":\"0.07\",\"HOP_RUN_DCM\":\"0\",\"HOP_RUN_CAPTURE_X\":\"0.8\",\"FLIGHT_TRUNK\":\"1\",\"WBCW_TRUNK_PUSH\":\"0.3\",\"HOP_FREE_X_W\":\"0.3\"$4}" python3 jrand.py
}
export -f hr; export D B X H HP
{
  for s in "L 27182 L" "R 27182 R" "Lh 16180 L" "Ln 31415 L" "Rn 31415 R"; do echo "FS01_${s%% *} ${s#* } FS01"; done
  for s in "R 27182 R" "Lh 16180 L" "Ln 31415 L"; do echo "FV20_${s%% *} ${s#* } FV20"; done
} | xargs -P 8 -L 1 bash -c '
case $3 in
  FS01) hr "$0" "$1" "$2" ",\"HOP_RUN_STOP_DY\":\"0.01\"" ;;
  FV20) hr "$0" "$1" "$2" "" ;;
esac'
echo "#### JB142_DONE"
