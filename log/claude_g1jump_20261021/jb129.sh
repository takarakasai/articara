#!/bin/bash
# 連続の片足跳び（jb129、§53 の続き）: HCD20（HL01＋HOP_CROUCH_DY 0.02）に JUMP_REPEAT_HEADING（支持足のヨー、HCDH_*）、HF2 のヨーの保持（HCDF_*）、HCD20 の 10 回跳び（HC10_*、HL10_* と比べる）。5 つの組、初速 1.5、N=16、j100。
D=/home/takara/work/dp/articara/log/claude_g1jump_20261021; B=$D/g1_flight_probe_j100
eval "$(sed -n '/^X=/p;/^H=/p;/^HP=/p' $D/jb112.sh)"
cd $D
hr() { # TAG SEED LEG REPEAT T EXTRA
  TAG=$1 SEED=$2 BIN=$B CONDS=1.5 N=16 W=2 EXTRA="{$X$H$HP,\"HOP_LEG\":\"$3\",\"JUMP_REPEAT\":\"$4\",\"T\":\"$5\",\"HOP_MOM_LAND_W\":\"0.1\",\"HOP_CROUCH_DY\":\"0.02\"$6}" python3 jrand.py
}
export -f hr; export D B X H HP
# 8 本ずつ（W=2 で 16 並列）
for v in HC10 HCDH HCDF; do
  for s in "5L 27182 L" "5R 27182 R" "5Lh 16180 L" "5Ln 31415 L" "5Rn 31415 R"; do
    echo "${v}_${s%% *} ${s#* } $v"
  done
done | xargs -P 8 -L 1 bash -c '
case $3 in
  HC10) hr "$0" "$1" "$2" 9 40 "" ;;
  HCDH) hr "$0" "$1" "$2" 4 20 ",\"JUMP_REPEAT_HEADING\":\"1\"" ;;
  HCDF) hr "$0" "$1" "$2" 4 20 ",\"FLIGHT_FEET_YAW\":\"1\",\"HOP_FEET_YAW_FROM\":\"2\",\"FLIGHT_TRUNK_YAW\":\"1\"" ;;
esac'
echo "#### JB129_DONE"
