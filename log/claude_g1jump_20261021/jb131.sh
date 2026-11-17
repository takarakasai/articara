#!/bin/bash
# 連続の片足跳び（jb131）: 基準 HCDF と前の基準 HL01 を初速 1.0／2.0 で（HCDFx_*・HL01x_*）。5 回跳び、5 つの組、N=16、j100（= base_hcdf）。
D=/home/takara/work/dp/articara/log/claude_g1jump_20261021; B=$D/g1_flight_probe_j100
eval "$(sed -n '/^X=/p;/^H=/p;/^HP=/p' $D/jb112.sh)"
cd $D
hr() { # TAG SEED LEG EXTRA
  TAG=$1 SEED=$2 BIN=$B CONDS=1.0,2.0 N=16 W=2 EXTRA="{$X$H$HP,\"HOP_LEG\":\"$3\",\"JUMP_REPEAT\":\"4\",\"T\":\"20\",\"HOP_MOM_LAND_W\":\"0.1\"$4}" python3 jrand.py
}
export -f hr; export D B X H HP
for v in HCDFx HL01x; do
  for s in "5L 27182 L" "5R 27182 R" "5Lh 16180 L" "5Ln 31415 L" "5Rn 31415 R"; do
    echo "${v}_${s%% *} ${s#* } $v"
  done
done | xargs -P 8 -L 1 bash -c '
case $3 in
  HCDFx) hr "$0" "$1" "$2" ",\"HOP_CROUCH_DY\":\"0.02\",\"FLIGHT_FEET_YAW\":\"1\",\"HOP_FEET_YAW_FROM\":\"2\",\"FLIGHT_TRUNK_YAW\":\"1\"" ;;
  HL01x) hr "$0" "$1" "$2" "" ;;
esac'
echo "#### JB131_DONE"
