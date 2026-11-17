#!/bin/bash
# 連続の片足跳び（jb134）: 初速 2.0 の後ろへのずれ。HCDF＋JUMP_VX 0.09／0.12（HVX09t_*・HVX12t_*）を 2 つの組で、5 回跳び、N=16、j100。比べるのは jb131 の HCDFx_*（CONDS 1.0,2.0）。
D=/home/takara/work/dp/articara/log/claude_g1jump_20261021; B=$D/g1_flight_probe_j100
eval "$(sed -n '/^X=/p;/^H=/p;/^HP=/p' $D/jb112.sh)"
cd $D
hr() { # TAG SEED LEG EXTRA
  TAG=$1 SEED=$2 BIN=$B CONDS=2.0 N=16 W=2 EXTRA="{$X$H$HP,\"HOP_LEG\":\"$3\",\"JUMP_REPEAT\":\"4\",\"T\":\"20\",\"HOP_MOM_LAND_W\":\"0.1\",\"HOP_CROUCH_DY\":\"0.02\",\"FLIGHT_FEET_YAW\":\"1\",\"HOP_FEET_YAW_FROM\":\"2\",\"FLIGHT_TRUNK_YAW\":\"1\"$4}" python3 jrand.py
}
for v in 09 12; do
hr HVX${v}t_5L 27182 L ",\"JUMP_VX\":\"0.$v\"" &
hr HVX${v}t_5Rn 31415 R ",\"JUMP_VX\":\"0.$v\"" &
done
wait
echo "#### JB134_DONE"
