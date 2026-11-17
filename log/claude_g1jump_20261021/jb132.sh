#!/bin/bash
# 連続の片足跳び（jb132）: HCDF の後ろへのずれ（離地の vx −0.05 m/s、1 回 3.5 cm）を押しの前への速さで打ち消す。JUMP_VX 0.03／0.06／0.09（HVX03_* など）を 2 つの組で、5 回跳び、初速 1.5、N=16、j100（= base_hcdf）。
D=/home/takara/work/dp/articara/log/claude_g1jump_20261021; B=$D/g1_flight_probe_j100
eval "$(sed -n '/^X=/p;/^H=/p;/^HP=/p' $D/jb112.sh)"
cd $D
hr() { # TAG SEED LEG EXTRA
  TAG=$1 SEED=$2 BIN=$B CONDS=1.5 N=16 W=2 EXTRA="{$X$H$HP,\"HOP_LEG\":\"$3\",\"JUMP_REPEAT\":\"4\",\"T\":\"20\",\"HOP_MOM_LAND_W\":\"0.1\",\"HOP_CROUCH_DY\":\"0.02\",\"FLIGHT_FEET_YAW\":\"1\",\"HOP_FEET_YAW_FROM\":\"2\",\"FLIGHT_TRUNK_YAW\":\"1\"$4}" python3 jrand.py
}
for v in 03 06 09; do
hr HVX${v}_5L 27182 L ",\"JUMP_VX\":\"0.$v\"" &
hr HVX${v}_5Rn 31415 R ",\"JUMP_VX\":\"0.$v\"" &
done
wait
echo "#### JB132_DONE"
