#!/bin/bash
# 連続の片足跳び（jb133）: HCDF＋JUMP_VX 0.09（後ろへのずれを打ち消す、jb132 で 1 回 −3.55 cm → −0.08 mm）の 10 回跳び（HVX10_*、HF10C_* と比べる）。5 つの組、初速 1.5、N=16、W=3、j100。
D=/home/takara/work/dp/articara/log/claude_g1jump_20261021; B=$D/g1_flight_probe_j100
eval "$(sed -n '/^X=/p;/^H=/p;/^HP=/p' $D/jb112.sh)"
cd $D
hr() { # TAG SEED LEG EXTRA
  TAG=$1 SEED=$2 BIN=$B CONDS=1.5 N=16 W=3 EXTRA="{$X$H$HP,\"HOP_LEG\":\"$3\",\"JUMP_REPEAT\":\"9\",\"T\":\"40\",\"HOP_MOM_LAND_W\":\"0.1\",\"HOP_CROUCH_DY\":\"0.02\",\"FLIGHT_FEET_YAW\":\"1\",\"HOP_FEET_YAW_FROM\":\"2\",\"FLIGHT_TRUNK_YAW\":\"1\"$4}" python3 jrand.py
}
for s in "5L 27182 L" "5R 27182 R" "5Lh 16180 L" "5Ln 31415 L" "5Rn 31415 R"; do set -- $s
hr HVX10_$1 $2 $3 ",\"JUMP_VX\":\"0.09\"" &
done
wait
echo "#### JB133_DONE"
