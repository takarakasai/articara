#!/bin/bash
# 連続の片足跳び（jb124、§53）: 基準 HL01 に、2 回目の跳躍からだけ空中の足（と骨盤）のヨーを保つ HOP_FEET_YAW_FROM 2。足と骨盤（HF2_*、FLIGHT_FEET_YAW＋FLIGHT_TRUNK_YAW 1）、足だけ（HF2f_*）。5 つの組（27182 左右・16180 左・31415 左右）、初速 1.5、N=16、j97。
D=/home/takara/work/dp/articara/log/claude_g1jump_20261021; B=$D/g1_flight_probe_j97
eval "$(sed -n '/^X=/p;/^H=/p;/^HP=/p' $D/jb112.sh)"
cd $D
hr() { # TAG SEED LEG EXTRA
  TAG=$1 SEED=$2 BIN=$B CONDS=1.5 N=16 W=1 EXTRA="{$X$H$HP,\"HOP_LEG\":\"$3\",\"JUMP_REPEAT\":\"4\",\"T\":\"20\",\"HOP_MOM_LAND_W\":\"0.1\"$4}" python3 jrand.py
}
F=',"FLIGHT_FEET_YAW":"1","HOP_FEET_YAW_FROM":"2"'
for s in "5L 27182 L" "5R 27182 R" "5Lh 16180 L" "5Ln 31415 L" "5Rn 31415 R"; do set -- $s
hr HF2_$1 $2 $3 "$F"',"FLIGHT_TRUNK_YAW":"1"' &
hr HF2f_$1 $2 $3 "$F" &
done
wait
echo "#### JB124_DONE"
