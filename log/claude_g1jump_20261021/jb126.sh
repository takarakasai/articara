#!/bin/bash
# 連続の片足跳び（jb126、§53）: 基準 HL01 に重心の基準を遊脚側へずらす HOP_COM_DY 0.01（HD1_*）・0.02（HD2_*）。2 回目からのしゃがみで重心が足裏の中心より外へ寄り、CoP が外の縁に張り付く（1 回目は 1.4 cm 内）。5 つの組、初速 1.5、5 回跳び、N=16、j97。
D=/home/takara/work/dp/articara/log/claude_g1jump_20261021; B=$D/g1_flight_probe_j97
eval "$(sed -n '/^X=/p;/^H=/p;/^HP=/p' $D/jb112.sh)"
cd $D
hr() { # TAG SEED LEG EXTRA
  TAG=$1 SEED=$2 BIN=$B CONDS=1.5 N=16 W=1 EXTRA="{$X$H$HP,\"HOP_LEG\":\"$3\",\"JUMP_REPEAT\":\"4\",\"T\":\"20\",\"HOP_MOM_LAND_W\":\"0.1\"$4}" python3 jrand.py
}
for s in "5L 27182 L" "5R 27182 R" "5Lh 16180 L" "5Ln 31415 L" "5Rn 31415 R"; do set -- $s
hr HD1_$1 $2 $3 ',"HOP_COM_DY":"0.01"' &
hr HD2_$1 $2 $3 ',"HOP_COM_DY":"0.02"' &
done
wait
echo "#### JB126_DONE"
