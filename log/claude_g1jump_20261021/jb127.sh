#!/bin/bash
# 連続の片足跳び（jb127、§53）: 基準 HL01 に、片足の地上で支持足の CoP を足裏の中心へ戻すように重心の基準を動かす HOP_COP_CENTRE 1（HC1_*）・3（HC3_*）。HOP_COM_DY 0.01 は軽い本を救い重い本を内へ倒した（質量で要るずらしが違う）。5 つの組、初速 1.5、5 回跳び、N=16、j98。
D=/home/takara/work/dp/articara/log/claude_g1jump_20261021; B=$D/g1_flight_probe_j98
eval "$(sed -n '/^X=/p;/^H=/p;/^HP=/p' $D/jb112.sh)"
cd $D
hr() { # TAG SEED LEG EXTRA
  TAG=$1 SEED=$2 BIN=$B CONDS=1.5 N=16 W=1 EXTRA="{$X$H$HP,\"HOP_LEG\":\"$3\",\"JUMP_REPEAT\":\"4\",\"T\":\"20\",\"HOP_MOM_LAND_W\":\"0.1\"$4}" python3 jrand.py
}
for s in "5L 27182 L" "5R 27182 R" "5Lh 16180 L" "5Ln 31415 L" "5Rn 31415 R"; do set -- $s
hr HC1_$1 $2 $3 ',"HOP_COP_CENTRE":"1"' &
hr HC3_$1 $2 $3 ',"HOP_COP_CENTRE":"3"' &
done
wait
echo "#### JB127_DONE"
