#!/bin/bash
# 連続の片足跳び（jb123、§52）: 候補 HL01（HR＋HOP_MOM_LAND_W 0.1）を新しい種 31415 の左右で HR と（HR5Ln・HL01_5Ln など）、重み 0.01（HL001_5*）を 3 つの組で。初速 1.5、N=16、j96。
D=/home/takara/work/dp/articara/log/claude_g1jump_20261021; B=$D/g1_flight_probe_j96
eval "$(sed -n '/^X=/p;/^H=/p;/^HP=/p' $D/jb112.sh)"
cd $D
hr() { # TAG SEED LEG EXTRA
  TAG=$1 SEED=$2 BIN=$B CONDS=1.5 N=16 W=2 EXTRA="{$X$H$HP,\"HOP_LEG\":\"$3\",\"JUMP_REPEAT\":\"4\",\"T\":\"20\"$4}" python3 jrand.py
}
hr HR5Ln 31415 L '' &
hr HR5Rn 31415 R '' &
hr HL01_5Ln 31415 L ',"HOP_MOM_LAND_W":"0.1"' &
hr HL01_5Rn 31415 R ',"HOP_MOM_LAND_W":"0.1"' &
hr HL001_5L 27182 L ',"HOP_MOM_LAND_W":"0.01"' &
hr HL001_5R 27182 R ',"HOP_MOM_LAND_W":"0.01"' &
hr HL001_5Lh 16180 L ',"HOP_MOM_LAND_W":"0.01"' &
wait
echo "#### JB123_DONE"
