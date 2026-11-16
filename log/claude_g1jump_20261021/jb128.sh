#!/bin/bash
# 連続の片足跳び（jb128、§53 の続き）: HL01＋HOP_CROUCH_DY（再始動のしゃがみと押しの間だけ重心の基準を遊脚側へ）0.01／0.015／0.02（HCD10_*・HCD15_*・HCD20_*）を 5 つの組で。初速 1.5、5 回跳び、N=16、j99。
D=/home/takara/work/dp/articara/log/claude_g1jump_20261021; B=$D/g1_flight_probe_j99
eval "$(sed -n '/^X=/p;/^H=/p;/^HP=/p' $D/jb112.sh)"
cd $D
hr() { # TAG SEED LEG EXTRA
  TAG=$1 SEED=$2 BIN=$B CONDS=1.5 N=16 W=2 EXTRA="{$X$H$HP,\"HOP_LEG\":\"$3\",\"JUMP_REPEAT\":\"4\",\"T\":\"20\",\"HOP_MOM_LAND_W\":\"0.1\"$4}" python3 jrand.py
}
export -f hr; export D B X H HP
# 8 本ずつ（W=2 で 16 並列）
for v in 10:0.01 15:0.015 20:0.02; do
  for s in "5L 27182 L" "5R 27182 R" "5Lh 16180 L" "5Ln 31415 L" "5Rn 31415 R"; do
    set -- $s; echo "HCD${v%%:*}_$1 $2 $3 ${v#*:}"
  done
done | xargs -P 8 -L 1 bash -c 'hr "$0" "$1" "$2" ",\"HOP_CROUCH_DY\":\"$3\""'
echo "#### JB128_DONE"
