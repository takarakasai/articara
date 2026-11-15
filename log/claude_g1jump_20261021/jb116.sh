#!/bin/bash
# 連続の片足跳び（jb116、§52）: 候補 HP（jb112）に JUMP_REPEAT（j91、跳び直しを支持足の接触だけで）。初速 1.5、3 回跳び（T 12、HR3*）と 5 回跳び（T 20、HR5*）、種 27182 の左・右、種 16180 の左、N=16。
D=/home/takara/work/dp/articara/log/claude_g1jump_20261021; B=$D/g1_flight_probe_j91
eval "$(sed -n '/^X=/p;/^H=/p;/^HP=/p' $D/jb112.sh)"
cd $D
hr() { # TAG SEED LEG REPEAT T
  TAG=$1 SEED=$2 BIN=$B CONDS=1.5 N=16 W=3 EXTRA="{$X$H$HP,\"HOP_LEG\":\"$3\",\"JUMP_REPEAT\":\"$4\",\"T\":\"$5\"}" python3 jrand.py
}
hr HR3L 27182 L 2 12 &
hr HR3R 27182 R 2 12 &
hr HR3Lh 16180 L 2 12 &
hr HR5L 27182 L 4 20 &
hr HR5R 27182 R 4 20 &
wait
echo "#### JB116_DONE"
