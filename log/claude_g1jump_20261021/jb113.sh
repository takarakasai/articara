#!/bin/bash
# 片足跳び（jb113、§51）: 2.0 の着地後の後ろへの転倒（離地の vx −0.15、h_y −0.24）に押しの前への速さ JUMP_VX 0.1／0.2／0.3 を足す。HP（jb112）、種 27182 の左、N=16、2.0 だけ。
D=/home/takara/work/dp/articara/log/claude_g1jump_20261021; B=$D/g1_flight_probe_j90
eval "$(sed -n '/^X=/p;/^H=/p;/^HP=/p' $D/jb112.sh)"
cd $D
for vx in 0.1 0.2 0.3; do
  TAG=HPLvx${vx#0.} SEED=27182 BIN=$B CONDS=2.0 N=16 W=5 EXTRA="{$X$H$HP,\"HOP_LEG\":\"L\",\"JUMP_VX\":\"$vx\"}" python3 jrand.py &
done
wait
echo "#### JB113_DONE"
