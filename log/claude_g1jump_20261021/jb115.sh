#!/bin/bash
# 片足跳び（jb115、§51）: 2.0 の後ろへの転倒に、遊脚の重み（HOP_FOOT_W 0.1／0.3、押しで遊脚が 13 cm 前へ振れる）と着地後の胴の重み（WBCW_TRUNK_LAND 0.03 = X のまま）。HP、種 27182 の左、N=16、2.0 だけ。
D=/home/takara/work/dp/articara/log/claude_g1jump_20261021; B=$D/g1_flight_probe_j90
eval "$(sed -n '/^X=/p;/^H=/p;/^HP=/p' $D/jb112.sh)"
cd $D
r() { TAG=$1 SEED=27182 BIN=$B CONDS=2.0 N=16 W=5 EXTRA="{$X$H$HP,\"HOP_LEG\":\"L\"$2}" python3 jrand.py; }
r HPLfw1 ',"HOP_FOOT_W":"0.1"' &
r HPLfw3 ',"HOP_FOOT_W":"0.3"' &
r HPLtl ',"WBCW_TRUNK_LAND":"0.03"' &
wait
echo "#### JB115_DONE"
