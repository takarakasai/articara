#!/bin/bash
# 片足跳び（jb114、§51）: 2.0 の後ろへの転倒は空中で胴が −0.26 → −0.68 rad 後ろへ回る（離地の h_y −0.24〜−0.38）。空中の胴（FLIGHT_TRUNK 1）、押しの間の角運動量の重み（WBCW_MOMENTUM_PUSH 10）、押しの胴（WBCW_TRUNK_PUSH 0.03）。HP、種 27182 の左、N=16、2.0 だけ。
D=/home/takara/work/dp/articara/log/claude_g1jump_20261021; B=$D/g1_flight_probe_j90
eval "$(sed -n '/^X=/p;/^H=/p;/^HP=/p' $D/jb112.sh)"
cd $D
r() { TAG=$1 SEED=27182 BIN=$B CONDS=2.0 N=16 W=5 EXTRA="{$X$H$HP,\"HOP_LEG\":\"L\"$2}" python3 jrand.py; }
r HPLft ',"FLIGHT_TRUNK":"1"' &
r HPLmp ',"WBCW_MOMENTUM_PUSH":"10"' &
r HPLtp ',"WBCW_TRUNK_PUSH":"0.03"' &
wait
echo "#### JB114_DONE"
