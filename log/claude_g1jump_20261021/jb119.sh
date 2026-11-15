#!/bin/bash
# 連続の片足跳び（jb119、§52）: HYT（HP＋JUMP_REPEAT 4＋FLIGHT_FEET_YAW＋FLIGHT_TRUNK_YAW 1）の空中の骨盤のヨーの重みを 3（HYT3_5*）・10（HYT10_5*）、種 16180 の左で HYT（HYT5Lh）と HR（HR5Lh）。初速 1.5、N=16、j92。
D=/home/takara/work/dp/articara/log/claude_g1jump_20261021; B=$D/g1_flight_probe_j92
eval "$(sed -n '/^X=/p;/^H=/p;/^HP=/p' $D/jb112.sh)"
cd $D
hr() { # TAG SEED LEG EXTRA
  TAG=$1 SEED=$2 BIN=$B CONDS=1.5 N=16 W=2 EXTRA="{$X$H$HP,\"HOP_LEG\":\"$3\",\"JUMP_REPEAT\":\"4\",\"T\":\"20\"$4}" python3 jrand.py
}
Y=',"FLIGHT_FEET_YAW":"1"'
for L in L R; do
hr HYT3_5$L 27182 $L "$Y"',"FLIGHT_TRUNK_YAW":"3"' &
hr HYT10_5$L 27182 $L "$Y"',"FLIGHT_TRUNK_YAW":"10"' &
done
hr HYT5Lh 16180 L "$Y"',"FLIGHT_TRUNK_YAW":"1"' &
hr HR5Lh 16180 L '' &
wait
echo "#### JB119_DONE"
