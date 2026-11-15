#!/bin/bash
# 連続の片足跳び（jb121、§52）: HYT（HP＋JUMP_REPEAT 4＋FLIGHT_FEET_YAW＋FLIGHT_TRUNK_YAW 1）に、地上で最初から骨盤のヨーを支持足のヨーへ寄せる HOP_TRUNK_YAW 1（HT1_5*）・10（HT10_5*）。初速 1.5、種 27182 の左右と種 16180 の左（*Lh）、N=16、j94。
D=/home/takara/work/dp/articara/log/claude_g1jump_20261021; B=$D/g1_flight_probe_j94
eval "$(sed -n '/^X=/p;/^H=/p;/^HP=/p' $D/jb112.sh)"
cd $D
hr() { # TAG SEED LEG EXTRA
  TAG=$1 SEED=$2 BIN=$B CONDS=1.5 N=16 W=2 EXTRA="{$X$H$HP,\"HOP_LEG\":\"$3\",\"JUMP_REPEAT\":\"4\",\"T\":\"20\"$4}" python3 jrand.py
}
Y=',"FLIGHT_FEET_YAW":"1","FLIGHT_TRUNK_YAW":"1"'
for w in 1 10; do
hr HT${w}_5L 27182 L "$Y"',"HOP_TRUNK_YAW":"'$w'"' &
hr HT${w}_5R 27182 R "$Y"',"HOP_TRUNK_YAW":"'$w'"' &
hr HT${w}_5Lh 16180 L "$Y"',"HOP_TRUNK_YAW":"'$w'"' &
done
wait
echo "#### JB121_DONE"
