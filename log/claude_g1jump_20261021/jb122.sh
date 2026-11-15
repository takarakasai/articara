#!/bin/bash
# 連続の片足跳び（jb122、§52）: 片足の着地から 2 s だけ角運動量のタスクの重みを下げる HOP_MOM_LAND_W 1／0.1。HR（HP＋JUMP_REPEAT 4）に（HL1_5*・HL01_5*）、HYT（＋FLIGHT_FEET_YAW＋FLIGHT_TRUNK_YAW 1）に（HYL1_5*・HYL01_5*）。初速 1.5、種 27182 の左右と種 16180 の左（*Lh）、N=16、j96。
D=/home/takara/work/dp/articara/log/claude_g1jump_20261021; B=$D/g1_flight_probe_j96
eval "$(sed -n '/^X=/p;/^H=/p;/^HP=/p' $D/jb112.sh)"
cd $D
hr() { # TAG SEED LEG EXTRA
  TAG=$1 SEED=$2 BIN=$B CONDS=1.5 N=16 W=1 EXTRA="{$X$H$HP,\"HOP_LEG\":\"$3\",\"JUMP_REPEAT\":\"4\",\"T\":\"20\"$4}" python3 jrand.py
}
Y=',"FLIGHT_FEET_YAW":"1","FLIGHT_TRUNK_YAW":"1"'
for s in "5L 27182 L" "5R 27182 R" "5Lh 16180 L"; do set -- $s
hr HL1_$1 $2 $3 ',"HOP_MOM_LAND_W":"1"' &
hr HL01_$1 $2 $3 ',"HOP_MOM_LAND_W":"0.1"' &
hr HYL1_$1 $2 $3 "$Y"',"HOP_MOM_LAND_W":"1"' &
hr HYL01_$1 $2 $3 "$Y"',"HOP_MOM_LAND_W":"0.1"' &
done
wait
echo "#### JB122_DONE"
