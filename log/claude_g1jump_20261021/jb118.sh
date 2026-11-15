#!/bin/bash
# 連続の片足跳び（jb118、§52）: 5 回跳び（HP＋JUMP_REPEAT 4、T 20）の押しが残すヨーの角運動量の扱い。HY（＋FLIGHT_FEET_YAW）に WBCW_MOMENTUM_PUSH 3（HYM5*）、FLIGHT_TRUNK_YAW 1（HYT5*）、STANCE_TRUNK_YAW 10（HYS5*）、HR に WBCW_MOMENTUM_PUSH 3（HM5*）。初速 1.5、種 27182 の左右、N=16、j92。
D=/home/takara/work/dp/articara/log/claude_g1jump_20261021; B=$D/g1_flight_probe_j92
eval "$(sed -n '/^X=/p;/^H=/p;/^HP=/p' $D/jb112.sh)"
cd $D
hr() { # TAG SEED LEG EXTRA
  TAG=$1 SEED=$2 BIN=$B CONDS=1.5 N=16 W=2 EXTRA="{$X$H$HP,\"HOP_LEG\":\"$3\",\"JUMP_REPEAT\":\"4\",\"T\":\"20\"$4}" python3 jrand.py
}
Y=',"FLIGHT_FEET_YAW":"1"'
for L in L R; do
hr HYM5$L 27182 $L "$Y"',"WBCW_MOMENTUM_PUSH":"3"' &
hr HYT5$L 27182 $L "$Y"',"FLIGHT_TRUNK_YAW":"1"' &
hr HYS5$L 27182 $L "$Y"',"STANCE_TRUNK_YAW":"10"' &
hr HM5$L 27182 $L ',"WBCW_MOMENTUM_PUSH":"3"' &
done
wait
echo "#### JB118_DONE"
