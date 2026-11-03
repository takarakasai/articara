#!/bin/bash
# 2.0 の前出し 6 cm の後ろへの転倒を止まり点の倍率で直せるか（2026-11-03、ユーザ選択「1」、j33）。
# TB4（足 10・胴なし）の 2.0 で FLIGHT_FEET_CAPTURE を 0.40〜0.50 に。前出し 4.5 cm（TB4 そのもの）でも同じく。
cd /home/takara/work/dp/articara
source ./setup-mujoco.sh >/dev/null
D=log/claude_g1jump_20261021; B=$D/g1_flight_probe_j33
X='"JUMP_ABORT":"1","JUMP_PUSH_R":"0.3","QD_CLAMP":"40","DIAG_TD":"1","FLIGHT_FEET_FLAT":"1","DIAG_HX":"1","RELEASE_FIRST_OFF":"1","WBCW_CONTACT_REST":"0.1","WBCW_TRUNK_LAND":"0.03","FALL_TILT":"1.5","LAND_REF_FOLLOW":"2","AIR_ABORT_T":"0.08"'
H=',"FLIGHT_FEET_CAPTURE_T":"0.06","KP_FLIGHT_FEET":"1000","KD_FLIGHT_FEET":"63","WBCW_TRUNK_PUSH":"0.003","WBCW_MOMENTUM":"10","WBCW_CONTACT":"100","WBCW_TRUNK_LAND_VZ":"0","JUMP_H_NAT":"1","WBCW_CONTACT_REST_LAUNCH":"1","JUMP_VX_TAB":"1.0:1.0,1.5:1.2,2.0:1.6","FLIGHT_TRUNK":"0","FLIGHT_FEET_W":"10"'
# 名前 前出し 倍率 種
J=("L6C40 0.06 0.40 31415" "L6C40s2 0.06 0.40 27182"
   "L6C45 0.06 0.45 31415" "L6C45s2 0.06 0.45 27182"
   "L6C50 0.06 0.50 31415" "L6C50s2 0.06 0.50 27182"
   "L45C45 0.045 0.45 31415" "L45C45s2 0.045 0.45 27182"
   "L45C50 0.045 0.50 31415" "L45C50s2 0.045 0.50 27182")
i=0
for j in "${J[@]}"; do set -- $j
  W=$([ $i -lt 6 ] && echo 2 || echo 1); i=$((i+1))
  TAG=$1 SEED=$4 BIN=$B CONDS=2.0 N=48 W=$W EXTRA="{$X$H,\"JUMP_LEAN_TAB\":\"2.0:$2\",\"FLIGHT_FEET_CAPTURE\":\"$3\"}" python3 $D/jrand.py &
done
wait
echo "#### JB51_DONE"
