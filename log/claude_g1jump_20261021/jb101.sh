#!/bin/bash
# 横への跳躍の下見（jb101、§49）: 1 回跳び、その場（JUMP_VX/LEAN の表を 0）の上に JUMP_VY（離地の横の速さ）と JUMP_LEAN_Y（しゃがみの間の横への寄せ）、j85。種 31415 N=16、T 6。
D=/home/takara/work/dp/articara/log/claude_g1jump_20261021; B=$D/g1_flight_probe_j85
X='"JUMP_ABORT":"1","JUMP_PUSH_R":"0.3","QD_CLAMP":"40","DIAG_TD":"1","FLIGHT_FEET_FLAT":"1","DIAG_HX":"1","RELEASE_FIRST_OFF":"1","WBCW_CONTACT_REST":"0.1","WBCW_TRUNK_LAND":"0.03","FALL_TILT":"1.5","LAND_REF_FOLLOW":"2","AIR_ABORT_T":"0.08"'
H=',"FLIGHT_FEET_CAPTURE_T":"0.06","KP_FLIGHT_FEET":"1000","KD_FLIGHT_FEET":"63","WBCW_TRUNK_PUSH":"0.003","WBCW_MOMENTUM":"10","WBCW_CONTACT":"100","WBCW_TRUNK_LAND_VZ":"0","JUMP_H_NAT":"1","WBCW_CONTACT_REST_LAUNCH":"1","FLIGHT_FEET_CAPTURE":"0.55","FLIGHT_TRUNK":"0","FLIGHT_FEET_W":"10","WBCW_MOMENTUM_PUSH":"1","MU_TORSION":"0.02","TRUNK_RP_YAWED":"1"'
Z=',"JUMP_VX_TAB":"1.0:0,2.0:0","JUMP_LEAN_TAB":"1.0:0,2.0:0","T":"6","FLIGHT_FEET_YAW":"1","FLIGHT_FEET_WIDTH":"0.236"'
cd $D
lat() { TAG=$1 SEED=31415 BIN=$B CONDS=1.5,2.0 N=16 W=1 EXTRA="{$X$H$Z$2}" python3 jrand.py; }
q() { while [ $(jobs -rp | wc -l) -ge 16 ]; do wait -n; done; }
q; lat LY0 '' &
for vy in 20 40 60; do for ly in 0 2; do
  q; lat LY${vy}_$ly ",\"JUMP_VY\":\"0.$vy\",\"JUMP_LEAN_Y\":\"0.0$ly\"" &
done; done
wait
echo "#### JB101_DONE"
