#!/bin/bash
# 1.0 の着地の時間差（振り出した足が 7 cm 上に残る）: FLIGHT_FEET_LEVEL=1（離地で高さをそろえる、旧）と =2（滑らかに下ろす）を R10W（3 回跳び）と比べる。j42。
D=/home/takara/work/dp/articara/log/claude_g1jump_20261021; B=$D/g1_flight_probe_j42
X='"JUMP_ABORT":"1","JUMP_PUSH_R":"0.3","QD_CLAMP":"40","DIAG_TD":"1","FLIGHT_FEET_FLAT":"1","DIAG_HX":"1","RELEASE_FIRST_OFF":"1","WBCW_CONTACT_REST":"0.1","WBCW_TRUNK_LAND":"0.03","FALL_TILT":"1.5","LAND_REF_FOLLOW":"2","AIR_ABORT_T":"0.08"'
H=',"FLIGHT_FEET_CAPTURE_T":"0.06","KP_FLIGHT_FEET":"1000","KD_FLIGHT_FEET":"63","WBCW_TRUNK_PUSH":"0.003","WBCW_MOMENTUM":"10","WBCW_CONTACT":"100","WBCW_TRUNK_LAND_VZ":"0","JUMP_H_NAT":"1","WBCW_CONTACT_REST_LAUNCH":"1","FLIGHT_FEET_CAPTURE":"0.55","FLIGHT_TRUNK":"0","FLIGHT_FEET_W":"10","WBCW_MOMENTUM_PUSH":"1"'
RW=',"T":"13","JUMP_REPEAT":"2","FLIGHT_FEET_YAW":"1","WBCW_TRUNK_LAND_T":"1.0","JUMP_REPEAT_SOFT_CROUCH":"1","JUMP_REPEAT_RECENTRE":"1","FLIGHT_FEET_WIDTH":"0.236"'
VX='"JUMP_VX_TAB":"1.0:1.0,1.5:1.2,2.0:1.6"'; LN='"JUMP_LEAN_TAB":"1.0:0,1.5:0.045,2.0:0.06"'
cd $D
run() { TAG=$1 SEED=31415 BIN=$B CONDS=1.0 N=48 W=8 EXTRA="{$X$H$RW,$2}" python3 jrand.py; }
run P2F1 "$VX,$LN"',"FLIGHT_FEET_LEVEL":"1"' &
run P2F2 "$VX,$LN"',"FLIGHT_FEET_LEVEL":"2"' &
wait
python3 rsumm.py R10W P2F1 P2F2 | grep v1.0
for t in R10W P2F1 P2F2; do echo "== $t"; python3 rearmfx.py v1.0 $t | head -2; done
echo "#### JB63_DONE"
