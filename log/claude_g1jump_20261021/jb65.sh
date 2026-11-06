#!/bin/bash
# 1.0 の押しの間の後ろへのピッチ（§25）: 押しの間の胴の重み WBCW_TRUNK_PUSH 0.03／0.1 と、しゃがみを浅く（CROUCH 0.12／0.14、押しを短く）を R10W（3 回跳び）と比べる。j41（既存のノブのみ）。
D=/home/takara/work/dp/articara/log/claude_g1jump_20261021; B=$D/g1_flight_probe_j41
X='"JUMP_ABORT":"1","JUMP_PUSH_R":"0.3","QD_CLAMP":"40","DIAG_TD":"1","FLIGHT_FEET_FLAT":"1","DIAG_HX":"1","RELEASE_FIRST_OFF":"1","WBCW_CONTACT_REST":"0.1","WBCW_TRUNK_LAND":"0.03","FALL_TILT":"1.5","LAND_REF_FOLLOW":"2","AIR_ABORT_T":"0.08"'
H=',"FLIGHT_FEET_CAPTURE_T":"0.06","KP_FLIGHT_FEET":"1000","KD_FLIGHT_FEET":"63","WBCW_MOMENTUM":"10","WBCW_CONTACT":"100","WBCW_TRUNK_LAND_VZ":"0","JUMP_H_NAT":"1","WBCW_CONTACT_REST_LAUNCH":"1","FLIGHT_FEET_CAPTURE":"0.55","FLIGHT_TRUNK":"0","FLIGHT_FEET_W":"10","WBCW_MOMENTUM_PUSH":"1","JUMP_VX_TAB":"1.0:1.0,1.5:1.2,2.0:1.6","JUMP_LEAN_TAB":"1.0:0,1.5:0.045,2.0:0.06"'
RW=',"T":"13","JUMP_REPEAT":"2","FLIGHT_FEET_YAW":"1","WBCW_TRUNK_LAND_T":"1.0","JUMP_REPEAT_SOFT_CROUCH":"1","JUMP_REPEAT_RECENTRE":"1","FLIGHT_FEET_WIDTH":"0.236"'
cd $D
run() { TAG=$1 SEED=31415 BIN=$B CONDS=1.0 N=48 W=4 EXTRA="{$X$H$RW,$2}" python3 jrand.py; }
run P3T03 '"WBCW_TRUNK_PUSH":"0.03"' &
run P3T10 '"WBCW_TRUNK_PUSH":"0.1"' &
run P3C12 '"WBCW_TRUNK_PUSH":"0.003","CROUCH":"0.12"' &
run P3C14 '"WBCW_TRUNK_PUSH":"0.003","CROUCH":"0.14"' &
wait
python3 rsumm.py R10W P3T03 P3T10 P3C12 P3C14 | grep v1.0
for t in P3T03 P3T10 P3C12 P3C14; do echo "== $t"; python3 pushfx.py v1.0 $t | grep -E "pushes|pitch|vz |flight"; done
echo "#### JB65_DONE"
