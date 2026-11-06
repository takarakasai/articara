#!/bin/bash
# CROUCH_TAB 1.0:0.12,1.5:0.18（j43、1.5 以上は R10W と同じ 0.18）を確かめる: 種 27182 の 3 回跳び、10 回跳び 2 つの種、1.25（補間で 0.15）を R10W と、1.0 の CROUCH 0.10。
D=/home/takara/work/dp/articara/log/claude_g1jump_20261021; B=$D/g1_flight_probe_j43
X='"JUMP_ABORT":"1","JUMP_PUSH_R":"0.3","QD_CLAMP":"40","DIAG_TD":"1","FLIGHT_FEET_FLAT":"1","DIAG_HX":"1","RELEASE_FIRST_OFF":"1","WBCW_CONTACT_REST":"0.1","WBCW_TRUNK_LAND":"0.03","FALL_TILT":"1.5","LAND_REF_FOLLOW":"2","AIR_ABORT_T":"0.08"'
H=',"FLIGHT_FEET_CAPTURE_T":"0.06","KP_FLIGHT_FEET":"1000","KD_FLIGHT_FEET":"63","WBCW_TRUNK_PUSH":"0.003","WBCW_MOMENTUM":"10","WBCW_CONTACT":"100","WBCW_TRUNK_LAND_VZ":"0","JUMP_H_NAT":"1","WBCW_CONTACT_REST_LAUNCH":"1","FLIGHT_FEET_CAPTURE":"0.55","FLIGHT_TRUNK":"0","FLIGHT_FEET_W":"10","WBCW_MOMENTUM_PUSH":"1","JUMP_VX_TAB":"1.0:1.0,1.5:1.2,2.0:1.6","JUMP_LEAN_TAB":"1.0:0,1.5:0.045,2.0:0.06"'
RW=',"JUMP_REPEAT":"2","FLIGHT_FEET_YAW":"1","WBCW_TRUNK_LAND_T":"1.0","JUMP_REPEAT_SOFT_CROUCH":"1","JUMP_REPEAT_RECENTRE":"1","FLIGHT_FEET_WIDTH":"0.236"'
CT='"CROUCH_TAB":"1.0:0.12,1.5:0.18"'
cd $D
run() { TAG=$1 SEED=$2 BIN=$B CONDS=$3 N=48 W=$4 EXTRA="{$X$H$RW,$5}" python3 jrand.py; }
run C12s2 27182 1.0 3 '"T":"13",'"$CT" &
run C12T10 31415 1.0 4 '"T":"37","JUMP_REPEAT":"9",'"$CT" &
run C12T10s2 27182 1.0 4 '"T":"37","JUMP_REPEAT":"9",'"$CT" &
run R10Wm 31415 1.25 2 '"T":"13"' &
run C12Tm 31415 1.25 2 '"T":"13",'"$CT" &
run P3C10 31415 1.0 1 '"T":"13","CROUCH":"0.10"' &
wait
python3 rsumm.py R10Ws2 C12s2 | grep v1.0
python3 rsumm.py R10W10 C12T10 | grep v1.0
python3 rsumm.py R10W10s2 C12T10s2 | grep v1.0
python3 rsumm.py R10Wm C12Tm | grep v1.25
python3 rsumm.py R10W P3C12 P3C10 | grep v1.0
echo "#### JB66_DONE"
