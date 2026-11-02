#!/bin/bash
# 前出し 6 cm（HNRL6）は離地 vx +0.69・前進 +40 cm だが前への転倒 12/24。転倒は飛行 0.09〜0.14 s の短い跳躍で、足が運び切れず重心の真下か後ろに着く（着地は飛行 ≥0.15 s、足は 3〜6 cm 前）。足の運びを速く（KP 2500）・遠く（倍率 1.0）、飛行の長い V 2.0 も。種 31415、24 本（2026-11-03、j30）。
cd /home/takara/work/dp/articara
source ./setup-mujoco.sh >/dev/null
D=log/claude_g1jump_20261021; B=$D/g1_flight_probe_j30
X='"JUMP_ABORT":"1","JUMP_PUSH_R":"0.3","QD_CLAMP":"40","DIAG_TD":"1","FLIGHT_FEET_FLAT":"1","DIAG_HX":"1","RELEASE_FIRST_OFF":"1","WBCW_CONTACT_REST":"0.1","WBCW_TRUNK_LAND":"0.03","FALL_TILT":"1.5","LAND_REF_FOLLOW":"2","AIR_ABORT_T":"0.08"'
C=',"FLIGHT_FEET_CAPTURE_T":"0.06","WBCW_TRUNK_PUSH":"0.003","WBCW_MOMENTUM":"10","WBCW_CONTACT":"100","WBCW_TRUNK_LAND_VZ":"0","JUMP_H_NAT":"1","WBCW_CONTACT_REST_LAUNCH":"1","JUMP_VX":"1.2","JUMP_LEAN":"0.06"'
run() { TAG=$1 SEED=31415 BIN=$B CONDS=$2 N=24 W=4 EXTRA="{$X$C$3}" python3 $D/jrand.py; }
run HNRL6 2.0 ',"FLIGHT_FEET_CAPTURE":"0.7","KP_FLIGHT_FEET":"1000","KD_FLIGHT_FEET":"63"' &
run L6k10 1.5 ',"FLIGHT_FEET_CAPTURE":"1.0","KP_FLIGHT_FEET":"1000","KD_FLIGHT_FEET":"63"' &
run L6kp 1.5 ',"FLIGHT_FEET_CAPTURE":"0.7","KP_FLIGHT_FEET":"2500","KD_FLIGHT_FEET":"100"' &
run L6kpk10 1.5 ',"FLIGHT_FEET_CAPTURE":"1.0","KP_FLIGHT_FEET":"2500","KD_FLIGHT_FEET":"100"' &
wait
echo "#### JB39_DONE"
