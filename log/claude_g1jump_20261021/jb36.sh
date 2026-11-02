#!/bin/bash
# 片足押しは前の指令とともに増える（VX 0／0.4／0.8／1.2 で 6／30／38／48 of 48）。重心の参照が先へ走り（指令 +30 m/s²）、WBC が接触の重み 100 を破って片足を持ち上げる（MNR10 m00、抜ける足の計画の接触加速度 z +7.5）とみて、WBCW_CONTACT を 1000／10000 に。種 31415、V 1.5、24 本（2026-11-03、j29）。
cd /home/takara/work/dp/articara
source ./setup-mujoco.sh >/dev/null
D=log/claude_g1jump_20261021; B=$D/g1_flight_probe_j29
X='"JUMP_ABORT":"1","JUMP_PUSH_R":"0.3","QD_CLAMP":"40","DIAG_TD":"1","FLIGHT_FEET_FLAT":"1","DIAG_HX":"1","RELEASE_FIRST_OFF":"1","WBCW_CONTACT_REST":"0.1","WBCW_TRUNK_LAND":"0.03","FALL_TILT":"1.5","LAND_REF_FOLLOW":"2","AIR_ABORT_T":"0.08"'
C=',"FLIGHT_FEET_CAPTURE":"0.7","FLIGHT_FEET_CAPTURE_T":"0.06","KP_FLIGHT_FEET":"1000","KD_FLIGHT_FEET":"63","WBCW_TRUNK_PUSH":"0.003","WBCW_MOMENTUM":"10","WBCW_TRUNK_LAND_VZ":"0"'
M=',"JUMP_H_NAT":"1","WBCW_CONTACT_REST_LAUNCH":"1","JUMP_MPC":"1","JUMP_MPC_WH_RUN":"1000","JUMP_MPC_FROM":"0.8","JUMP_VX":"1.0"'
run() { TAG=$1 SEED=31415 BIN=$B CONDS=1.5 N=24 W=4 EXTRA="{$X$C$2}" python3 $D/jrand.py; }
run HC3 ',"JUMP_VX":"1.2","WBCW_CONTACT":"1000"' &
run HC4 ',"JUMP_VX":"1.2","WBCW_CONTACT":"10000"' &
run MNC3 "$M"',"WBCW_CONTACT":"1000"' &
run MNC4 "$M"',"WBCW_CONTACT":"10000"' &
wait
echo "#### JB36_DONE"
