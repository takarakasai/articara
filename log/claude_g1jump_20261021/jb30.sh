#!/bin/bash
# HV12k7 を前への跳躍の基準に固める確認: 2 つ目の種（27182）と垂直の JUMP_V 1.0（2026-11-03、j27）。比べ先は TV3R（RF2A／RF2As2）。
cd /home/takara/work/dp/articara
source ./setup-mujoco.sh >/dev/null
D=log/claude_g1jump_20261021; B=$D/g1_flight_probe_j27
X='"JUMP_ABORT":"1","JUMP_PUSH_R":"0.3","QD_CLAMP":"40","DIAG_TD":"1","FLIGHT_FEET_FLAT":"1","DIAG_HX":"1","RELEASE_FIRST_OFF":"1","WBCW_CONTACT_REST":"0.1","WBCW_TRUNK_LAND":"0.03","FALL_TILT":"1.5","LAND_REF_FOLLOW":"2","AIR_ABORT_T":"0.08"'
C=',"FLIGHT_FEET_CAPTURE":"0.7","FLIGHT_FEET_CAPTURE_T":"0.06","KP_FLIGHT_FEET":"1000","KD_FLIGHT_FEET":"63","WBCW_TRUNK_PUSH":"0.003","WBCW_MOMENTUM":"10","WBCW_CONTACT":"100","WBCW_TRUNK_LAND_VZ":"0","JUMP_VX":"1.2"'
# 種 31415 の 1.0（1.5／2.0 は runs/HV12k7）と、種 27182 の 1.0／1.5／2.0
TAG=HV12k7v10 SEED=31415 BIN=$B CONDS=1.0 N=48 W=4 EXTRA="{$X$C}" python3 $D/jrand.py &
TAG=HV12k7s2 SEED=27182 BIN=$B CONDS=1.0,1.5,2.0 N=48 W=8 EXTRA="{$X$C}" python3 $D/jrand.py &
wait
echo "#### JB30_DONE"
