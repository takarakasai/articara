#!/bin/bash
# 踏み切りの MPC（JUMP_MPC、j28）の下見: 始める時刻（しゃがみの頭 0／ために入る 0.8）と解き直しの起点（計画の状態／実測）。VX 1.0、途中の h_y の重み 1000、種 31415、V 1.5、24 本（2026-11-03）。比べ先は runs/HV12k7 の同じ m00〜m23。
cd /home/takara/work/dp/articara
source ./setup-mujoco.sh >/dev/null
D=log/claude_g1jump_20261021; B=$D/g1_flight_probe_j28
X='"JUMP_ABORT":"1","JUMP_PUSH_R":"0.3","QD_CLAMP":"40","DIAG_TD":"1","FLIGHT_FEET_FLAT":"1","DIAG_HX":"1","RELEASE_FIRST_OFF":"1","WBCW_CONTACT_REST":"0.1","WBCW_TRUNK_LAND":"0.03","FALL_TILT":"1.5","LAND_REF_FOLLOW":"2","AIR_ABORT_T":"0.08"'
C=',"FLIGHT_FEET_CAPTURE":"0.7","FLIGHT_FEET_CAPTURE_T":"0.06","KP_FLIGHT_FEET":"1000","KD_FLIGHT_FEET":"63","WBCW_TRUNK_PUSH":"0.003","WBCW_MOMENTUM":"10","WBCW_CONTACT":"100","WBCW_TRUNK_LAND_VZ":"0","JUMP_VX":"1.0","JUMP_MPC":"1","JUMP_MPC_WH_RUN":"1000"'
run() { TAG=$1 SEED=31415 BIN=$B CONDS=1.5 N=24 W=4 EXTRA="{$X$C$2}" python3 $D/jrand.py; }
run MA0 ',"JUMP_MPC_FROM":"0"' &
run MA8 ',"JUMP_MPC_FROM":"0.8"' &
run MM8 ',"JUMP_MPC_FROM":"0.8","JUMP_MPC_MEAS":"1"' &
run MA5 ',"JUMP_MPC_FROM":"0.5"' &
wait
echo "#### JB31_DONE"
