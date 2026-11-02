#!/bin/bash
# 押しの前に重心を前へ（JUMP_LEAN）を、接触を硬くして h_nat を入れた形で。§10 の LEAN は接触 0.1 の下で着地を崩した（HV12L3 23/48）。§13 までの測定は、押しの始まりの重心の位置（CoP の踵の縁との距離）が前への速度の天井と指す。種 31415、V 1.5、24 本（2026-11-03、j30）。
cd /home/takara/work/dp/articara
source ./setup-mujoco.sh >/dev/null
D=log/claude_g1jump_20261021; B=$D/g1_flight_probe_j30
X='"JUMP_ABORT":"1","JUMP_PUSH_R":"0.3","QD_CLAMP":"40","DIAG_TD":"1","FLIGHT_FEET_FLAT":"1","DIAG_HX":"1","RELEASE_FIRST_OFF":"1","WBCW_CONTACT_REST":"0.1","WBCW_TRUNK_LAND":"0.03","FALL_TILT":"1.5","LAND_REF_FOLLOW":"2","AIR_ABORT_T":"0.08"'
C=',"FLIGHT_FEET_CAPTURE":"0.7","FLIGHT_FEET_CAPTURE_T":"0.06","KP_FLIGHT_FEET":"1000","KD_FLIGHT_FEET":"63","WBCW_TRUNK_PUSH":"0.003","WBCW_MOMENTUM":"10","WBCW_CONTACT":"100","WBCW_TRUNK_LAND_VZ":"0","JUMP_H_NAT":"1","WBCW_CONTACT_REST_LAUNCH":"1"'
M=',"JUMP_MPC":"1","JUMP_MPC_WH_RUN":"1000","JUMP_MPC_FROM":"0.8","JUMP_VX":"1.0"'
run() { TAG=$1 SEED=31415 BIN=$B CONDS=1.5 N=24 W=4 EXTRA="{$X$C$2}" python3 $D/jrand.py; }
run HNRL3 ',"JUMP_VX":"1.2","JUMP_LEAN":"0.03"' &
run HNRL6 ',"JUMP_VX":"1.2","JUMP_LEAN":"0.06"' &
run HNRL6v8 ',"JUMP_VX":"0.8","JUMP_LEAN":"0.06"' &
run MNRL6 "$M"',"JUMP_LEAN":"0.06"' &
wait
echo "#### JB38_DONE"
