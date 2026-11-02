#!/bin/bash
# MPC の重心の前の上限 JUMP_MPC_X_FWD（既定 0.12 は足裏のつま先の縁より前）を 0.03／0.06／0.09 に。ための接触を硬くした（REST_LAUNCH=1）ときは重心が指令どおり前へ出て +0.12 に達し、縦の押しが崩れて（離地 vz 0.3〜0.5）前へ倒れた（jb32）。種 31415、V 1.5、24 本（2026-11-03、j28）。
cd /home/takara/work/dp/articara
source ./setup-mujoco.sh >/dev/null
D=log/claude_g1jump_20261021; B=$D/g1_flight_probe_j28
X='"JUMP_ABORT":"1","JUMP_PUSH_R":"0.3","QD_CLAMP":"40","DIAG_TD":"1","FLIGHT_FEET_FLAT":"1","DIAG_HX":"1","RELEASE_FIRST_OFF":"1","WBCW_CONTACT_REST":"0.1","WBCW_TRUNK_LAND":"0.03","FALL_TILT":"1.5","LAND_REF_FOLLOW":"2","AIR_ABORT_T":"0.08"'
C=',"FLIGHT_FEET_CAPTURE":"0.7","FLIGHT_FEET_CAPTURE_T":"0.06","KP_FLIGHT_FEET":"1000","KD_FLIGHT_FEET":"63","WBCW_TRUNK_PUSH":"0.003","WBCW_MOMENTUM":"10","WBCW_CONTACT":"100","WBCW_TRUNK_LAND_VZ":"0","JUMP_VX":"1.0","JUMP_MPC":"1","JUMP_MPC_WH_RUN":"1000","JUMP_MPC_FROM":"0.8"'
run() { TAG=$1 SEED=31415 BIN=$B CONDS=1.5 N=24 W=4 EXTRA="{$X$C$2}" python3 $D/jrand.py; }
run MRX3 ',"WBCW_CONTACT_REST_LAUNCH":"1","JUMP_MPC_X_FWD":"0.03"' &
run MRX6 ',"WBCW_CONTACT_REST_LAUNCH":"1","JUMP_MPC_X_FWD":"0.06"' &
run MRX9 ',"WBCW_CONTACT_REST_LAUNCH":"1","JUMP_MPC_X_FWD":"0.09"' &
run MAX6 ',"JUMP_MPC_X_FWD":"0.06"' &
wait
echo "#### JB33_DONE"
