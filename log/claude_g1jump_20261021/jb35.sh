#!/bin/bash
# 押しの終わりは全試行で片足（release の時点でもう片足が 0 N、HV12k7 48/48）。CoP が踵の縁に張り付き足が転がるとみて、押しの間の踵の縁を前へ（JUMP_HEEL_CUT）。MPC は CoP の余白も合わせる。種 31415、V 1.5、24 本（2026-11-03、j29）。
cd /home/takara/work/dp/articara
source ./setup-mujoco.sh >/dev/null
D=log/claude_g1jump_20261021; B=$D/g1_flight_probe_j29
X='"JUMP_ABORT":"1","JUMP_PUSH_R":"0.3","QD_CLAMP":"40","DIAG_TD":"1","FLIGHT_FEET_FLAT":"1","DIAG_HX":"1","RELEASE_FIRST_OFF":"1","WBCW_CONTACT_REST":"0.1","WBCW_TRUNK_LAND":"0.03","FALL_TILT":"1.5","LAND_REF_FOLLOW":"2","AIR_ABORT_T":"0.08"'
C=',"FLIGHT_FEET_CAPTURE":"0.7","FLIGHT_FEET_CAPTURE_T":"0.06","KP_FLIGHT_FEET":"1000","KD_FLIGHT_FEET":"63","WBCW_TRUNK_PUSH":"0.003","WBCW_MOMENTUM":"10","WBCW_CONTACT":"100","WBCW_TRUNK_LAND_VZ":"0"'
M=',"JUMP_H_NAT":"1","WBCW_CONTACT_REST_LAUNCH":"1","JUMP_MPC":"1","JUMP_MPC_WH_RUN":"1000","JUMP_MPC_FROM":"0.8","JUMP_VX":"1.0"'
run() { TAG=$1 SEED=31415 BIN=$B CONDS=1.5 N=24 W=4 EXTRA="{$X$C$2}" python3 $D/jrand.py; }
run Hh3 ',"JUMP_VX":"1.2","JUMP_HEEL_CUT":"0.03"' &
run HNRh3 ',"JUMP_VX":"1.2","JUMP_H_NAT":"1","WBCW_CONTACT_REST_LAUNCH":"1","JUMP_HEEL_CUT":"0.03"' &
run MNRh3 "$M"',"JUMP_HEEL_CUT":"0.03","JUMP_MPC_COP_MARGIN":"0.035"' &
run MNRh5 "$M"',"JUMP_HEEL_CUT":"0.05","JUMP_MPC_COP_MARGIN":"0.055"' &
wait
echo "#### JB35_DONE"
