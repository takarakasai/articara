#!/bin/bash
# 体の形から出る角運動量を参照に（JUMP_H_NAT、j29）: しゃがみ・ための h_y は胴まっすぐで脚を畳む動きで決まる（≈3 vz）。0 を求めると、接触が硬いとき重心が前へ流れしゃがみが浅くなった（jb32）。台本（HV12k7）と MPC（MA8）に、接触を硬くする REST_LAUNCH と組み合わせて。種 31415、V 1.5、24 本（2026-11-03）。
cd /home/takara/work/dp/articara
source ./setup-mujoco.sh >/dev/null
D=log/claude_g1jump_20261021; B=$D/g1_flight_probe_j29
X='"JUMP_ABORT":"1","JUMP_PUSH_R":"0.3","QD_CLAMP":"40","DIAG_TD":"1","FLIGHT_FEET_FLAT":"1","DIAG_HX":"1","RELEASE_FIRST_OFF":"1","WBCW_CONTACT_REST":"0.1","WBCW_TRUNK_LAND":"0.03","FALL_TILT":"1.5","LAND_REF_FOLLOW":"2","AIR_ABORT_T":"0.08"'
C=',"FLIGHT_FEET_CAPTURE":"0.7","FLIGHT_FEET_CAPTURE_T":"0.06","KP_FLIGHT_FEET":"1000","KD_FLIGHT_FEET":"63","WBCW_TRUNK_PUSH":"0.003","WBCW_MOMENTUM":"10","WBCW_CONTACT":"100","WBCW_TRUNK_LAND_VZ":"0","JUMP_H_NAT":"1"'
M=',"JUMP_MPC":"1","JUMP_MPC_WH_RUN":"1000","JUMP_MPC_FROM":"0.8","JUMP_VX":"1.0"'
run() { TAG=$1 SEED=31415 BIN=$B CONDS=1.5 N=24 W=4 EXTRA="{$X$C$2}" python3 $D/jrand.py; }
run HN12 ',"JUMP_VX":"1.2"' &
run HNR12 ',"JUMP_VX":"1.2","WBCW_CONTACT_REST_LAUNCH":"1"' &
run HNR12P ',"JUMP_VX":"1.2","WBCW_CONTACT_REST_LAUNCH":"1","JUMP_H_NAT_PUSH":"1"' &
run MNR10 ',"WBCW_CONTACT_REST_LAUNCH":"1"'"$M" &
wait
echo "#### JB34_DONE"
