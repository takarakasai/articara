#!/bin/bash
# 前出し 3 cm（HNRL3 = HV12k7 ＋ 接触を硬く ＋ h_nat ＋ JUMP_LEAN 0.03）を比べ先 HV12k7 と同じ物差しで: 2 つの種 × V 1.0／1.5／2.0 × 48 本（2026-11-03、j30）。
cd /home/takara/work/dp/articara
source ./setup-mujoco.sh >/dev/null
D=log/claude_g1jump_20261021; B=$D/g1_flight_probe_j30
X='"JUMP_ABORT":"1","JUMP_PUSH_R":"0.3","QD_CLAMP":"40","DIAG_TD":"1","FLIGHT_FEET_FLAT":"1","DIAG_HX":"1","RELEASE_FIRST_OFF":"1","WBCW_CONTACT_REST":"0.1","WBCW_TRUNK_LAND":"0.03","FALL_TILT":"1.5","LAND_REF_FOLLOW":"2","AIR_ABORT_T":"0.08"'
C=',"FLIGHT_FEET_CAPTURE":"0.7","FLIGHT_FEET_CAPTURE_T":"0.06","KP_FLIGHT_FEET":"1000","KD_FLIGHT_FEET":"63","WBCW_TRUNK_PUSH":"0.003","WBCW_MOMENTUM":"10","WBCW_CONTACT":"100","WBCW_TRUNK_LAND_VZ":"0","JUMP_H_NAT":"1","WBCW_CONTACT_REST_LAUNCH":"1","JUMP_VX":"1.2","JUMP_LEAN":"0.03"'
TAG=HNRL3f SEED=31415 BIN=$B CONDS=1.0,1.5,2.0 N=48 W=8 EXTRA="{$X$C}" python3 $D/jrand.py &
TAG=HNRL3s2 SEED=27182 BIN=$B CONDS=1.0,1.5,2.0 N=48 W=8 EXTRA="{$X$C}" python3 $D/jrand.py &
wait
echo "#### JB40_DONE"
