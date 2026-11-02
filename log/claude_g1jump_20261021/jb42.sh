#!/bin/bash
# 1.0 の後ろへの転倒: 前出し 0 で止まり点の倍率 FLIGHT_FEET_CAPTURE を下げる（jb41 の続き、2026-11-03、j30）。
cd /home/takara/work/dp/articara
source ./setup-mujoco.sh >/dev/null
D=log/claude_g1jump_20261021; B=$D/g1_flight_probe_j30
X='"JUMP_ABORT":"1","JUMP_PUSH_R":"0.3","QD_CLAMP":"40","DIAG_TD":"1","FLIGHT_FEET_FLAT":"1","DIAG_HX":"1","RELEASE_FIRST_OFF":"1","WBCW_CONTACT_REST":"0.1","WBCW_TRUNK_LAND":"0.03","FALL_TILT":"1.5","LAND_REF_FOLLOW":"2","AIR_ABORT_T":"0.08"'
C=',"FLIGHT_FEET_CAPTURE_T":"0.06","KP_FLIGHT_FEET":"1000","KD_FLIGHT_FEET":"63","WBCW_TRUNK_PUSH":"0.003","WBCW_MOMENTUM":"10","WBCW_CONTACT":"100","WBCW_TRUNK_LAND_VZ":"0","JUMP_H_NAT":"1","WBCW_CONTACT_REST_LAUNCH":"1"'
run() { # V LEAN VX CAP
  local tag=SC$(echo $1 | tr -d .)X$(echo $3 | tr -d .)C$(echo $4 | tr -d .)
  TAG=$tag SEED=31415 BIN=$B CONDS=$1 N=48 W=8 EXTRA="{$X$C,\"JUMP_LEAN\":\"$2\",\"JUMP_VX\":\"$3\",\"FLIGHT_FEET_CAPTURE\":\"$4\"}" python3 $D/jrand.py
}
J=()
for c in 0.4 0.55; do for vx in 0.6 0.8 1.0; do J+=("1.0 0 $vx $c"); done; done
i=0
for j in "${J[@]}"; do run $j & i=$((i+1)); [ $((i%2)) = 0 ] && wait; done
wait
echo "#### JB42_DONE"
