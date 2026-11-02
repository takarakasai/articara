#!/bin/bash
# 前への速度を飛行時間とそろえる: 高さごとに前出し JUMP_LEAN × 前の指令 JUMP_VX を掃引（HNRL3 の土台、種 31415、48 本、2026-11-03、j30）。
cd /home/takara/work/dp/articara
source ./setup-mujoco.sh >/dev/null
D=log/claude_g1jump_20261021; B=$D/g1_flight_probe_j30
X='"JUMP_ABORT":"1","JUMP_PUSH_R":"0.3","QD_CLAMP":"40","DIAG_TD":"1","FLIGHT_FEET_FLAT":"1","DIAG_HX":"1","RELEASE_FIRST_OFF":"1","WBCW_CONTACT_REST":"0.1","WBCW_TRUNK_LAND":"0.03","FALL_TILT":"1.5","LAND_REF_FOLLOW":"2","AIR_ABORT_T":"0.08"'
C=',"FLIGHT_FEET_CAPTURE":"0.7","FLIGHT_FEET_CAPTURE_T":"0.06","KP_FLIGHT_FEET":"1000","KD_FLIGHT_FEET":"63","WBCW_TRUNK_PUSH":"0.003","WBCW_MOMENTUM":"10","WBCW_CONTACT":"100","WBCW_TRUNK_LAND_VZ":"0","JUMP_H_NAT":"1","WBCW_CONTACT_REST_LAUNCH":"1"'
run() { # V LEAN VX
  local tag=SW$(echo $1 | tr -d .)L$(printf '%03d' $(python3 -c "print(round($2*1000))"))X$(echo $3 | tr -d .)
  TAG=$tag SEED=31415 BIN=$B CONDS=$1 N=48 W=8 EXTRA="{$X$C,\"JUMP_LEAN\":\"$2\",\"JUMP_VX\":\"$3\"}" python3 $D/jrand.py
}
J=()
for l in 0 0.015 0.03; do for vx in 0.4 0.6 0.8 1.0; do J+=("1.0 $l $vx"); done; done
J+=("1.5 0.03 1.5" "1.5 0.045 1.2" "1.5 0.045 1.5")
for l in 0.03 0.045 0.06; do for vx in 1.2 1.6 2.0; do [ "$l $vx" = "0.03 1.2" ] || J+=("2.0 $l $vx"); done; done
i=0
for j in "${J[@]}"; do run $j & i=$((i+1)); [ $((i%2)) = 0 ] && wait; done
wait
echo "#### JB41_DONE"
