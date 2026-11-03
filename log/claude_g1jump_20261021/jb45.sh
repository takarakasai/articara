#!/bin/bash
# 空中で足を引き上げる FLIGHT_FEET_TUCK（j32）を基準 TB2 に。前出し 6 cm と組んだものも（種 31415、48 本、2026-11-03）。
cd /home/takara/work/dp/articara
source ./setup-mujoco.sh >/dev/null
D=log/claude_g1jump_20261021; B=$D/g1_flight_probe_j32
X='"JUMP_ABORT":"1","JUMP_PUSH_R":"0.3","QD_CLAMP":"40","DIAG_TD":"1","FLIGHT_FEET_FLAT":"1","DIAG_HX":"1","RELEASE_FIRST_OFF":"1","WBCW_CONTACT_REST":"0.1","WBCW_TRUNK_LAND":"0.03","FALL_TILT":"1.5","LAND_REF_FOLLOW":"2","AIR_ABORT_T":"0.08"'
H=',"FLIGHT_FEET_CAPTURE_T":"0.06","KP_FLIGHT_FEET":"1000","KD_FLIGHT_FEET":"63","WBCW_TRUNK_PUSH":"0.003","WBCW_MOMENTUM":"10","WBCW_CONTACT":"100","WBCW_TRUNK_LAND_VZ":"0","JUMP_H_NAT":"1","WBCW_CONTACT_REST_LAUNCH":"1","FLIGHT_FEET_CAPTURE":"0.55"'
TB2="$H"',"JUMP_VX_TAB":"1.0:1.0,1.5:1.2,2.0:1.6","JUMP_LEAN_TAB":"1.0:0,1.5:0.03,2.0:0.045"'
L6="$H"',"JUMP_VX_TAB":"1.0:1.0,1.5:1.2,2.0:1.6","JUMP_LEAN_TAB":"1.0:0,1.5:0.06,2.0:0.06"'
J=()
for d in 0.02 0.04 0.06; do J+=("TK$(echo $d | tr -d .) 1.0,1.5,2.0 TB2 $d"); done
for d in 0 0.04; do J+=("TL6K$(echo $d | tr -d .) 1.5,2.0 L6 $d"); done
for j in "${J[@]}"; do set -- $j
  E=${!3}
  TAG=$1 SEED=31415 BIN=$B CONDS=$2 N=48 W=3 EXTRA="{$X$E,\"FLIGHT_FEET_TUCK\":\"$4\"}" python3 $D/jrand.py &
done
wait
echo "#### JB45_DONE"
