#!/bin/bash
# 空中の足のタスクが胴のタスクに負けていた: FLIGHT_FEET_W × FLIGHT_TRUNK を基準 TB2 に。前出し 6 cm とも（種 31415、48 本、2026-11-03、j33）。
cd /home/takara/work/dp/articara
source ./setup-mujoco.sh >/dev/null
D=log/claude_g1jump_20261021; B=$D/g1_flight_probe_j33
X='"JUMP_ABORT":"1","JUMP_PUSH_R":"0.3","QD_CLAMP":"40","DIAG_TD":"1","FLIGHT_FEET_FLAT":"1","DIAG_HX":"1","RELEASE_FIRST_OFF":"1","WBCW_CONTACT_REST":"0.1","WBCW_TRUNK_LAND":"0.03","FALL_TILT":"1.5","LAND_REF_FOLLOW":"2","AIR_ABORT_T":"0.08"'
H=',"FLIGHT_FEET_CAPTURE_T":"0.06","KP_FLIGHT_FEET":"1000","KD_FLIGHT_FEET":"63","WBCW_TRUNK_PUSH":"0.003","WBCW_MOMENTUM":"10","WBCW_CONTACT":"100","WBCW_TRUNK_LAND_VZ":"0","JUMP_H_NAT":"1","WBCW_CONTACT_REST_LAUNCH":"1","FLIGHT_FEET_CAPTURE":"0.55"'
TB2="$H"',"JUMP_VX_TAB":"1.0:1.0,1.5:1.2,2.0:1.6","JUMP_LEAN_TAB":"1.0:0,1.5:0.03,2.0:0.045"'
L6="$H"',"JUMP_VX_TAB":"1.0:1.0,1.5:1.2,2.0:1.6","JUMP_LEAN_TAB":"1.0:0,1.5:0.06,2.0:0.06"'
J=()
for w in 3 10 30; do for tr in 1 0; do J+=("FW${w}T$tr 1.0,1.5,2.0 TB2 $w $tr"); done; done
J+=("L6FW10T0 1.5,2.0 L6 10 0" "L6FW30T0 1.5,2.0 L6 30 0")
for j in "${J[@]}"; do set -- $j
  E=${!3}
  TAG=$1 SEED=31415 BIN=$B CONDS=$2 N=48 W=2 EXTRA="{$X$E,\"FLIGHT_FEET_W\":\"$4\",\"FLIGHT_TRUNK\":\"$5\"}" python3 $D/jrand.py &
done
wait
echo "#### JB46_DONE"
