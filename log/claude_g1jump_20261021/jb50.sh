#!/bin/bash
# TB4 候補（TB3 の足の重み 30→10、1.5 の前出し 3→4.5 cm）を中間の高さと 3 つ目の種で TB3 と比べる（2026-11-03、j33）。
# TB4b は 1.25 の前出しを 0 に（1.0〜1.25 を平らに）。
cd /home/takara/work/dp/articara
source ./setup-mujoco.sh >/dev/null
D=log/claude_g1jump_20261021; B=$D/g1_flight_probe_j33
X='"JUMP_ABORT":"1","JUMP_PUSH_R":"0.3","QD_CLAMP":"40","DIAG_TD":"1","FLIGHT_FEET_FLAT":"1","DIAG_HX":"1","RELEASE_FIRST_OFF":"1","WBCW_CONTACT_REST":"0.1","WBCW_TRUNK_LAND":"0.03","FALL_TILT":"1.5","LAND_REF_FOLLOW":"2","AIR_ABORT_T":"0.08"'
H=',"FLIGHT_FEET_CAPTURE_T":"0.06","KP_FLIGHT_FEET":"1000","KD_FLIGHT_FEET":"63","WBCW_TRUNK_PUSH":"0.003","WBCW_MOMENTUM":"10","WBCW_CONTACT":"100","WBCW_TRUNK_LAND_VZ":"0","JUMP_H_NAT":"1","WBCW_CONTACT_REST_LAUNCH":"1","FLIGHT_FEET_CAPTURE":"0.55","JUMP_VX_TAB":"1.0:1.0,1.5:1.2,2.0:1.6","FLIGHT_TRUNK":"0"'
TB3="$H"',"FLIGHT_FEET_W":"30","JUMP_LEAN_TAB":"1.0:0,1.5:0.03,2.0:0.045"'
TB4="$H"',"FLIGHT_FEET_W":"10","JUMP_LEAN_TAB":"1.0:0,1.5:0.045,2.0:0.045"'
TB4b="$H"',"FLIGHT_FEET_W":"10","JUMP_LEAN_TAB":"1.0:0,1.25:0,1.5:0.045,2.0:0.045"'
# 名前 高さ 構成 種 並び
J=("TB4mid 1.25,1.75 TB4 31415 2" "TB4mids2 1.25,1.75 TB4 27182 2"
   "TB4b125 1.25 TB4b 31415 1" "TB4b125s2 1.25 TB4b 27182 1"
   "TB3s3 1.0,1.5,2.0 TB3 16180 3" "TB4s3 1.0,1.5,2.0 TB4 16180 3"
   "TB3mids3 1.25,1.75 TB3 16180 2" "TB4mids3 1.25,1.75 TB4 16180 2")
for j in "${J[@]}"; do set -- $j
  E=${!3}
  TAG=$1 SEED=$4 BIN=$B CONDS=$2 N=48 W=$5 EXTRA="{$X$E}" python3 $D/jrand.py &
done
wait
echo "#### JB50_DONE"
