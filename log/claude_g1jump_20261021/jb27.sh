#!/bin/bash
# H8 の 1.5 は着地の後に胴を重み 0.3 で起こした反動で後ろへ倒れる（着地 vz が門 0.7 未満）。どの着地でも胴の重みを 0.03 に（WBCW_TRUNK_LAND_VZ=0）（2026-11-02、j25、種 31415）。
cd /home/takara/work/dp/articara
source ./setup-mujoco.sh >/dev/null
D=log/claude_g1jump_20261021; B=$D/g1_flight_probe_j25
X='"JUMP_ABORT":"1","JUMP_PUSH_R":"0.3","QD_CLAMP":"40","DIAG_TD":"1","FLIGHT_FEET_FLAT":"1","DIAG_HX":"1","RELEASE_FIRST_OFF":"1","WBCW_CONTACT_REST":"0.1","WBCW_TRUNK_LAND":"0.03","FALL_TILT":"1.5","LAND_REF_FOLLOW":"2","AIR_ABORT_T":"0.08"'
C=',"FLIGHT_FEET_CAPTURE":"1","FLIGHT_FEET_CAPTURE_T":"0.06","KP_FLIGHT_FEET":"1000","KD_FLIGHT_FEET":"63","WBCW_TRUNK_PUSH":"0.003","WBCW_MOMENTUM":"10","WBCW_CONTACT":"100","WBCW_TRUNK_LAND_VZ":"0"'
run() { TAG=$1 SEED=31415 BIN=$B CONDS=$3 N=48 W=4 EXTRA="{$X$2}" python3 $D/jrand.py; }
run H8L ',"JUMP_VX":"0.8"'"$C" 1.5,2.0
python3 $D/jrand.py cmp H8 H8L
echo "#### JB27_DONE"
