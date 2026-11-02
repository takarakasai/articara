#!/bin/bash
# 高さの表（j31 の JUMP_LEAN_TAB／JUMP_VX_TAB）＋止まり点の倍率 0.55 を両方の種で確かめる。中間の高さ 1.25／1.75 は HV12k7 と並べる（2026-11-03）。
cd /home/takara/work/dp/articara
source ./setup-mujoco.sh >/dev/null
D=log/claude_g1jump_20261021; B=$D/g1_flight_probe_j31; B27=$D/g1_flight_probe_j27
X='"JUMP_ABORT":"1","JUMP_PUSH_R":"0.3","QD_CLAMP":"40","DIAG_TD":"1","FLIGHT_FEET_FLAT":"1","DIAG_HX":"1","RELEASE_FIRST_OFF":"1","WBCW_CONTACT_REST":"0.1","WBCW_TRUNK_LAND":"0.03","FALL_TILT":"1.5","LAND_REF_FOLLOW":"2","AIR_ABORT_T":"0.08"'
H=',"FLIGHT_FEET_CAPTURE_T":"0.06","KP_FLIGHT_FEET":"1000","KD_FLIGHT_FEET":"63","WBCW_TRUNK_PUSH":"0.003","WBCW_MOMENTUM":"10","WBCW_CONTACT":"100","WBCW_TRUNK_LAND_VZ":"0"'
HV="$H"',"FLIGHT_FEET_CAPTURE":"0.7","JUMP_VX":"1.2"'
N1="$H"',"JUMP_H_NAT":"1","WBCW_CONTACT_REST_LAUNCH":"1","FLIGHT_FEET_CAPTURE":"0.55","JUMP_VX_TAB":"1.0:1.0,1.5:1.2,2.0:1.2"'
TB1="$N1"',"JUMP_LEAN_TAB":"1.0:0,1.5:0.03,2.0:0.03"'
TB2="$N1"',"JUMP_LEAN_TAB":"1.0:0,1.5:0.03,2.0:0.045"'
TB2=${TB2/2.0:1.2\"/2.0:1.6\"}
TAG=TB1f  SEED=31415 BIN=$B CONDS=1.0,1.25,1.5,1.75,2.0 N=48 W=6 EXTRA="{$X$TB1}" python3 $D/jrand.py &
TAG=TB1s2 SEED=27182 BIN=$B CONDS=1.0,1.5,2.0 N=48 W=4 EXTRA="{$X$TB1}" python3 $D/jrand.py &
TAG=TB2f  SEED=31415 BIN=$B CONDS=1.75,2.0 N=48 W=2 EXTRA="{$X$TB2}" python3 $D/jrand.py &
TAG=TB2s2 SEED=27182 BIN=$B CONDS=2.0 N=48 W=2 EXTRA="{$X$TB2}" python3 $D/jrand.py &
TAG=HV12k7mid SEED=31415 BIN=$B27 CONDS=1.25,1.75 N=48 W=2 EXTRA="{$X$HV}" python3 $D/jrand.py &
wait
echo "#### JB44_DONE"
