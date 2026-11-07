#!/bin/bash
# 向きを変える跳躍の最初の測り: その場の跳躍（TB5 から前への速さ・前出しを外す、FLIGHT_FEET_YAW=1）に JUMP_TURN_HZ 0／2／4／8 と FLIGHT_FEET_TURN=1。1.5・2.0、N=8、種 31415、j44。
D=/home/takara/work/dp/articara/log/claude_g1jump_20261021; B=$D/g1_flight_probe_j44
X='"JUMP_ABORT":"1","JUMP_PUSH_R":"0.3","QD_CLAMP":"40","DIAG_TD":"1","FLIGHT_FEET_FLAT":"1","DIAG_HX":"1","RELEASE_FIRST_OFF":"1","WBCW_CONTACT_REST":"0.1","WBCW_TRUNK_LAND":"0.03","FALL_TILT":"1.5","LAND_REF_FOLLOW":"2","AIR_ABORT_T":"0.08"'
H=',"FLIGHT_FEET_CAPTURE_T":"0.06","KP_FLIGHT_FEET":"1000","KD_FLIGHT_FEET":"63","WBCW_TRUNK_PUSH":"0.003","WBCW_MOMENTUM":"10","WBCW_CONTACT":"100","WBCW_TRUNK_LAND_VZ":"0","JUMP_H_NAT":"1","WBCW_CONTACT_REST_LAUNCH":"1","FLIGHT_FEET_CAPTURE":"0.55","FLIGHT_TRUNK":"0","FLIGHT_FEET_W":"10","WBCW_MOMENTUM_PUSH":"1","FLIGHT_FEET_YAW":"1"'
cd $D
run() { TAG=$1 SEED=31415 BIN=$B CONDS=1.5,2.0 N=8 W=4 EXTRA="{$X$H,$2}" python3 jrand.py; }
run TZ0 '"T":"6"' &
run TZ2 '"T":"6","JUMP_TURN_HZ":"2","FLIGHT_FEET_TURN":"1"' &
run TZ4 '"T":"6","JUMP_TURN_HZ":"4","FLIGHT_FEET_TURN":"1"' &
run TZ8 '"T":"6","JUMP_TURN_HZ":"8","FLIGHT_FEET_TURN":"1"' &
wait
for t in TZ0 TZ2 TZ4 TZ8; do for c in v1.5 v2.0; do
  echo "== $t $c: $(grep -l 'verdict: LANDED' runs/$t/$c/m*/run.log | wc -l)/8 landed"
  grep -h "^  turn:" runs/$t/$c/m*/run.log | head -8
done; done
echo "#### JT1_DONE"
