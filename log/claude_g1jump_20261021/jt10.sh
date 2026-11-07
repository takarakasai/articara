#!/bin/bash
# 回す 3 回跳びの前への速さを離地 vx 0 へ合わせる（回すと離地 vx が +0.2 ずれる: jt9）。表の倍率 1.0/1.5 で 0.10〜0.20、2.0 で 0.25〜0.35、STANCE_TRUNK_YAW 10 の有無。N=8、種 31415、j47。
D=/home/takara/work/dp/articara/log/claude_g1jump_20261021; B=$D/g1_flight_probe_j47
X='"JUMP_ABORT":"1","JUMP_PUSH_R":"0.3","QD_CLAMP":"40","DIAG_TD":"1","FLIGHT_FEET_FLAT":"1","DIAG_HX":"1","RELEASE_FIRST_OFF":"1","WBCW_CONTACT_REST":"0.1","WBCW_TRUNK_LAND":"0.03","FALL_TILT":"1.5","LAND_REF_FOLLOW":"2","AIR_ABORT_T":"0.08"'
H=',"FLIGHT_FEET_CAPTURE_T":"0.06","KP_FLIGHT_FEET":"1000","KD_FLIGHT_FEET":"63","WBCW_TRUNK_PUSH":"0.003","WBCW_MOMENTUM":"10","WBCW_CONTACT":"100","WBCW_TRUNK_LAND_VZ":"0","JUMP_H_NAT":"1","WBCW_CONTACT_REST_LAUNCH":"1","FLIGHT_FEET_CAPTURE":"0.55","FLIGHT_TRUNK":"0","FLIGHT_FEET_W":"10","WBCW_MOMENTUM_PUSH":"1","FLIGHT_FEET_YAW":"1"'
RW=',"T":"13","JUMP_REPEAT":"2","WBCW_TRUNK_LAND_T":"1.0","JUMP_REPEAT_SOFT_CROUCH":"1","JUMP_REPEAT_RECENTRE":"1","FLIGHT_FEET_WIDTH":"0.236","CROUCH_TAB":"1.0:0.12,1.5:0.18","JUMP_REPEAT_HEADING":"1"'
TU=',"JUMP_TURN_HZ":"2","FLIGHT_FEET_TURN":"1","KP_FLIGHT_FEET_ROT":"2000","KD_FLIGHT_FEET_ROT":"89"'
F3=',"JUMP_VX_TAB":"1.0:0.3,1.5:0.36,2.0:0.48","JUMP_LEAN_TAB":"1.0:0,1.5:0.0135,2.0:0.018"'
F5=',"JUMP_VX_TAB":"1.0:0.5,1.5:0.6,2.0:0.8","JUMP_LEAN_TAB":"1.0:0,1.5:0.0225,2.0:0.03"'
cd $D
run() { TAG=$1 SEED=31415 BIN=$B CONDS=1.0,1.5,2.0 N=8 W=4 EXTRA="{$X$H$RW$2}" python3 jrand.py; }
run RT15 ',"JUMP_VX_TAB":"1.0:0.150,1.5:0.180,2.0:0.480","JUMP_LEAN_TAB":"1.0:0,1.5:0.00675,2.0:0.0180"'"$TU"'' &
run RT15s ',"JUMP_VX_TAB":"1.0:0.150,1.5:0.180,2.0:0.480","JUMP_LEAN_TAB":"1.0:0,1.5:0.00675,2.0:0.0180"'"$TU"',"STANCE_TRUNK_YAW":"10"' &
run RT10s ',"JUMP_VX_TAB":"1.0:0.100,1.5:0.120,2.0:0.400","JUMP_LEAN_TAB":"1.0:0,1.5:0.00450,2.0:0.0150"'"$TU"',"STANCE_TRUNK_YAW":"10"' &
run RT20s ',"JUMP_VX_TAB":"1.0:0.200,1.5:0.240,2.0:0.560","JUMP_LEAN_TAB":"1.0:0,1.5:0.00900,2.0:0.0210"'"$TU"',"STANCE_TRUNK_YAW":"10"' &
wait
python3 rtsumm.py RT3s RT15 RT15s RT10s RT20s
for t in RT15 RT15s RT10s RT20s; do for c in v1.0 v1.5 v2.0; do echo "$t $c $(for f in runs/$t/$c/m*/run.log; do grep -m1 '\[lift-off\]' $f; done | sed 's/.* v (\([-+.0-9]*\), .*/\1/' | sort -n | awk '{a[NR]=$1} END{print "1st lift-off vx med", a[int(NR/2)+1]}')"; done; done
echo "#### JT10_DONE"
