#!/bin/bash
# 回す連続の跳躍で入れた 2 つの直し（MU_TORSION 0.02＋TRUNK_RP_YAWED=1）を 1 回跳びの基準で確かめる。
# TB5 の 5 高さ×3 種（TB5all5{,s2,s3} と対）、TV3R の 3 高さ×2 種（RF2A{,s2} と対）。N=48、j51。
D=/home/takara/work/dp/articara/log/claude_g1jump_20261021; B=$D/g1_flight_probe_j51
X='"JUMP_ABORT":"1","JUMP_PUSH_R":"0.3","QD_CLAMP":"40","DIAG_TD":"1","FLIGHT_FEET_FLAT":"1","DIAG_HX":"1","RELEASE_FIRST_OFF":"1","WBCW_CONTACT_REST":"0.1","WBCW_TRUNK_LAND":"0.03","FALL_TILT":"1.5","LAND_REF_FOLLOW":"2","AIR_ABORT_T":"0.08"'
TB5=',"FLIGHT_FEET_CAPTURE_T":"0.06","KP_FLIGHT_FEET":"1000","KD_FLIGHT_FEET":"63","WBCW_TRUNK_PUSH":"0.003","WBCW_MOMENTUM":"10","WBCW_CONTACT":"100","WBCW_TRUNK_LAND_VZ":"0","JUMP_H_NAT":"1","WBCW_CONTACT_REST_LAUNCH":"1","FLIGHT_FEET_CAPTURE":"0.55","JUMP_VX_TAB":"1.0:1.0,1.5:1.2,2.0:1.6","FLIGHT_TRUNK":"0","FLIGHT_FEET_W":"10","WBCW_MOMENTUM_PUSH":"1","JUMP_LEAN_TAB":"1.0:0,1.5:0.045,2.0:0.06"'
TV=',"WBCW_TRUNK_LAND_VZ":"0.7"'
M=',"MU_TORSION":"0.02","TRUNK_RP_YAWED":"1"'
cd $D
run() { TAG=$1 SEED=$2 BIN=$B CONDS=$3 N=48 W=$4 EXTRA="{$X$5}" python3 jrand.py; }
run TBM   31415 1.0,1.25,1.5,1.75,2.0 4 "$TB5$M" &
run TBMs2 27182 1.0,1.25,1.5,1.75,2.0 4 "$TB5$M" &
run TBMs3 16180 1.0,1.25,1.5,1.75,2.0 4 "$TB5$M" &
run TVM   31415 1.0,1.5,2.0 2 "$TV$M" &
run TVMs2 27182 1.0,1.5,2.0 2 "$TV$M" &
wait
for p in "TB5all5 TBM" "TB5all5s2 TBMs2" "TB5all5s3 TBMs3" "RF2A TVM" "RF2As2 TVMs2"; do python3 summ.py $p | grep -v "^${p%% *} "; done
echo "#### JT20_DONE"
