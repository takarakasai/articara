#!/bin/bash
# 位置を狙う跳躍の表: TB6 の前への速さ・前出しの表を s 倍にして、1 回跳びの足裏の中点の前進（DIAG_FWD_FEET）を細かく測る。1.5・2.0、N=16、種 31415、j54。
D=/home/takara/work/dp/articara/log/claude_g1jump_20261021; B=$D/g1_flight_probe_j54
X='"JUMP_ABORT":"1","JUMP_PUSH_R":"0.3","QD_CLAMP":"40","DIAG_TD":"1","FLIGHT_FEET_FLAT":"1","DIAG_HX":"1","RELEASE_FIRST_OFF":"1","WBCW_CONTACT_REST":"0.1","WBCW_TRUNK_LAND":"0.03","FALL_TILT":"1.5","LAND_REF_FOLLOW":"2","AIR_ABORT_T":"0.08"'
H=',"FLIGHT_FEET_CAPTURE_T":"0.06","KP_FLIGHT_FEET":"1000","KD_FLIGHT_FEET":"63","WBCW_TRUNK_PUSH":"0.003","WBCW_MOMENTUM":"10","WBCW_CONTACT":"100","WBCW_TRUNK_LAND_VZ":"0","JUMP_H_NAT":"1","WBCW_CONTACT_REST_LAUNCH":"1","FLIGHT_FEET_CAPTURE":"0.55","FLIGHT_TRUNK":"0","FLIGHT_FEET_W":"10","WBCW_MOMENTUM_PUSH":"1","MU_TORSION":"0.02","TRUNK_RP_YAWED":"1","DIAG_FWD_FEET":"1"'
cd $D
run() { # TAG 倍率
  local s=$2
  local vx=$(python3 -c "s=$s; print(f'1.0:{1.0*s:.4f},1.5:{1.2*s:.4f},2.0:{1.6*s:.4f}')")
  local ln=$(python3 -c "s=$s; print(f'1.0:0,1.5:{0.045*s:.5f},2.0:{0.06*s:.5f}')")
  TAG=$1 SEED=31415 BIN=$B CONDS=1.5,2.0 N=16 W=1 EXTRA="{$X$H,\"JUMP_VX_TAB\":\"$vx\",\"JUMP_LEAN_TAB\":\"$ln\"}" python3 jrand.py
}
for s in 025 040 050 055 060 065 070 075 085; do run PF$s $(python3 -c "print(int('$s')/100)") & done
wait
python3 fsumm.py PF000 PF025 PF040 PF050 PF055 PF060 PF065 PF070 PF075 PF085 PF100 PF110
echo "#### JP2_DONE"
