#!/bin/bash
# 片足押しの仕組み（DIAG_PUSHF、HV12k7 m00）: 両足の CoP が踵の縁に張り付くと、QP は左右の荷重を分けて（右が平均 293 N 多い）ロールを横の力 × 高さで打ち消し、右足が横に 20° 転がる。VX 0 では左右差 2.5 N。押しの間だけ両足の縦の力をそろえる JUMP_FZ_SYM_W を 1／10／100。種 31415、V 1.5、24 本（2026-11-03、j30）。
cd /home/takara/work/dp/articara
source ./setup-mujoco.sh >/dev/null
D=log/claude_g1jump_20261021; B=$D/g1_flight_probe_j30
X='"JUMP_ABORT":"1","JUMP_PUSH_R":"0.3","QD_CLAMP":"40","DIAG_TD":"1","FLIGHT_FEET_FLAT":"1","DIAG_HX":"1","RELEASE_FIRST_OFF":"1","WBCW_CONTACT_REST":"0.1","WBCW_TRUNK_LAND":"0.03","FALL_TILT":"1.5","LAND_REF_FOLLOW":"2","AIR_ABORT_T":"0.08"'
C=',"FLIGHT_FEET_CAPTURE":"0.7","FLIGHT_FEET_CAPTURE_T":"0.06","KP_FLIGHT_FEET":"1000","KD_FLIGHT_FEET":"63","WBCW_TRUNK_PUSH":"0.003","WBCW_MOMENTUM":"10","WBCW_CONTACT":"100","WBCW_TRUNK_LAND_VZ":"0"'
M=',"JUMP_H_NAT":"1","WBCW_CONTACT_REST_LAUNCH":"1","JUMP_MPC":"1","JUMP_MPC_WH_RUN":"1000","JUMP_MPC_FROM":"0.8","JUMP_VX":"1.0"'
run() { TAG=$1 SEED=31415 BIN=$B CONDS=1.5 N=24 W=4 EXTRA="{$X$C$2}" python3 $D/jrand.py; }
run HS1 ',"JUMP_VX":"1.2","JUMP_FZ_SYM_W":"1"' &
run HS10 ',"JUMP_VX":"1.2","JUMP_FZ_SYM_W":"10"' &
run HS100 ',"JUMP_VX":"1.2","JUMP_FZ_SYM_W":"100"' &
run MNS10 "$M"',"JUMP_FZ_SYM_W":"10"' &
wait
echo "#### JB37_DONE"
