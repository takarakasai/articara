#!/bin/bash
# 台を高くする下調べ（jb14、§34）: 基準 BX（jb13 の JR1_ と同じ設定、j64 = base_bx）で台の高さだけ 0.075・0.10 に。0.10 は JUMP_BOX_TUCK 0.05（BX のまま）と 0.10（足を上げる量を台の高さに合わせる）。台 0.20・0.30・0.40・0.60、1.5・2.0、N=8、種 16180（jb12 の JQ1_ と同じ本）。
D=/home/takara/work/dp/articara/log/claude_g1jump_20261021; B=$D/g1_flight_probe_base_bx
X='"JUMP_ABORT":"1","JUMP_PUSH_R":"0.3","QD_CLAMP":"40","DIAG_TD":"1","FLIGHT_FEET_FLAT":"1","DIAG_HX":"1","RELEASE_FIRST_OFF":"1","WBCW_CONTACT_REST":"0.1","WBCW_TRUNK_LAND":"0.03","FALL_TILT":"1.5","LAND_REF_FOLLOW":"2","AIR_ABORT_T":"0.08"'
H=',"FLIGHT_FEET_CAPTURE_T":"0.06","KP_FLIGHT_FEET":"1000","KD_FLIGHT_FEET":"63","WBCW_TRUNK_PUSH":"0.003","WBCW_MOMENTUM":"10","WBCW_CONTACT":"100","WBCW_TRUNK_LAND_VZ":"0","JUMP_H_NAT":"1","WBCW_CONTACT_REST_LAUNCH":"1","FLIGHT_FEET_CAPTURE":"0.55","FLIGHT_TRUNK":"0","FLIGHT_FEET_W":"10","WBCW_MOMENTUM_PUSH":"1","MU_TORSION":"0.02","TRUNK_RP_YAWED":"1"'
TB=',"JUMP_VX_TAB":"1.0:1.0,1.5:1.2,2.0:1.6","JUMP_LEAN_TAB":"1.0:0,1.5:0.045,2.0:0.06"'
R=',"JUMP_REPEAT":"5","JUMP_REPEAT_RECENTRE":"1","JUMP_REPEAT_SOFT_CROUCH":"1","FLIGHT_FEET_WIDTH":"0.236","WBCW_TRUNK_LAND_T":"1.0","T":"24","FLIGHT_FEET_YAW":"1"'
FT='1.5|0:-0.112,0.25:-0.026,0.4:0.021,0.5:0.068,0.55:0.090,0.6:0.124,0.65:0.230,0.7:0.265,0.85:0.290,1.0:0.334,1.1:0.347;2.0|0:-0.148,0.25:-0.026,0.4:0.038,0.5:0.104,0.55:0.131,0.6:0.161,0.65:0.216,0.7:0.249,0.75:0.271,0.85:0.325,1.0:0.362'
cd $D
# run TAG BOX TUCK
run() { TAG=$1 SEED=16180 BIN=$B CONDS=1.5,2.0 N=8 W=1 EXTRA="{$X$H$TB$R,\"JUMP_FWD_TAB\":\"$FT\",\"JUMP_BOX_TUCK\":\"$3\",\"JUMP_BOX\":\"$2\",$M}" python3 jrand.py; }
M='"JUMP_BOX_GAP":"0.10","JUMP_BOX_GAP_TOL":"0.025","JUMP_BOX_S":"1.2","JUMP_BOX_CROUCH_W":"1"'
for b in 20 30 40 60; do
  run JH75_$b 0.$b,0.075 0.05 &
  run JH10_$b 0.$b,0.10 0.05 &
  run JH10T_$b 0.$b,0.10 0.10 &
done
wait
for p in JH75 JH10 JH10T; do python3 jbsumm.py ${p}_20 ${p}_30 ${p}_40 ${p}_60; python3 jbjump.py ${p}_20 ${p}_30 ${p}_40 ${p}_60; done
echo "#### JB14_DONE"
