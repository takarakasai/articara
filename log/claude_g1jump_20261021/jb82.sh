#!/bin/bash
# 台から跳び降りる（jb82、§41）: 落ちる着地だけ胴の着地のゲインを変える KP/KD_TRUNK_LAND_DROP 60/150（DD、j75、8e8f10f）を 10〜50 cm×間合い 0.02／0.05／0.10 で。50 cm は元（DN50）と KD だけ（DE50、KD_TRUNK_LAND_DROP 150）も。台なしは落ちないのでゲインが掛からず DN00 と同一（cmp で確認）。種 27182 N=16。
D=/home/takara/work/dp/articara/log/claude_g1jump_20261021; B=$D/g1_flight_probe_j75
X='"JUMP_ABORT":"1","JUMP_PUSH_R":"0.3","QD_CLAMP":"40","DIAG_TD":"1","FLIGHT_FEET_FLAT":"1","DIAG_HX":"1","RELEASE_FIRST_OFF":"1","WBCW_CONTACT_REST":"0.1","WBCW_TRUNK_LAND":"0.03","FALL_TILT":"1.5","LAND_REF_FOLLOW":"2","AIR_ABORT_T":"0.08"'
H=',"FLIGHT_FEET_CAPTURE_T":"0.06","KP_FLIGHT_FEET":"1000","KD_FLIGHT_FEET":"63","WBCW_TRUNK_PUSH":"0.003","WBCW_MOMENTUM":"10","WBCW_CONTACT":"100","WBCW_TRUNK_LAND_VZ":"0","JUMP_H_NAT":"1","WBCW_CONTACT_REST_LAUNCH":"1","FLIGHT_FEET_CAPTURE":"0.55","FLIGHT_TRUNK":"0","FLIGHT_FEET_W":"10","WBCW_MOMENTUM_PUSH":"1","MU_TORSION":"0.02","TRUNK_RP_YAWED":"1"'
TB=',"JUMP_VX_TAB":"1.0:1.0,1.5:1.2,2.0:1.6","JUMP_LEAN_TAB":"1.0:0,1.5:0.045,2.0:0.06"'
R=',"JUMP_REPEAT":"5","JUMP_REPEAT_RECENTRE":"1","JUMP_REPEAT_SOFT_CROUCH":"1","FLIGHT_FEET_WIDTH":"0.236","WBCW_TRUNK_LAND_T":"1.0","T":"24","FLIGHT_FEET_YAW":"1"'
FT='1.5|0:-0.112,0.25:-0.026,0.4:0.021,0.5:0.068,0.55:0.090,0.6:0.124,0.65:0.230,0.7:0.265,0.85:0.290,1.0:0.334,1.1:0.347;2.0|0:-0.148,0.25:-0.026,0.4:0.038,0.5:0.104,0.55:0.131,0.6:0.161,0.65:0.216,0.7:0.249,0.75:0.271,0.85:0.325,1.0:0.362'
cd $D
dn() { # TAG H(m, 0 = 台なし) G SEED N [追加の "K":"V",...]
  local e bx box tg
  e=$(python3 -c "print(round(0.13+$3,3))"); bx=$(python3 -c "print(round($e-2.0,3))"); tg=$(python3 -c "print(round($3+0.25,3))")
  box=""; [ "$2" != "0" ] && box=",\"JUMP_BOX\":\"$bx,$2,2.0\""
  TAG=$1 SEED=$4 BIN=$B CONDS=1.5,2.0 N=$5 W=1 EXTRA="{$X$H$TB$R,\"JUMP_FWD_TAB\":\"$FT\",\"JUMP_FWD_TARGET\":\"$tg\"$box$6}" python3 jrand.py
}
q() { while [ $(jobs -rp | wc -l) -ge 16 ]; do wait -n; done; }
DD=',"KP_TRUNK_LAND_DROP":"60","KD_TRUNK_LAND_DROP":"150"'
for g in 02 05 10; do
  q; dn DN50_$g 0.50 0.$g 27182 16 &
  q; dn DE50_$g 0.50 0.$g 27182 16 ',"KD_TRUNK_LAND_DROP":"150"' &
  for h in 10 20 30 40 50; do q; dn DD${h}_$g 0.$h 0.$g 27182 16 "$DD" & done
done
wait
for g in 02 05 10; do for h in 10 20 30 40 50; do python3 dnsumm.py DN${h}_$g DD${h}_$g; python3 jrand.py cmp DN${h}_$g DD${h}_$g | tail -1; done; python3 dnsumm.py DE50_$g; done
echo "#### JB82_DONE"
