#!/bin/bash
# 台から跳び降りる（jb97、§48）: 直しの後ろへの跳躍の弱さ。RDB で直しの跳躍の後に倒れる 23 本のうち 20 本が 2.0（1.5 は 183 本中 3、2.0 は 178 本中 20）、離地の vz 1.35（立つ本 1.10）・h_y +0.17（+0.45）。後ろへの跳躍だけ初速を JUMP_FWD_BACK_V（j84）に: BV15 = RDB＋1.5。1.5 の本は物理が同じ（表示だけ違う）なので 2.0 だけ回す。
D=/home/takara/work/dp/articara/log/claude_g1jump_20261021; B=$D/g1_flight_probe_j84
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
  TAG=$1 SEED=$4 BIN=$B CONDS=${CONDS:-1.5,2.0} N=$5 W=1 EXTRA="{$X$H$TB$R,\"JUMP_FWD_TAB\":\"$FT\",\"JUMP_FWD_TARGET\":\"$tg\"$box$6}" python3 jrand.py
}
DD=',"KP_TRUNK_LAND_DROP":"60","KD_TRUNK_LAND_DROP":"150"'
DR="$DD"',"LAND_RISE_DROP":"0.7","LAND_HOLD_DROP":"0.1","LAND_RISE_DROP_H":"0.25"'
VB="$DR"',"LAND_RISE_DROP_DECEL":"0.65"'
V15="$VB"',"JUMP_FWD_BACK_V":"1.5"'
dn2() { CONDS=2.0 dn "$@"; }
fy() { # TAG TARGET SEED [追加]  平地の位置を狙う跳躍（FY の jf3/jf4 と同じ、2.0 だけ）
  TAG=$1 SEED=$3 BIN=$B CONDS=2.0 N=16 W=1 EXTRA="{$X$H$TB$R,\"JUMP_FWD_TAB\":\"$FT\",\"JUMP_FWD_TARGET\":\"$2\"$4}" python3 jrand.py
}
for h in 30 40 50; do for g in 02 05 10; do
  q; dn2 BV15_${h}_$g 0.$h 0.$g 27182 16 "$V15" & q; dn2 BV15h${h}_$g 0.$h 0.$g 16180 8 "$V15" &
  q; dn2 BV15p${h}_$g 0.$h 0.$g 31415 8 "$V15" &
done; done
for t in 20:0.20 50:0.50 80:0.80 100:1.0 m30:-0.30; do
  q; fy BVY${t%%:*} ${t#*:} 27182 ',"JUMP_FWD_BACK_V":"1.5"' & q; fy BVZ${t%%:*} ${t#*:} 16180 ',"JUMP_FWD_BACK_V":"1.5"' &
done
wait
echo "#### JB97_DONE"
