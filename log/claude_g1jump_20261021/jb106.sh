#!/bin/bash
# 横の位置を狙う跳躍（jb106、§50）: 2.0 の 1 回目に決まった条件（27182 m01、16180 m03）が着地の後に後ろへ流れて倒れる（摩擦 0.61、足が 0.6 s 滑る）。既存の着地のノブを jb105 の全体で: LK = KP/KD_TRUNK_LAND 60/150、LS = LAND_REF_SOLES 0.5、LH = LAND_HOLD 0.1。
D=/home/takara/work/dp/articara/log/claude_g1jump_20261021; B=$D/g1_flight_probe_j86
X='"JUMP_ABORT":"1","JUMP_PUSH_R":"0.3","QD_CLAMP":"40","DIAG_TD":"1","FLIGHT_FEET_FLAT":"1","DIAG_HX":"1","RELEASE_FIRST_OFF":"1","WBCW_CONTACT_REST":"0.1","WBCW_TRUNK_LAND":"0.03","FALL_TILT":"1.5","LAND_REF_FOLLOW":"2","AIR_ABORT_T":"0.08"'
H=',"FLIGHT_FEET_CAPTURE_T":"0.06","KP_FLIGHT_FEET":"1000","KD_FLIGHT_FEET":"63","WBCW_TRUNK_PUSH":"0.003","WBCW_MOMENTUM":"10","WBCW_CONTACT":"100","WBCW_TRUNK_LAND_VZ":"0","JUMP_H_NAT":"1","WBCW_CONTACT_REST_LAUNCH":"1","FLIGHT_FEET_CAPTURE":"0.55","FLIGHT_TRUNK":"0","FLIGHT_FEET_W":"10","WBCW_MOMENTUM_PUSH":"1","MU_TORSION":"0.02","TRUNK_RP_YAWED":"1"'
Z=',"JUMP_VX_TAB":"1.0:0.25,1.5:0.3,2.0:0.4","JUMP_LEAN_TAB":"1.0:0,1.5:0.01125,2.0:0.015"'
R=',"JUMP_REPEAT":"5","JUMP_REPEAT_RECENTRE":"1","JUMP_REPEAT_SOFT_CROUCH":"1","FLIGHT_FEET_WIDTH":"0.236","WBCW_TRUNK_LAND_T":"1.0","T":"24","FLIGHT_FEET_YAW":"1"'
LT='1.5|-0.6:-0.124,-0.5:-0.111,-0.4:-0.076,-0.3:-0.034,-0.299:0,0.299:0,0.3:0.032,0.4:0.074,0.5:0.110,0.6:0.139;2.0|-0.6:-0.144,-0.5:-0.112,-0.4:-0.091,-0.399:0,0.399:0,0.4:0.092,0.5:0.113,0.6:0.144'
cd $D
lt() { TAG=$1 SEED=$3 BIN=$B CONDS=1.5,2.0 N=16 W=1 EXTRA="{$X$H$Z$R,\"JUMP_LAT_TAB\":\"$LT\",\"JUMP_LAT_TARGET\":\"$2\"$4}" python3 jrand.py; }
q() { while [ $(jobs -rp | wc -l) -ge 16 ]; do wait -n; done; }
VK=',"KP_TRUNK_LAND":"60","KD_TRUNK_LAND":"150"'
VS=',"LAND_REF_SOLES":"0.5"'
VH=',"LAND_HOLD":"0.1"'
for k in K S H; do v=V$k; for t in 10:0.10 30:0.30 50:0.50 m30:-0.30 m50:-0.50; do
  q; lt L${k}T${t%%:*} ${t#*:} 27182 "${!v}" & q; lt L${k}U${t%%:*} ${t#*:} 16180 "${!v}" &
done; done
wait
echo "#### JB106_DONE"
