#!/bin/bash
# 横の位置を狙う跳躍（jb105、§49）: JUMP_LAT_TARGET（j86）、表は jb102〜104 の中央値（種 31415）。小さい |VY| は 2.0 で倒れる（0.2〜0.3 で 8〜10/16）ので、残りが小さくても 2.0 は |VY| 0.4、1.5 は 0.3 から（表に段を入れる）。土台は TB6 の表の 0.25 倍（前後の移動がほぼ 0）＋FY の連続の設定。種 27182／16180 N=16。
D=/home/takara/work/dp/articara/log/claude_g1jump_20261021; B=$D/g1_flight_probe_j86
X='"JUMP_ABORT":"1","JUMP_PUSH_R":"0.3","QD_CLAMP":"40","DIAG_TD":"1","FLIGHT_FEET_FLAT":"1","DIAG_HX":"1","RELEASE_FIRST_OFF":"1","WBCW_CONTACT_REST":"0.1","WBCW_TRUNK_LAND":"0.03","FALL_TILT":"1.5","LAND_REF_FOLLOW":"2","AIR_ABORT_T":"0.08"'
H=',"FLIGHT_FEET_CAPTURE_T":"0.06","KP_FLIGHT_FEET":"1000","KD_FLIGHT_FEET":"63","WBCW_TRUNK_PUSH":"0.003","WBCW_MOMENTUM":"10","WBCW_CONTACT":"100","WBCW_TRUNK_LAND_VZ":"0","JUMP_H_NAT":"1","WBCW_CONTACT_REST_LAUNCH":"1","FLIGHT_FEET_CAPTURE":"0.55","FLIGHT_TRUNK":"0","FLIGHT_FEET_W":"10","WBCW_MOMENTUM_PUSH":"1","MU_TORSION":"0.02","TRUNK_RP_YAWED":"1"'
Z=',"JUMP_VX_TAB":"1.0:0.25,1.5:0.3,2.0:0.4","JUMP_LEAN_TAB":"1.0:0,1.5:0.01125,2.0:0.015"'
R=',"JUMP_REPEAT":"5","JUMP_REPEAT_RECENTRE":"1","JUMP_REPEAT_SOFT_CROUCH":"1","FLIGHT_FEET_WIDTH":"0.236","WBCW_TRUNK_LAND_T":"1.0","T":"24","FLIGHT_FEET_YAW":"1"'
LT='1.5|-0.6:-0.124,-0.5:-0.111,-0.4:-0.076,-0.3:-0.034,-0.299:0,0.299:0,0.3:0.032,0.4:0.074,0.5:0.110,0.6:0.139;2.0|-0.6:-0.144,-0.5:-0.112,-0.4:-0.091,-0.399:0,0.399:0,0.4:0.092,0.5:0.113,0.6:0.144'
cd $D
lt() { TAG=$1 SEED=$3 BIN=$B CONDS=1.5,2.0 N=16 W=1 EXTRA="{$X$H$Z$R,\"JUMP_LAT_TAB\":\"$LT\",\"JUMP_LAT_TARGET\":\"$2\"$4}" python3 jrand.py; }
q() { while [ $(jobs -rp | wc -l) -ge 16 ]; do wait -n; done; }
for t in 10:0.10 30:0.30 50:0.50 m30:-0.30 m50:-0.50; do
  q; lt LT${t%%:*} ${t#*:} 27182 & q; lt LU${t%%:*} ${t#*:} 16180 &
done
wait
echo "#### JB105_DONE"
