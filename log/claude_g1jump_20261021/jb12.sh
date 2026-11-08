#!/bin/bash
# 段への跳び乗りを間合いを揃えて（jb12）: jb9 の JM に JUMP_BOX_CROUCH_W（跳び直しの後の台への跳躍のかがみ・溜めの接触の重み、既定は WBCW_CONTACT_REST 0.1）1・3・10、j64。0.1 では接触の段に入っている圧力中心の枠も柔らかく、計画が爪先の縁を越えて足が傾き、重心が後ろへ揺り戻る（DIAG_CROUCH）。jb9: jb8 の台への跳躍は、着地の足首の原点が台の縁 +0.06 m 未満だと倒れる（以上の 80 本は全部立った）。跳び直しの後は離地の vx が 0.48〜0.55（1 回目は 0.62）で届かない。JL: 間合い 0.07（許容 0.02）、JM: 間合い 0.10 のまま JUMP_BOX_S 1.2。台 0.20・0.30・0.40・0.60、N=8、種 16180、j61。jb8: jb7 の FLIGHT_FEET_TUCK 0.05 を JUMP_BOX_TUCK 0.05（台への跳躍だけ）に、j61。jb7 は全部の跳躍で足を上げたので、間合いを詰める後ろへの小さな跳躍が倒れた（0.20 の 2.0 で 15/16 が 1 回目で）。jb7: JUMP_BOX_GAP 0.10（爪先から台の前の面まで、許容 JUMP_BOX_GAP_TOL 0.025）に揃うまで平地で跳んで（JUMP_FWD_TAB は jf3 の FT）、揃ったら倍率 1 で台へ。台に載ったら止める。jb6 の設定（TB6・FLIGHT_FEET_TUCK 0.05）に jf3 の R（連続の設定）。台の高さ 5 cm、前の面 0.20・0.25・0.30・0.40・0.60。1.5・2.0、N=16、種 16180、j60。立った姿勢の爪先は x 0.133 なので、0.25 の台は間合い 0.117（jb6 で 14・14 of 16）。
D=/home/takara/work/dp/articara/log/claude_g1jump_20261021; B=$D/g1_flight_probe_j64
X='"JUMP_ABORT":"1","JUMP_PUSH_R":"0.3","QD_CLAMP":"40","DIAG_TD":"1","FLIGHT_FEET_FLAT":"1","DIAG_HX":"1","RELEASE_FIRST_OFF":"1","WBCW_CONTACT_REST":"0.1","WBCW_TRUNK_LAND":"0.03","FALL_TILT":"1.5","LAND_REF_FOLLOW":"2","AIR_ABORT_T":"0.08"'
H=',"FLIGHT_FEET_CAPTURE_T":"0.06","KP_FLIGHT_FEET":"1000","KD_FLIGHT_FEET":"63","WBCW_TRUNK_PUSH":"0.003","WBCW_MOMENTUM":"10","WBCW_CONTACT":"100","WBCW_TRUNK_LAND_VZ":"0","JUMP_H_NAT":"1","WBCW_CONTACT_REST_LAUNCH":"1","FLIGHT_FEET_CAPTURE":"0.55","FLIGHT_TRUNK":"0","FLIGHT_FEET_W":"10","WBCW_MOMENTUM_PUSH":"1","MU_TORSION":"0.02","TRUNK_RP_YAWED":"1"'
TB=',"JUMP_VX_TAB":"1.0:1.0,1.5:1.2,2.0:1.6","JUMP_LEAN_TAB":"1.0:0,1.5:0.045,2.0:0.06"'
R=',"JUMP_REPEAT":"5","JUMP_REPEAT_RECENTRE":"1","JUMP_REPEAT_SOFT_CROUCH":"1","FLIGHT_FEET_WIDTH":"0.236","WBCW_TRUNK_LAND_T":"1.0","T":"24","FLIGHT_FEET_YAW":"1"'
FT='1.5|0:-0.112,0.25:-0.026,0.4:0.021,0.5:0.068,0.55:0.090,0.6:0.124,0.65:0.230,0.7:0.265,0.85:0.290,1.0:0.334,1.1:0.347;2.0|0:-0.148,0.25:-0.026,0.4:0.038,0.5:0.104,0.55:0.131,0.6:0.161,0.65:0.216,0.7:0.249,0.75:0.271,0.85:0.325,1.0:0.362'
cd $D
run() { TAG=$1 SEED=16180 BIN=$B CONDS=1.5,2.0 N=8 W=1 EXTRA="{$X$H$TB$R,\"JUMP_FWD_TAB\":\"$FT\",\"JUMP_BOX_TUCK\":\"0.05\",\"JUMP_BOX\":\"$2\",$3}" python3 jrand.py; }
M='"JUMP_BOX_GAP":"0.10","JUMP_BOX_GAP_TOL":"0.025","JUMP_BOX_S":"1.2"'
for w in 1 3 10; do for b in 20 30 40 60; do run JQ${w}_$b 0.$b,0.05 "$M,\"JUMP_BOX_CROUCH_W\":\"$w\"" & done; done
wait
for w in 1 3 10; do python3 jbsumm.py JQ${w}_20 JQ${w}_30 JQ${w}_40 JQ${w}_60; python3 jbjump.py JQ${w}_20 JQ${w}_30 JQ${w}_40 JQ${w}_60; done
echo "#### JB12_DONE"
