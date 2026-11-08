#!/bin/bash
# 段への跳び乗り（jb5）: 対照。台の上の平地（JUMP_BOX -2,0.05,4 で足の下から台、出現も台の上）で TB6 の 1 回跳び。箱と箱の接触が着地の後を悪くするか。1.5・2.0、N=8、種 27182、j59。
D=/home/takara/work/dp/articara/log/claude_g1jump_20261021; B=$D/g1_flight_probe_j59
X='"JUMP_ABORT":"1","JUMP_PUSH_R":"0.3","QD_CLAMP":"40","DIAG_TD":"1","FLIGHT_FEET_FLAT":"1","DIAG_HX":"1","RELEASE_FIRST_OFF":"1","WBCW_CONTACT_REST":"0.1","WBCW_TRUNK_LAND":"0.03","FALL_TILT":"1.5","LAND_REF_FOLLOW":"2","AIR_ABORT_T":"0.08"'
H=',"FLIGHT_FEET_CAPTURE_T":"0.06","KP_FLIGHT_FEET":"1000","KD_FLIGHT_FEET":"63","WBCW_TRUNK_PUSH":"0.003","WBCW_MOMENTUM":"10","WBCW_CONTACT":"100","WBCW_TRUNK_LAND_VZ":"0","JUMP_H_NAT":"1","WBCW_CONTACT_REST_LAUNCH":"1","FLIGHT_FEET_CAPTURE":"0.55","FLIGHT_TRUNK":"0","FLIGHT_FEET_W":"10","WBCW_MOMENTUM_PUSH":"1","MU_TORSION":"0.02","TRUNK_RP_YAWED":"1"'
TB=',"JUMP_VX_TAB":"1.0:1.0,1.5:1.2,2.0:1.6","JUMP_LEAN_TAB":"1.0:0,1.5:0.045,2.0:0.06"'
cd $D
run() { TAG=$1 SEED=27182 BIN=$B CONDS=1.5,2.0 N=8 W=4 EXTRA="{$X$H$TB,\"DIAG_FWD_FEET\":\"1\",\"JUMP_BOX\":\"-2,0.05,4\"}" python3 jrand.py; }
run JE00 x &
wait
python3 fsumm.py JE00
echo "#### JB5_DONE"
