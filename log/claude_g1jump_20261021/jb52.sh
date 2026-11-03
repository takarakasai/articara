#!/bin/bash
# 押しの間だけ角運動量のタスクの重みを下げる WBCW_MOMENTUM_PUSH（j34、未設定でビット同一）を電池で（2026-11-03、ユーザ選択「1」）。
# 前出し 6 cm の弱い押し（m28・m31）は単発で 10→3／1／0.3 にすると離地 vz 0.63→1.2〜1.4 で着地した。
cd /home/takara/work/dp/articara
source ./setup-mujoco.sh >/dev/null
D=log/claude_g1jump_20261021; B=$D/g1_flight_probe_j34
X='"JUMP_ABORT":"1","JUMP_PUSH_R":"0.3","QD_CLAMP":"40","DIAG_TD":"1","FLIGHT_FEET_FLAT":"1","DIAG_HX":"1","RELEASE_FIRST_OFF":"1","WBCW_CONTACT_REST":"0.1","WBCW_TRUNK_LAND":"0.03","FALL_TILT":"1.5","LAND_REF_FOLLOW":"2","AIR_ABORT_T":"0.08"'
H=',"FLIGHT_FEET_CAPTURE_T":"0.06","KP_FLIGHT_FEET":"1000","KD_FLIGHT_FEET":"63","WBCW_TRUNK_PUSH":"0.003","WBCW_MOMENTUM":"10","WBCW_CONTACT":"100","WBCW_TRUNK_LAND_VZ":"0","JUMP_H_NAT":"1","WBCW_CONTACT_REST_LAUNCH":"1","FLIGHT_FEET_CAPTURE":"0.55","JUMP_VX_TAB":"1.0:1.0,1.5:1.2,2.0:1.6","FLIGHT_TRUNK":"0","FLIGHT_FEET_W":"10"'
TB4="$H"',"JUMP_LEAN_TAB":"1.0:0,1.5:0.045,2.0:0.045"'
L6="$H"',"JUMP_LEAN_TAB":"2.0:0.06"'
# 名前 高さ 構成 押しの重み 種 並び
J=("L6MP3 2.0 L6 3 31415 1" "L6MP3s2 2.0 L6 3 27182 1" "L6MP1 2.0 L6 1 31415 1" "L6MP1s2 2.0 L6 1 27182 1"
   "TB4MP3 1.0,1.5,2.0 TB4 3 31415 3" "TB4MP3s2 1.0,1.5,2.0 TB4 3 27182 3" "TB4MP1 1.0,1.5,2.0 TB4 1 31415 3" "TB4MP1s2 1.0,1.5,2.0 TB4 1 27182 3")
for j in "${J[@]}"; do set -- $j
  E=${!3}
  TAG=$1 SEED=$5 BIN=$B CONDS=$2 N=48 W=$6 EXTRA="{$X$E,\"WBCW_MOMENTUM_PUSH\":\"$4\"}" python3 $D/jrand.py &
done
wait
echo "#### JB52_DONE"
