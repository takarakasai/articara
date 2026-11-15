#!/bin/bash
# 連続の片足跳び（jb117、§52）: jb116 の HR5（HP＋JUMP_REPEAT 4、5 回跳び）に、空中で足のヨーを離地のヨーに保つ FLIGHT_FEET_YAW（HY5*）、さらに遊脚の高さのゲイン KP_HOP_FOOT_Z 5000／KD_HOP_FOOT 100（HYZ5*）、横の捕獲 HOP_FLIGHT_Y_CAPTURE 0.8（HYC5*）。初速 1.5、種 27182 の左右、N=16、j92。
D=/home/takara/work/dp/articara/log/claude_g1jump_20261021; B=$D/g1_flight_probe_j92
eval "$(sed -n '/^X=/p;/^H=/p;/^HP=/p' $D/jb112.sh)"
cd $D
hr() { # TAG SEED LEG EXTRA
  TAG=$1 SEED=$2 BIN=$B CONDS=1.5 N=16 W=2 EXTRA="{$X$H$HP,\"HOP_LEG\":\"$3\",\"JUMP_REPEAT\":\"4\",\"T\":\"20\"$4}" python3 jrand.py
}
Y=',"FLIGHT_FEET_YAW":"1"'
hr HY5L 27182 L "$Y" &
hr HY5R 27182 R "$Y" &
hr HYZ5L 27182 L "$Y"',"KP_HOP_FOOT_Z":"5000","KD_HOP_FOOT":"100"' &
hr HYZ5R 27182 R "$Y"',"KP_HOP_FOOT_Z":"5000","KD_HOP_FOOT":"100"' &
hr HYC5L 27182 L "$Y"',"HOP_FLIGHT_Y_CAPTURE":"0.8"' &
hr HYC5R 27182 R "$Y"',"HOP_FLIGHT_Y_CAPTURE":"0.8"' &
wait
echo "#### JB117_DONE"
