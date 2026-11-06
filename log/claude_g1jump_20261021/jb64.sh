#!/bin/bash
# R10W（3 回跳び）の 1.0 の 48 本を 1 刻みの記録（TRAJ_CSV）つきで回し直す。押しの間の接触の入れ替わりと胴の傾きを成否と比べる。j41（未設定で R10W とビット同一）。
D=/home/takara/work/dp/articara/log/claude_g1jump_20261021; O=/data/claude_g1jump_trace/r10w_v1.0
cd /home/takara/work/dp/articara; source ./setup-mujoco.sh >/dev/null 2>&1; cd $D
ls -d runs/R10W/v1.0/m* | xargs -P 8 -I{} sh -c 'm=$(basename {}); ./one.sh '$O'/$m.log R10W/v1.0/$m TRAJ_CSV='$O'/$m.csv'
echo "#### JB64_DONE"
