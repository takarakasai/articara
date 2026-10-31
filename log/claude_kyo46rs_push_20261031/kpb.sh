#!/bin/bash
# 押しの小ベンチ（2026-10-31）。使い方: kpb.sh NAME 'EXTRA_JSON_BODY' [BIN]
#   kfamily.sh の kp と同じ個体（種 FAM_SEED、既定 31415、N=48、T=20、段は x=30）で、条件は CONDS
#   （既定 0.9@0,0.9@90,0.9@270）。基準は KFAM_k3n_kp（v3、tu63）と個体ごとに対応する。
#   出力は log/claude_g1rand_20260924/KPB_<NAME>_kp/。比べるのは kbreak.py KFAM_k3n_kp KPB_<NAME>_kp。
set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
NAME=$1; X=${2:-}; BIN=${3:-log/claude_g1rand_20260925/kyo46rs_walk_tu63}
C=log/claude_g1rand_20260925/kyo_base_v3.json
export PUSH_STAIR_X=30
TAG=KPB_${NAME}_kp BASE=$C BIN=$BIN MODE=push N=${N:-48} W=${W:-12} SEED=${FAM_SEED:-31415} EXTRA="{$X}" \
  CONDS=${CONDS:-0.9@0,0.9@90,0.9@270} python3 log/claude_g1rand_20260924/rand.py
python3 log/claude_kyo46rs_push_20261031/kbreak.py ${BASEB:-KFAM_k3n_kp} KPB_${NAME}_kp
echo "#### KPB_DONE $NAME"
