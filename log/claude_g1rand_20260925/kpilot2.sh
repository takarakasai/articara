#!/bin/bash
# kfamily.sh の下見 2（2026-10-31）: 降段の T の打ち切り。KV3_desc_v3（T=28、種 20260924）を T=32 で回し直し、
# 28 s の後に落ちる個体が無いかを見る（計画 N_STEPS=40 は約 34 s で尽きる）。
set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
L=log/claude_g1rand_20260925; R=log/claude_g1rand_20260924/rand.py
MODE=stairs START_X=0.40 STAIR_TREAD=0.110 T=32 N=24 W=16 DX=0.01 SEED=20260924 RISERS=0.000,-0.010,-0.020 \
  TAG=KPIL_desc_v3_T32 BASE=$L/kyo_desc1_v3.json BIN=$L/kyo46rs_walk_tu63 EXTRA='{}' python3 $R | sed 's/^/desc v3 T32 /'
MODE=stairs START_X=0.40 STAIR_TREAD=0.110 T=32 N=24 W=16 DX=0.01 SEED=20260924 RISERS=0.000,-0.010 \
  TAG=KPIL_desc_v2_T32 BASE=$L/kyo_desc1_v2.json BIN=$L/kyo46rs_walk_tu63 EXTRA='{}' python3 $R | sed 's/^/desc v2 T32 /'
echo "#### KPILOT2_DONE"
