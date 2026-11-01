#!/bin/bash
# kyo46rs の斜面 ±2°（kpilot1.sh と同じ置き方、N=12、種 31415）: v3・v4（鉛直の門、tu67）・v4（斜面の門、ml10）。2026-11-01。
set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
L=log/claude_g1rand_20260925; R=log/claude_g1rand_20260924/rand.py
export SEED=31415 W=8 N=12 DX=0.01
F="MODE=flat FLAT_START_X=30 T=28"
V3=$L/kyo_base_v3.json; V4=log/claude_kyo46rs_base_20261101v4/controls.json
for arm in "v3 $V3 $L/kyo46rs_walk_tu67" "v4 $V4 $L/kyo46rs_walk_tu67" "v4f $V4 $L/kyo46rs_walk_ml10"; do
  set -- $arm; nm=$1; C=$2; BIN=$3
  for d in 2 -2; do
    X='"TERRAIN_SLOPE":"'$d',0.15,0.40"'; [ ${d:0:1} = - ] && X="$X,\"SPAWN_Z_EXTRA\":\"0.005\""
    env $F TAG=KSL_${nm}_slope$d BASE=$C BIN=$BIN EXTRA="{$X}" CONDS='T=28' python3 $R | sed "s/^/$nm slope$d /"
    env $F TAG=KSL_${nm}_slopeP$d BASE=$C BIN=$BIN EXTRA="{$X,\"TERRAIN_PITCH\":\"1\"}" CONDS='T=28' python3 $R | sed "s/^/$nm slopeP$d /"
  done
done
echo "#### KSLOPE_DONE"
