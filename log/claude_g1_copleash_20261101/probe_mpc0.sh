#!/bin/bash
# comb_bump10 の損の大きい 2 条件（前 11、90° 7）で、MPC の箱は縛らない版（COP_LEASH_MPC=0）。2026-11-01。
cd /home/takara/work/dp/articara; export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
C=log/claude_g1champion_20261031v621/controls.json; BIN=log/claude_g1rand_20260925/kyo46rs_walk_tu67
PUSH_STAIR_X=30 TAG=FAM_gL5w_comb_bump10 BASE=$C BIN=$BIN MODE=push N=48 W=8 SEED=31415 \
  EXTRA='{"COP_LEASH":"0.005","COP_LEASH_TILT":"0.02","TERRAIN_BUMPS":"6,0.010,0.25,0.8,1.8"}' CONDS=11@0,7@90 python3 log/claude_g1rand_20260924/rand.py
echo "#### PROBE_DONE"
