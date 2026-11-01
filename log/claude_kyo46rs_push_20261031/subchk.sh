#!/bin/bash
# subchk.sh NAME EXTRA_FILE: 家族の一部（種 n）で候補を見る（2026-11-01、ml5）。
#   kp の 0.9 N·s 4 方向、kbump05、kdesc r-0.020、ksense 雑音（大）。結果は KFAM_<NAME>n_* （家族と同じ名前の形）。
set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
L=log/claude_g1rand_20260925; C=$L/kyo_base_v3.json; B=${BIN:-$L/kyo46rs_walk_ml5}; N0=$1; W=${W:-12}
R=log/claude_g1rand_20260924/rand.py; X=$(cat $2)
( export PUSH_STAIR_X=30
  TAG=KFAM_${N0}n_kp BASE=$C BIN=$B MODE=push N=48 W=$W SEED=31415 EXTRA="{$X}" CONDS=0.9@0,0.9@90,0.9@180,0.9@270 python3 $R )
TAG=KFAM_${N0}n_kdesc STAIR_N=1 STAIR_TREAD=0.110 BASE=$C BIN=$B MODE=stairs N=48 W=$W SEED=31415 START_X=0.40 DX=0.01 T=32 \
  EXTRA="{$X,\"SPAWN_Z_EXTRA\":\"0.005\"}" RISERS=-0.020 python3 $R
env MODE=flat N=48 W=$W SEED=31415 T=28 DX=0.01 FLAT_START_X=30 TAG=KFAM_${N0}n_kbump05 BASE=$C BIN=$B \
  EXTRA="{$X,\"TERRAIN_BUMPS\":\"4,0.005,0.08,0.15,0.60\"}" CONDS='T=28' python3 $R
env MODE=flat N=24 W=$W SEED=31415 T=28 DX=0.01 FLAT_START_X=30 TAG=KFAM_${N0}n_ksense BASE=$C BIN=$B EXTRA="{$X}" \
  CONDS='SENSE_NOISE_QD=0.1;SENSE_NOISE_V=0.03;SENSE_NOISE_W=0.03' python3 $R
echo "#### SUBCHK_DONE $N0"
