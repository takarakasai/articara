#!/bin/bash
# 切り替え後のバイナリ（misa-locomotion を git の rev で読む）で kpc4L の家族の一部が再現するかを見る（2026-11-01）。
# 使い方: BIN=... ml5_check.sh TAGNAME。種 n（31415）の kp 0.9@0／0.9@90、kdesc r-0.020、ksense 雑音（大）を回す。
set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
L=log/claude_g1rand_20260925; C=$L/kyo_base_v3.json; B=${BIN:?}; N0=$1; W=${W:-12}
R=log/claude_g1rand_20260924/rand.py; X=$(cat log/claude_kyo46rs_push_20261031/cand_kpc4L.extra)
( export PUSH_STAIR_X=30
  TAG=KFAM_${N0}_kp BASE=$C BIN=$B MODE=push N=48 W=$W SEED=31415 EXTRA="{$X}" CONDS=0.9@0,0.9@90 python3 $R )
TAG=KFAM_${N0}_kdesc STAIR_N=1 STAIR_TREAD=0.110 BASE=$C BIN=$B MODE=stairs N=48 W=$W SEED=31415 START_X=0.40 DX=0.01 T=32 \
  EXTRA="{$X,\"SPAWN_Z_EXTRA\":\"0.005\"}" RISERS=-0.020 python3 $R
env MODE=flat N=24 W=$W SEED=31415 T=28 DX=0.01 FLAT_START_X=30 TAG=KFAM_${N0}_ksense BASE=$C BIN=$B EXTRA="{$X}" \
  CONDS='SENSE_NOISE_QD=0.1;SENSE_NOISE_V=0.03;SENSE_NOISE_W=0.03' python3 $R
echo "#### ML5_CHECK_DONE"
