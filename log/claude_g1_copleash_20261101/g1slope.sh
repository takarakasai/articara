#!/bin/bash
# 斜面の 3 ベンチ（family.sh と同じ定義、種 n）だけ。使い方: g1slope.sh NAME BIN 'EXTRA_JSON_BODY'。2026-11-01。
set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
NAME=$1; BIN=$2; X=${3:-}
C=log/claude_g1champion_20261031v621/controls.json
J() { if [ -n "$X" ]; then echo "{$X${1:+,$1}}"; else echo "{${1:-}}"; fi; }
S1=31415; R=log/claude_g1rand_20260924/rand.py; W=${W:-24}; P4=11@0,11@180,7@90,5@270
F="MODE=flat N=48 W=$W SEED=$S1 T=40 FLAT_MIN_STEPS=24 FLAT_START_X=30"
t() { echo "FAM_${NAME}_$1"; }
env $F TAG=$(t slope5) BASE=$C BIN=$BIN EXTRA="$(J '"TERRAIN_SLOPE":"5,1.0,4.0","STEP_DZ_MIN":"0.08","STEP_DZ_SWING_MIN":"0.08","TERRAIN_PITCH":"1"')" CONDS="T=40" python3 $R
env $F TAG=$(t slope-5) BASE=$C BIN=$BIN EXTRA="$(J '"TERRAIN_SLOPE":"-5,1.0,4.0","STEP_DZ_MIN":"0.08","STEP_DZ_SWING_MIN":"0.08","TERRAIN_PITCH":"1","SPAWN_Z_EXTRA":"0.005"')" CONDS="T=40" python3 $R
PUSH_STAIR_X=30 TAG=$(t comb_slope3) BASE=$C BIN=$BIN MODE=push N=48 W=$W SEED=$S1 EXTRA="$(J '"TERRAIN_SLOPE":"3,1.0,4.0","STEP_DZ_MIN":"0.03","TERRAIN_PITCH":"1"')" CONDS=$P4 python3 $R
echo "#### G1SLOPE_DONE $NAME"
