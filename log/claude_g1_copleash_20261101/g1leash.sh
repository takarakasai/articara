#!/bin/bash
# G1 v6.21 に COP_LEASH（kyo46rs v4 の候補 kl9 の仕組み）を足す選別。家族 family.sh の地形・斜面の 5 ベンチを
# そのままの定義で回す。使い方: g1leash.sh NAME 'EXTRA_JSON_BODY' [MIX]
#   MIX=1 なら凹凸 2 ベンチだけを SEED_MIX_TERRAIN=1（家族の種で地形を振り直す）で、TAG は FAM_<NAME>m_*。
# 種は家族の n（FAM_SEED=31415、FAM_SEED_HOLD=27182）。基準は FAM_v621n_*（MIX=1 は FAM_v621Lm_* を同じ脚本で）。2026-11-01。
set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
NAME=$1; X=${2:-}; MIX=${3:-0}
C=log/claude_g1champion_20261031v621/controls.json; BIN=log/claude_g1rand_20260925/kyo46rs_walk_tu67
J() { if [ -n "$X" ]; then echo "{$X${1:+,$1}}"; else echo "{${1:-}}"; fi; }
S1=31415; R=log/claude_g1rand_20260924/rand.py; W=${W:-24}; P4=11@0,11@180,7@90,5@270
F="MODE=flat N=48 W=$W SEED=$S1 T=40 FLAT_MIN_STEPS=24 FLAT_START_X=30"
if [ "$MIX" = 1 ]; then
  export SEED_MIX_TERRAIN=1; t() { echo "FAM_${NAME}m_$1"; }
  env $F TAG=$(t bump10) BASE=$C BIN=$BIN EXTRA="$(J '"TERRAIN_BUMPS":"6,0.010,0.25,1.0,4.0"')" CONDS="T=40" python3 $R
  PUSH_STAIR_X=30 TAG=$(t comb_bump10) BASE=$C BIN=$BIN MODE=push N=48 W=$W SEED=$S1 EXTRA="$(J '"TERRAIN_BUMPS":"6,0.010,0.25,0.8,1.8"')" CONDS=$P4 python3 $R
else
  t() { echo "FAM_${NAME}_$1"; }
  env $F TAG=$(t bump10) BASE=$C BIN=$BIN EXTRA="$(J '"TERRAIN_BUMPS":"6,0.010,0.25,1.0,4.0"')" CONDS="T=40" python3 $R
  env $F TAG=$(t slope5) BASE=$C BIN=$BIN EXTRA="$(J '"TERRAIN_SLOPE":"5,1.0,4.0","STEP_DZ_MIN":"0.08","STEP_DZ_SWING_MIN":"0.08","TERRAIN_PITCH":"1"')" CONDS="T=40" python3 $R
  env $F TAG=$(t slope-5) BASE=$C BIN=$BIN EXTRA="$(J '"TERRAIN_SLOPE":"-5,1.0,4.0","STEP_DZ_MIN":"0.08","STEP_DZ_SWING_MIN":"0.08","TERRAIN_PITCH":"1","SPAWN_Z_EXTRA":"0.005"')" CONDS="T=40" python3 $R
  ( export PUSH_STAIR_X=30
    TAG=$(t comb_slope3) BASE=$C BIN=$BIN MODE=push N=48 W=$W SEED=$S1 EXTRA="$(J '"TERRAIN_SLOPE":"3,1.0,4.0","STEP_DZ_MIN":"0.03","TERRAIN_PITCH":"1"')" CONDS=$P4 python3 $R
    TAG=$(t comb_bump10) BASE=$C BIN=$BIN MODE=push N=48 W=$W SEED=$S1 EXTRA="$(J '"TERRAIN_BUMPS":"6,0.010,0.25,0.8,1.8"')" CONDS=$P4 python3 $R )
fi
echo "#### G1LEASH_DONE $NAME $MIX"
