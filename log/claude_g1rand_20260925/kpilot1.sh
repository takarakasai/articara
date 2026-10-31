#!/bin/bash
# kfamily.sh の下見（2026-10-31）: 基準 v2 で地形・センサ・振れ幅の広い個体・速度を N=12 で回し、回る規模を選ぶ。
set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
L=log/claude_g1rand_20260925; R=log/claude_g1rand_20260924/rand.py
BIN=$L/kyo46rs_walk_tu63; C=$L/kyo_base_v2.json
export SEED=31415 W=${W:-8} N=12 DX=0.01
F="MODE=flat FLAT_START_X=30 T=28"
# 平地の速度
env $F TAG=KPIL_flat BASE=$C BIN=$BIN EXTRA='{}' CONDS='VX=0.03,VX=0.06,VX=0.09' python3 $R | sed 's/^/flat /'
# 凹凸（模型に無い）5／10 mm、長さ 8 cm 前後、x 0.15〜0.60
for h in 0.005 0.010; do
  env $F TAG=KPIL_bump${h#0.0} BASE=$C BIN=$BIN EXTRA="{\"TERRAIN_BUMPS\":\"4,$h,0.08,0.15,0.60\"}" CONDS='T=28' python3 $R | sed "s/^/bump$h /"
done
# 既知の斜面 ±2／±3°、x 0.15 から 0.40 m
for d in 2 3 -2 -3; do
  X='"TERRAIN_SLOPE":"'$d',0.15,0.40"'; [ ${d:0:1} = - ] && X="$X,\"SPAWN_Z_EXTRA\":\"0.005\""
  env $F TAG=KPIL_slope$d BASE=$C BIN=$BIN EXTRA="{$X}" CONDS='T=28' python3 $R | sed "s/^/slope$d /"
  env $F TAG=KPIL_slopeP$d BASE=$C BIN=$BIN EXTRA="{$X,\"TERRAIN_PITCH\":\"1\"}" CONDS='T=28' python3 $R | sed "s/^/slopeP$d /"
done
# センサ
env $F TAG=KPIL_sense BASE=$C BIN=$BIN EXTRA='{}' CONDS='SENSE_DELAY_TICKS=4,SENSE_DELAY_TICKS=8,SENSE_NOISE_QD=0.1;SENSE_NOISE_V=0.03;SENSE_NOISE_W=0.03,SENSE_NOISE_Q=0.002,ACT_DELAY_TICKS=1,ACT_DELAY_TICKS=2' python3 $R | sed 's/^/sense /'
# 振れ幅の広い個体（押し 0 と 0.6 N·s × 4 方向）
( export MU_LO=0.40 MU_HI=0.90 JITTER=0.03 DX=0.05 PUSH_STAIR_X=30
  TAG=KPIL_w BASE=$C BIN=$BIN MODE=push EXTRA='{}' CONDS=0@0,0.6@0,0.6@90,0.6@180,0.6@270 python3 $R | sed 's/^/w /'
  export JITTER=0.02; TAG=KPIL_w2 BASE=$C BIN=$BIN MODE=push EXTRA='{}' CONDS=0@0,0.6@0,0.6@90,0.6@180,0.6@270 python3 $R | sed 's/^/w2 /' )
echo "#### KPILOT1_DONE"
