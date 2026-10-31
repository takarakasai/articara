#!/bin/bash
# 凹凸 5／10 mm とセンサの損の切り分け（2026-11-01）: kpc4 の 3 つの仕組みを分けて、家族の kbump05・kbump10・ksense を 3 つの種で。
#   dm0 = レンチ MPC の適用だけ、dc3 = 適用＋MPC の着地、dpr = PUSH_REPLAN だけ（cand_*.extra）。kfamily.sh と同じ指定。
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
L=log/claude_g1rand_20260925; B=$L/kyo46rs_walk_tu63; C=$L/kyo_base_v3.json; P=log/claude_kyo46rs_push_20261031
R=log/claude_g1rand_20260924/rand.py
one() { # NAME SEED
  X=$(cat $P/cand_${1%?}.extra); W=${W:-10}
  for h in 05 10; do
    env MODE=flat N=48 W=$W SEED=$2 T=28 DX=0.01 FLAT_START_X=30 TAG=KFAM_$1_kbump$h BASE=$C BIN=$B \
      EXTRA="{$X,\"TERRAIN_BUMPS\":\"4,0.0$h,0.08,0.15,0.60\"}" CONDS='T=28' python3 $R
  done
  env MODE=flat N=24 W=$W SEED=$2 T=28 DX=0.01 FLAT_START_X=30 TAG=KFAM_$1_ksense BASE=$C BIN=$B EXTRA="{$X}" \
    CONDS='SENSE_DELAY_TICKS=2,SENSE_DELAY_TICKS=4,SENSE_DELAY_TICKS=6,SENSE_NOISE_QD=0.05;SENSE_NOISE_V=0.015;SENSE_NOISE_W=0.015,SENSE_NOISE_QD=0.1;SENSE_NOISE_V=0.03;SENSE_NOISE_W=0.03,SENSE_NOISE_Q=0.002,ACT_DELAY_TICKS=2' python3 $R
}
for v in dm0 dc3 dpr; do
  one ${v}n 31415 > $P/fls_${v}n.out 2>&1 & one ${v}o 27182 > $P/fls_${v}o.out 2>&1 & one ${v}t 16180 > $P/fls_${v}t.out 2>&1 & wait
  python3 $L/family_judge.py --kyo --benches kbump05,kbump10,ksense k3n:${v}n k3o:${v}o k3t:${v}t > $P/fls_judge_$v.out 2>&1
  echo "#### FLS_DONE $v"
done
echo "#### FLS_ALL_DONE"
