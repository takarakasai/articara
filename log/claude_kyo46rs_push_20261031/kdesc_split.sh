#!/bin/bash
# 降段 20 mm の全落ちの切り分け（2026-10-31）: kpc4 の 3 つの仕組みを分けて、家族の kdesc だけを 3 つの種で。
#   dm0 = レンチ MPC の適用だけ、dc3 = 適用＋MPC の着地（kpc4 − PUSH_REPLAN）、dpr = PUSH_REPLAN だけ。
#   kfamily.sh の kdesc と同じ指定（種は FAM_SEED のみ）。判定:
#   python3 log/claude_g1rand_20260925/family_judge.py --kyo --benches kdesc k3n:<N>n k3o:<N>o k3t:<N>t
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
L=log/claude_g1rand_20260925; B=$L/kyo46rs_walk_tu63; C=$L/kyo_base_v3.json; P=log/claude_kyo46rs_push_20261031
R=log/claude_g1rand_20260924/rand.py
one() { # NAME SEED
  X=$(cat $P/cand_${1%?}.extra)
  TAG=KFAM_$1_kdesc STAIR_N=1 STAIR_TREAD=0.110 BASE=$C BIN=$B MODE=stairs N=48 W=${W:-10} SEED=$2 START_X=0.40 DX=0.01 T=32 \
    EXTRA="{$X,\"SPAWN_Z_EXTRA\":\"0.005\"}" RISERS=0.000,-0.010,-0.020 python3 $R > $P/kds_$1.out 2>&1
}
for v in dm0 dc3 dpr; do
  one ${v}n 31415 & one ${v}o 27182 & one ${v}t 16180 & wait
  python3 $L/family_judge.py --kyo --benches kdesc k3n:${v}n k3o:${v}o k3t:${v}t > $P/kds_judge_$v.out 2>&1
  echo "#### KDS_DONE $v"
done
echo "#### KDS_ALL_DONE"
