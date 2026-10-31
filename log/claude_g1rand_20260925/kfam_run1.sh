#!/bin/bash
# v2 と v3 の家族を 3 つの種で（2026-10-31）。同時に 2 つまで。
cd /home/takara/work/dp/articara
L=log/claude_g1rand_20260925; B=$L/kyo46rs_walk_tu63
run() { FAM_SEED=$2 FAM_SEED_HOLD=$3 bash $L/kfamily.sh $1 $4 $B "$5" > $L/kfam_$1.out 2>&1; }
for s in n:31415:27182 o:27182:31415 t:16180:14142; do IFS=: read k a b <<< "$s"
  run k2$k $a $b $L/kyo_base_v2.json '' & run k3$k $a $b $L/kyo_base_v3.json '' & wait
done
echo "#### KFAM_RUN1_DONE"
