#!/bin/bash
# G1 で閉じた植物の漏れ 2 つを kyo46rs の v3 の上で（2026-10-31）: JOINT_Q_EST=1 と MU_ASSUMED=0.58、各 3 つの種。
# 0.58: kyo46rs の friction_margin 0.857（profile.rs、G1 と同じ）× 0.58 = 0.497 が、振る μ の下端 0.60 の 0.83 倍。
cd /home/takara/work/dp/articara
L=log/claude_g1rand_20260925; B=$L/kyo46rs_walk_tu63; C=$L/kyo_base_v3.json
run() { FAM_SEED=$2 FAM_SEED_HOLD=$3 bash $L/kfamily.sh $1 $C $B "$4" > $L/kfam_$1.out 2>&1; }
for s in n:31415:27182 o:27182:31415 t:16180:14142; do IFS=: read k a b <<< "$s"
  run kjq$k $a $b '"JOINT_Q_EST":"1"' & run kmu$k $a $b '"MU_ASSUMED":"0.58"' & wait
done
echo "#### KFAM_RUN2_DONE"
