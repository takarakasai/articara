#!/bin/bash
# 押しの回復の候補を kyo46rs の家族（kfamily.sh）で 3 つの種に（2026-10-31）。使い方: kfam_push.sh NAME 'EXTRA_JSON_BODY'
#   NAME に n／o／t を足した 3 つの家族（KFAM_<NAME>{n,o,t}_*）。基準は KFAM_k3{n,o,t}_*（v3、tu63）。
#   n と o を同時に、t を 1 つで回す（同時に 2 つまで）。判定:
#   python3 log/claude_g1rand_20260925/family_judge.py --kyo k3n:<NAME>n k3o:<NAME>o k3t:<NAME>t
cd /home/takara/work/dp/articara
L=log/claude_g1rand_20260925; B=$L/kyo46rs_walk_tu63; C=$L/kyo_base_v3.json; P=log/claude_kyo46rs_push_20261031
NAME=$1; X=$2
run() { FAM_SEED=$2 FAM_SEED_HOLD=$3 W=${W:-16} bash $L/kfamily.sh $1 $C $B "$X" > $P/kfam_$1.out 2>&1; }
run ${NAME}n 31415 27182 & run ${NAME}o 27182 31415 & wait
W=24 run ${NAME}t 16180 14142
python3 $L/family_judge.py --kyo k3n:${NAME}n k3o:${NAME}o k3t:${NAME}t > $P/kfam_judge_${NAME}.out 2>&1
echo "#### KFAM_PUSH_DONE $NAME"
