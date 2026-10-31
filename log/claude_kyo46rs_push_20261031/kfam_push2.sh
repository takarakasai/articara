#!/bin/bash
# kfam_push.sh と同じ（3 つの種の家族と判定）、バイナリを BIN で渡す版（2026-10-31）。使い方: BIN=... kfam_push2.sh NAME 'EXTRA_JSON_BODY'
cd /home/takara/work/dp/articara
L=log/claude_g1rand_20260925; B=${BIN:?}; C=$L/kyo_base_v3.json; P=log/claude_kyo46rs_push_20261031
NAME=$1; X=$2
run() { FAM_SEED=$2 FAM_SEED_HOLD=$3 W=${W:-12} bash $L/kfamily.sh $1 $C $B "$X" > $P/kfam_$1.out 2>&1; }
run ${NAME}n 31415 27182 & run ${NAME}o 27182 31415 & wait
W=20 run ${NAME}t 16180 14142
python3 $L/family_judge.py --kyo k3n:${NAME}n k3o:${NAME}o k3t:${NAME}t > $P/kfam_judge_${NAME}.out 2>&1
echo "#### KFAM_PUSH_DONE $NAME"
