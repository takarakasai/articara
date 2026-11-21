#!/bin/bash
# wr02.sh OUTDIR BIN [K=V ...]: wr01.sh (the 11-21 best running gains, strides 0.06 / 0.08 / 0.10,
# INIT_SEED 1..16) with extra overrides appended. Prints survivors per stride.
# Env SEEDS="a b" (default "1 16") and STRIDES="..." (default "0.06 0.08 0.10"); TRAJ=1 writes TRAJ_CSV per run.
D=$(cd "$(dirname "$0")" && pwd); W=${1:?outdir}; export WALK_BIN=${2:?bin}; shift 2; mkdir -p $W
SEEDS=${SEEDS:-1 16}; ST=${STRIDES:-0.06 0.08 0.10}
C=/home/takara/work/dp/articara/log/claude_g1rand_20260924/FAM_v621n_bump10/T40/m00/controls.json
R="T=15 T_DS=0 T_SS=0.30 T_FLIGHT=0.08 RUN_SWING_RISE=0.3 RUN_SWING_FALL=0.3 RUN_EVENT_TD=1 RUN_LAND_RAMP=0.03 PUSH_REPLAN=0 PUSH_Z_DROP=0 REF_LIBRARY= MPC_FOOTSTEP=0 RUN_FOOT_KX=0.1 RUN_FOOT_K=0.2 RUN_FOOT_TAU=0.1 $*"
for st in $ST; do for s in $(seq $SEEDS); do echo "$st $s"; done; done |
  xargs -P 16 -L 1 sh -c "$D/runwalk.sh $C $W/st\$0_s\$1.log $R STRIDE=\$0 INIT_SEED=\$1 ${TRAJ:+TRAJ_CSV=$W/st\$0_s\$1.csv} >/dev/null"
t=0; for st in $ST; do n=$(grep -l 'verdict: SURVIVED' $W/st${st}_s*.log | wc -l); t=$((t+n)); echo -n "stride $st: $n  "; done; echo "total $t"
