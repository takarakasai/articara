#!/bin/bash
# kr.sh OUTDIR [K=V ...]: kyo46rs v4 + the G1 running (WR10) scaled to kyo's leg (length x0.45, time x0.67).
# Env STRIDES (default "0.032 0.043 0.054"), SEEDS (default "1 16"), WALK_BIN (default tu88), TRAJ=1 writes TRAJ_CSV.
# VX is set to STRIDE/T_SS like the G1 default (kyo's controls fix VX=0.03). Prints survivors per stride.
D=$(cd "$(dirname "$0")" && pwd); W=${1:?outdir}; shift; mkdir -p $W
export WALK_BIN=${WALK_BIN:-log/claude_g1rand_20260925/kyo46rs_walk_tu88}
SEEDS=${SEEDS:-1 16}; ST=${STRIDES:-0.032 0.043 0.054}
C=/home/takara/work/dp/articara/log/claude_kyo46rs_base_20261101v4/controls.json
TSS=0.20; for a in "$@"; do case $a in T_SS=*) TSS=${a#T_SS=};; esac; done
R="INIT_JITTER=0.01 T=10 N_STEPS=28 STRIDE_RAMP=3 FLAT_START_X=30 T_DS=0 T_SS=0.20 T_FLIGHT=0.04 RUN_SWING_RISE=0.15 RUN_SWING_FALL=0.3 RUN_EVENT_TD=1 RUN_LAND_RAMP=0.02 \
PUSH_REPLAN=0.045 PUSH_Z_DROP=0 REF_LIBRARY= MPC_FOOTSTEP=0 RUN_FOOT_KX=0 RUN_FOOT_K=0.2 RUN_FOOT_TAU=0.067 RUN_FOOT_MAX=0.036 RUN_FOOT_XI_MAX=0.045 \
RUN_Z_LEVEL=1 VX2=0 VX2_AT=24 VX2_RAMP=3 RUN_FOOT_XI_G=1.5 SENSE_PRED_TICKS=3 CP_RUN_CROSS_HOLD=0.009 $*"
for st in $ST; do for s in $(seq $SEEDS); do echo "$st $s"; done; done |
  xargs -P 16 -L 1 sh -c "$D/../claude_g1run_20261121/runwalk.sh $C $W/st\$0_s\$1.log $R STRIDE=\$0 VX=\$(python3 -c \"print(\$0/$TSS)\") INIT_SEED=\$1 TERRAIN_SEED=\$1 SENSE_SEED=\$1 ${TRAJ:+TRAJ_CSV=$W/st\$0_s\$1.csv} >/dev/null"
t=0; for st in $ST; do n=$(grep -l 'verdict: SURVIVED' $W/st${st}_s*.log 2>/dev/null | wc -l); t=$((t+n)); echo -n "stride $st: $n  "; done; echo "total $t"
