#!/bin/bash
# wr01.sh OUTDIR [BIN]: G1 walker running (misa-locomotion g1-run 522346d = kyo46rs_walk_tu69),
# the best gains of 11-21 (RUN_FOOT_KX 0.1 / RUN_FOOT_K 0.2 / RUN_FOOT_TAU 0.1),
# strides 0.06 / 0.08 / 0.10 (0.2 / 0.27 / 0.33 m/s), INIT_SEED 1..16. Prints survivors per stride.
D=$(cd "$(dirname "$0")" && pwd); W=${1:?outdir}; mkdir -p $W
export WALK_BIN=${2:-log/claude_g1rand_20260925/kyo46rs_walk_tu69}
C=/home/takara/work/dp/articara/log/claude_g1rand_20260924/FAM_v621n_bump10/T40/m00/controls.json
R="T=15 T_DS=0 T_SS=0.30 T_FLIGHT=0.08 RUN_SWING_RISE=0.3 RUN_SWING_FALL=0.3 RUN_EVENT_TD=1 RUN_LAND_RAMP=0.03 PUSH_REPLAN=0 PUSH_Z_DROP=0 REF_LIBRARY= MPC_FOOTSTEP=0 RUN_FOOT_KX=0.1 RUN_FOOT_K=0.2 RUN_FOOT_TAU=0.1"
for st in 0.06 0.08 0.10; do for s in $(seq 1 16); do echo "$st $s"; done; done |
  xargs -P 16 -L 1 sh -c "$D/runwalk.sh $C $W/st\$0_s\$1.log $R STRIDE=\$0 INIT_SEED=\$1 >/dev/null"
for st in 0.06 0.08 0.10; do echo "stride $st: $(grep -l 'verdict: SURVIVED' $W/st${st}_s*.log | wc -l)/16"; done
