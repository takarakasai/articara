#!/bin/bash
# wr06.sh OUTDIR [BIN]: the 11-22 §69 running settings: WR04 (wr04.sh) with the touchdown following
# the DCM's error against the plan (RUN_FOOT_XI_G 1.5, sagittal) in place of the velocity term
# (RUN_FOOT_KX 0). Strides 0.072 / 0.096 / 0.12. Env SEEDS / TRAJ as in wr02.sh.
# Baseline WR06 (BASELINE_RUN_WR06.md). tu80 = articara with misa-locomotion g1-run-z 39ad9a3 patched in.
# Robustness battery: EXTRA="RUN_FOOT_XI_G=1.5 RUN_FOOT_KX=0" wr05.sh OUT BIN.
D=$(cd "$(dirname "$0")" && pwd)
STRIDES=${STRIDES:-0.072 0.096 0.12} exec $D/wr02.sh ${1:?outdir} ${2:-log/claude_g1rand_20260925/kyo46rs_walk_tu80} \
  T_FLIGHT=0.06 RUN_Z_LEVEL=1 RUN_SWING_RISE=0.15 VX2=0 VX2_AT=24 VX2_RAMP=3 RUN_FOOT_XI_G=1.5 RUN_FOOT_KX=0
