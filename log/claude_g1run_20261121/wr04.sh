#!/bin/bash
# wr04.sh OUTDIR [BIN]: the 11-22 §64 running settings: §63 (wr03.sh) with a longer flight
# (T_FLIGHT 0.06), the trailing foot lifted fast (RUN_SWING_RISE 0.15) and the last steps slowed
# to a stop (VX2=0 from step 24 over 3 steps). Strides 0.072 / 0.096 / 0.12 = 0.2 / 0.27 / 0.33 m/s.
# Env SEEDS / TRAJ as in wr02.sh.
# Baseline WR04 (11-22, BASELINE_RUN_WR04.md). tu72 = articara with misa-locomotion d6d7ee4 pinned (bit-identical to tu71).
D=$(cd "$(dirname "$0")" && pwd)
STRIDES=${STRIDES:-0.072 0.096 0.12} exec $D/wr02.sh ${1:?outdir} ${2:-log/claude_g1rand_20260925/kyo46rs_walk_tu72} \
  T_FLIGHT=0.06 RUN_Z_LEVEL=1 RUN_SWING_RISE=0.15 VX2=0 VX2_AT=24 VX2_RAMP=3
