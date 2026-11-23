#!/bin/bash
# wr07.sh OUTDIR [BIN]: the 11-23 §71 running settings: WR06 (wr06.sh) with the latency predictor
# SENSE_PRED_TICKS 3 (the estimate advanced 3 control periods by the WBC's own last accelerations).
# Strides 0.072 / 0.096 / 0.12. Env SEEDS / TRAJ as in wr02.sh.
# Baseline WR07 (BASELINE_RUN_WR07.md). tu81 = articara with misa-locomotion main 04ef595 patched in.
# With a sensor / actuator delay, set SENSE_PRED_TICKS to the total delay + 1 or more (wr02.sh directly).
# Robustness battery: EXTRA="RUN_FOOT_XI_G=1.5 RUN_FOOT_KX=0 SENSE_PRED_TICKS=3" wr05.sh OUT BIN.
D=$(cd "$(dirname "$0")" && pwd)
STRIDES=${STRIDES:-0.072 0.096 0.12} exec $D/wr02.sh ${1:?outdir} ${2:-log/claude_g1rand_20260925/kyo46rs_walk_tu81} \
  T_FLIGHT=0.06 RUN_Z_LEVEL=1 RUN_SWING_RISE=0.15 VX2=0 VX2_AT=24 VX2_RAMP=3 RUN_FOOT_XI_G=1.5 RUN_FOOT_KX=0 SENSE_PRED_TICKS=3
