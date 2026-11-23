#!/bin/bash
# wr08.sh OUTDIR [BIN]: the 11-23 §73 running settings: WR07 (wr07.sh) with the stance held when the DCM
# is off toward the stance side (CP_RUN_CROSS_HOLD 0.02: the contact planner does not cut the stance
# where the swing foot would have to cross). Strides 0.072 / 0.096 / 0.12. Env SEEDS / TRAJ as in wr02.sh.
# Baseline WR08 (BASELINE_RUN_WR08.md). tu83 = articara at misa-locomotion 4ba73ef (Cargo.toml rev).
# With a sensor / actuator delay, set SENSE_PRED_TICKS to the total delay + 1 or more (wr02.sh directly).
# Robustness battery: EXTRA="RUN_FOOT_XI_G=1.5 RUN_FOOT_KX=0 SENSE_PRED_TICKS=3 CP_RUN_CROSS_HOLD=0.02" wr05.sh OUT BIN.
D=$(cd "$(dirname "$0")" && pwd)
STRIDES=${STRIDES:-0.072 0.096 0.12} exec $D/wr02.sh ${1:?outdir} ${2:-log/claude_g1rand_20260925/kyo46rs_walk_tu83} \
  T_FLIGHT=0.06 RUN_Z_LEVEL=1 RUN_SWING_RISE=0.15 VX2=0 VX2_AT=24 VX2_RAMP=3 RUN_FOOT_XI_G=1.5 RUN_FOOT_KX=0 SENSE_PRED_TICKS=3 \
  CP_RUN_CROSS_HOLD=0.02
