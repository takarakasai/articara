#!/bin/bash
# wr09.sh OUTDIR [BIN]: the 11-24 §74 candidate: WR08 (wr08.sh) with the predictor's length from the delay
# identified online (SENSE_PRED_AUTO=16: a +-1 Nm torque dither for the first 1 s, SENSE_PRED_DITHER_T=1.0;
# its correlation with the measured joint velocity changes gives the total delay L; predict L + 3).
# SENSE_PRED_TICKS=3 until the estimate is in (0.5 s). Strides 0.072 / 0.096 / 0.12. Env SEEDS / TRAJ as in wr02.sh.
# tu87 = articara with the local misa-locomotion 703b708 + SENSE_PRED_DITHER_T (not pushed).
# Robustness battery: EXTRA="RUN_FOOT_XI_G=1.5 RUN_FOOT_KX=0 SENSE_PRED_TICKS=3 CP_RUN_CROSS_HOLD=0.02 SENSE_PRED_AUTO=16 SENSE_PRED_AUTO_ADD=3 SENSE_PRED_DITHER_T=1.0" wr05.sh OUT BIN.
D=$(cd "$(dirname "$0")" && pwd)
STRIDES=${STRIDES:-0.072 0.096 0.12} exec $D/wr02.sh ${1:?outdir} ${2:-log/claude_g1rand_20260925/kyo46rs_walk_tu87} \
  T_FLIGHT=0.06 RUN_Z_LEVEL=1 RUN_SWING_RISE=0.15 VX2=0 VX2_AT=24 VX2_RAMP=3 RUN_FOOT_XI_G=1.5 RUN_FOOT_KX=0 SENSE_PRED_TICKS=3 \
  CP_RUN_CROSS_HOLD=0.02 SENSE_PRED_AUTO=16 SENSE_PRED_AUTO_ADD=3 SENSE_PRED_DITHER_T=1.0
