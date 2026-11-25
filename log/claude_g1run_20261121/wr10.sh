#!/bin/bash
# wr10.sh OUTDIR [BIN]: the 11-25 §75 candidate: WR09 (wr09.sh) with the walking controls' backward re-plan
# at a running trigger (PUSH_REPLAN=0.10: the walking 0.03 fires on the normal running error before any push).
# Strides 0.072 / 0.096 / 0.12. Env SEEDS / TRAJ as in wr02.sh. No new code: tu87 as wr09.sh.
# Robustness battery: EXTRA="RUN_FOOT_XI_G=1.5 RUN_FOOT_KX=0 SENSE_PRED_TICKS=3 CP_RUN_CROSS_HOLD=0.02 SENSE_PRED_AUTO=16 SENSE_PRED_AUTO_ADD=3 SENSE_PRED_DITHER_T=1.0 PUSH_REPLAN=0.10" wr05.sh OUT BIN.
D=$(cd "$(dirname "$0")" && pwd)
STRIDES=${STRIDES:-0.072 0.096 0.12} exec $D/wr02.sh ${1:?outdir} ${2:-log/claude_g1rand_20260925/kyo46rs_walk_tu87} \
  T_FLIGHT=0.06 RUN_Z_LEVEL=1 RUN_SWING_RISE=0.15 VX2=0 VX2_AT=24 VX2_RAMP=3 RUN_FOOT_XI_G=1.5 RUN_FOOT_KX=0 SENSE_PRED_TICKS=3 \
  CP_RUN_CROSS_HOLD=0.02 SENSE_PRED_AUTO=16 SENSE_PRED_AUTO_ADD=3 SENSE_PRED_DITHER_T=1.0 PUSH_REPLAN=0.10
