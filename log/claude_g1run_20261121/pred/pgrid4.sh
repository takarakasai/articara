#!/bin/bash
# pgrid4.sh OUTDIR [BIN]: WR08 nom (48 runs, seeds 1..16) with the delay identified online (SENSE_PRED_AUTO=16,
# SENSE_PRED_AUTO_ADD=$ADD, default 2) over sensor / actuator delays the prediction is not told (§74).
# Env ADD, XS="" (extra K=V for every cell).
D=$(cd "$(dirname "$0")/.." && pwd); O=${1:?outdir}; B=${2:-log/claude_g1rand_20260925/kyo46rs_walk_tu86}
for c in "d0|" "d2|SENSE_DELAY_TICKS=2" "d4|SENSE_DELAY_TICKS=4" "d6|SENSE_DELAY_TICKS=6" "d8|SENSE_DELAY_TICKS=8" \
         "d10|SENSE_DELAY_TICKS=10" "d12|SENSE_DELAY_TICKS=12" "a2|ACT_DELAY_TICKS=2" "a4|ACT_DELAY_TICKS=4" \
         "d4a2|SENSE_DELAY_TICKS=4 ACT_DELAY_TICKS=2" "d8a4|SENSE_DELAY_TICKS=8 ACT_DELAY_TICKS=4"; do
  n=${c%%|*}; x=${c#*|}; echo "== $n"
  STRIDES="0.072 0.096 0.12" $D/wr02.sh $O/$n $B T_FLIGHT=0.06 RUN_Z_LEVEL=1 RUN_SWING_RISE=0.15 VX2=0 VX2_AT=24 VX2_RAMP=3 \
    RUN_FOOT_XI_G=1.5 RUN_FOOT_KX=0 CP_RUN_CROSS_HOLD=0.02 SENSE_PRED_TICKS=3 SENSE_PRED_AUTO=16 SENSE_PRED_AUTO_ADD=${ADD:-2} $x ${XS:-}
done
echo PGRID4_DONE
