#!/bin/bash
# pgrid3.sh OUTDIR [BIN]: WR08 nom (48 runs, seeds 1..16) over the actual sensor delay SENSE_DELAY_TICKS
# x SENSE_PRED_TICKS, including predictions longer than the delay (§74: the cost of a margin).
# Env DS="0 2 4 6 8", PS="3 5 7 9 11 13", XS="" (extra K=V for every cell).
D=$(cd "$(dirname "$0")/.." && pwd); O=${1:?outdir}; B=${2:-log/claude_g1rand_20260925/kyo46rs_walk_tu84}
for d in ${DS:-0 2 4 6 8}; do for p in ${PS:-3 5 7 9 11 13}; do
  echo "== d${d}_p${p}"
  STRIDES="0.072 0.096 0.12" $D/wr02.sh $O/d${d}_p${p} $B T_FLIGHT=0.06 RUN_Z_LEVEL=1 RUN_SWING_RISE=0.15 VX2=0 VX2_AT=24 VX2_RAMP=3 \
    RUN_FOOT_XI_G=1.5 RUN_FOOT_KX=0 CP_RUN_CROSS_HOLD=0.02 SENSE_DELAY_TICKS=$d SENSE_PRED_TICKS=$p ${XS:-}
done; done
echo PGRID3_DONE
