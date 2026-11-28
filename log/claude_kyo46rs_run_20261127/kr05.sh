#!/bin/bash
# kr05.sh OUTDIR: robustness of the kyo46rs running candidate KR01 (kr01.sh), the G1 battery wr05.sh scaled to kyo.
# Each condition is 48 runs (strides 0.032 / 0.043 / 0.054 x seeds SEED0+1..SEED0+16; TERRAIN_SEED / SENSE_SEED follow).
# Sizes follow the kyo walking family (kfamily.sh): pushes 0.3 / 0.6 N.s (kyo's walking family 0.3..0.9), sensor
# delay 2 / 4 ticks (8 kills kyo's walking), the G1 noise, payload 1 kg (14 % of the body, as G1's 5 kg), bumps 5 / 10 mm
# as 6 boxes of length 0.11 m over x 0.11..0.90 (G1's 0.25 m over 0.25..2.0 scaled by the leg, stiff contact).
# Pushes at step 12 (right-foot stance) as in wr05.sh; pl3x / pl6x push left at step 13 (the crossing case on the other side).
# Env ONLY="name ...", EXTRA="K=V ..." (added to every condition), SEED0 (default 0), RUN=script (default kr01.sh).
D=$(cd "$(dirname "$0")" && pwd); O=${1:?outdir}; S0=${SEED0:-0}; RUN=${RUN:-$D/kr01.sh}
P="PUSH_STEP=12 PUSH_AT=0.5"; PX="PUSH_STEP=13 PUSH_AT=0.5"; BS="TERRAIN_BUMP_SOLREF=0.005,1"
CONDS=(
  "nom|"
  "mu60|MU_GROUND=0.60" "mu50|MU_GROUND=0.50" "mu40|MU_GROUND=0.40"
  "delay2|SENSE_DELAY_TICKS=2" "delay4|SENSE_DELAY_TICKS=4" "noise|SENSE_NOISE_QD=0.1 SENSE_NOISE_V=0.03 SENSE_NOISE_W=0.03"
  "bump5|TERRAIN_BUMPS=6,0.005,0.11,0.11,0.90 $BS" "bump10|TERRAIN_BUMPS=6,0.010,0.11,0.11,0.90 $BS"
  "pay1|PLANT_PAYLOAD=1,0,0,0.09" "act2|ACT_DELAY_TICKS=2"
  "pf3|PUSH_IMPULSE=0.3 PUSH_DEG=0 $P" "pb3|PUSH_IMPULSE=0.3 PUSH_DEG=180 $P"
  "pl3|PUSH_IMPULSE=0.3 PUSH_DEG=90 $P" "pr3|PUSH_IMPULSE=0.3 PUSH_DEG=270 $P"
  "pf6|PUSH_IMPULSE=0.6 PUSH_DEG=0 $P" "pb6|PUSH_IMPULSE=0.6 PUSH_DEG=180 $P"
  "pl6|PUSH_IMPULSE=0.6 PUSH_DEG=90 $P" "pr6|PUSH_IMPULSE=0.6 PUSH_DEG=270 $P"
  "pl3x|PUSH_IMPULSE=0.3 PUSH_DEG=90 $PX" "pl6x|PUSH_IMPULSE=0.6 PUSH_DEG=90 $PX"
)
tot=0
for c in "${CONDS[@]}"; do n=${c%%|*}; x=${c#*|}
  [ -n "${ONLY:-}" ] && [[ " $ONLY " != *" $n "* ]] && continue
  r=$(SEEDS="$((S0+1)) $((S0+16))" $RUN $O/$n $x ${EXTRA:-})
  k=$(echo "$r" | grep -o 'total [0-9]*' | cut -d' ' -f2); tot=$((tot+k))
  echo "$n: $(echo "$r" | sed 's/stride //g; s/  total.*//')  = $k / 48"
done
echo "sum $tot"
