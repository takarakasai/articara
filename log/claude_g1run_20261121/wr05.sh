#!/bin/bash
# wr05.sh OUTDIR [BIN]: robustness of the running baseline WR04 (wr04.sh). Each condition is
# 48 runs (strides 0.072 / 0.096 / 0.12 x INIT_SEED 1..16); TERRAIN_SEED / SENSE_SEED follow
# INIT_SEED so the bumps and the noise differ per run. Prints survivors per condition.
# The base controls carry accelerometer noise (SENSE_NOISE_A 0.02), so nom differs from wr04.sh
# (one noise stream there): compare conditions with nom of this battery. Env ONLY="name ..." runs a subset.
L=$(cd "$(dirname "$0")" && pwd); O=${1:?outdir}; B=${2:-log/claude_g1rand_20260925/kyo46rs_walk_tu72}
export WALK_BIN=$B
P="PUSH_STEP=12 PUSH_AT=0.5"
CONDS=(
  "nom|"
  "mu60|MU_GROUND=0.60" "mu50|MU_GROUND=0.50" "mu40|MU_GROUND=0.40"
  "delay8|SENSE_DELAY_TICKS=8" "noise|SENSE_NOISE_QD=0.1 SENSE_NOISE_V=0.03 SENSE_NOISE_W=0.03"
  "bump5|TERRAIN_BUMPS=6,0.005,0.25,0.5,2.0" "bump10|TERRAIN_BUMPS=6,0.010,0.25,0.5,2.0"
  "pay5|PLANT_PAYLOAD=5,0,0,0.2" "act2|ACT_DELAY_TICKS=2"
  "pf3|PUSH_IMPULSE=3 PUSH_DEG=0 $P" "pb3|PUSH_IMPULSE=3 PUSH_DEG=180 $P"
  "pl3|PUSH_IMPULSE=3 PUSH_DEG=90 $P" "pr3|PUSH_IMPULSE=3 PUSH_DEG=270 $P"
  "pf6|PUSH_IMPULSE=6 PUSH_DEG=0 $P" "pb6|PUSH_IMPULSE=6 PUSH_DEG=180 $P"
  "pl6|PUSH_IMPULSE=6 PUSH_DEG=90 $P" "pr6|PUSH_IMPULSE=6 PUSH_DEG=270 $P"
)
for c in "${CONDS[@]}"; do n=${c%%|*}; x=${c#*|}
  [ -n "${ONLY:-}" ] && [[ " $ONLY " != *" $n "* ]] && continue
  W=$O/$n; mkdir -p $W
  for st in 0.072 0.096 0.12; do for s in $(seq 1 16); do echo "$st $s"; done; done |
    xargs -P 16 -L 1 sh -c "$L/runwalk.sh /home/takara/work/dp/articara/log/claude_g1rand_20260924/FAM_v621n_bump10/T40/m00/controls.json $W/st\$0_s\$1.log \
      T=15 T_DS=0 T_SS=0.30 T_FLIGHT=0.06 RUN_SWING_RISE=0.15 RUN_SWING_FALL=0.3 RUN_EVENT_TD=1 RUN_LAND_RAMP=0.03 PUSH_REPLAN=0 PUSH_Z_DROP=0 REF_LIBRARY= MPC_FOOTSTEP=0 RUN_FOOT_KX=0.1 RUN_FOOT_K=0.2 RUN_FOOT_TAU=0.1 \
      RUN_Z_LEVEL=1 VX2=0 VX2_AT=24 VX2_RAMP=3 $x STRIDE=\$0 INIT_SEED=\$1 TERRAIN_SEED=\$1 SENSE_SEED=\$1 >/dev/null" 
  echo -n "$n:"; t=0; for st in 0.072 0.096 0.12; do k=$(grep -l 'verdict: SURVIVED' $W/st${st}_s*.log 2>/dev/null | wc -l); t=$((t+k)); echo -n " $k"; done; echo "  = $t / 48"
done
