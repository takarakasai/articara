#!/bin/bash
# mixchk.sh NAME EXTRA_FILE|-: 凹凸とセンサの雑音を家族の種で振り直して回す（2026-11-01、rand.py の
#   SEED_MIX_TERRAIN=1・SEED_MIX_SENSE=1）。kfamily.sh の kbump05・kbump10 と ksense の雑音 3 条件を、
#   3 つの種（n 31415、o 27182、t 16180）で。結果は KFAM_<NAME><n|o|t>_{kbump05m,kbump10m,ksensem}。
set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
L=log/claude_g1rand_20260925; C=$L/kyo_base_v3.json; B=${BIN:-$L/kyo46rs_walk_ml5}; N0=$1; W=${W:-16}
R=log/claude_g1rand_20260924/rand.py
if [ "$2" = - ]; then X=''; else X=$(cat $2); fi
J() { if [ -n "$X" ]; then echo "{$X${1:+,$1}}"; else echo "{${1:-}}"; fi; }
export SEED_MIX_TERRAIN=1 SEED_MIX_SENSE=1
for p in n:31415 o:27182 t:16180; do s=${p%%:*}; S=${p#*:}
  for h in 05 10; do
    env MODE=flat N=48 W=$W SEED=$S T=28 DX=0.01 FLAT_START_X=30 TAG=KFAM_${N0}${s}_kbump${h}m BASE=$C BIN=$B \
      EXTRA="$(J "\"TERRAIN_BUMPS\":\"4,0.0$h,0.08,0.15,0.60\"")" CONDS='T=28' python3 $R
  done
  env MODE=flat N=24 W=$W SEED=$S T=28 DX=0.01 FLAT_START_X=30 TAG=KFAM_${N0}${s}_ksensem BASE=$C BIN=$B EXTRA="$(J)" \
    CONDS='SENSE_NOISE_QD=0.05;SENSE_NOISE_V=0.015;SENSE_NOISE_W=0.015,SENSE_NOISE_QD=0.1;SENSE_NOISE_V=0.03;SENSE_NOISE_W=0.03,SENSE_NOISE_Q=0.002' python3 $R
done
echo "#### MIXCHK_DONE $N0"
