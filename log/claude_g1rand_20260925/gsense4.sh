set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
L=log/claude_g1rand_20260925; BIN=$L/kyo46rs_walk_tu48; C=log/claude_g1champion_20261022v614/controls.json
NOISE="SENSE_NOISE_QD=0.1;SENSE_NOISE_V=0.03;SENSE_NOISE_W=0.03"
for f in 15 25 30; do MODE=flat N=48 W=16 SEED=20260924 T=25 TAG=GSENSE4_f$f BASE=$C BIN=$BIN EXTRA="{\"SENSE_FILTER_HZ\":\"$f\"}" CONDS="$NOISE" python3 log/claude_g1rand_20260924/rand.py | sed "s/^/noise f$f /"; done
export PUSH_STAIR_X=30
for f in 20 25; do
  TAG=GSENSE4_push_f$f BASE=$C BIN=$BIN MODE=push N=48 W=16 SEED=20260924 EXTRA="{\"SENSE_FILTER_HZ\":\"$f\"}" CONDS=11@0,11@180,7@90,5@270 python3 log/claude_g1rand_20260924/rand.py | sed "s/^/push f$f /"
  echo "=== clean push v6.14 -> filter $f Hz"; python3 $L/gpair.py GCOMB_flat GSENSE4_push_f$f
done
echo "#### GSENSE4_DONE"
