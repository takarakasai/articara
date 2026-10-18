set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
L=log/claude_g1rand_20260925; BIN=$L/kyo46rs_walk_tu49; C=log/claude_g1champion_20261022v614/controls.json
NOISE="SENSE_NOISE_QD=0.1;SENSE_NOISE_V=0.03;SENSE_NOISE_W=0.03"
for f in 5 10 15 20; do MODE=flat N=48 W=24 SEED=20260924 T=25 TAG=GSENSE7_f$f BASE=$C BIN=$BIN EXTRA="{\"SENSE_FILTER_V_HZ\":\"$f\"}" CONDS="$NOISE" python3 log/claude_g1rand_20260924/rand.py | sed "s/^/noise v$f /"; done
C8=0@0,8@0,11@0,3@270,5@270,7@90,11@180,14@180
for f in 10 15; do X="\"SENSE_FILTER_V_HZ\":\"$f\""
  TAG=GSENSE7_p_f$f BASE=$C BIN=$BIN MODE=push N=48 W=24 SEED=20260924 EXTRA="{$X}" CONDS=$C8 python3 log/claude_g1rand_20260924/rand.py | sed "s/^/p v$f /"
  TAG=GSENSE7_lat_f$f BASE=$C BIN=$BIN MODE=push N=24 W=24 SEED=20260924 EXTRA="{$X}" CONDS=3@270,4@270,5@270,6@270,3@90,5@90,7@90 python3 log/claude_g1rand_20260924/rand.py | sed "s/^/lat v$f /"
  echo "=== v$f p vs v6.14"; python3 $L/mcpair.py GV614e_p GSENSE7_p_f$f push | grep -v '^GSENSE'
  echo "=== v$f lat vs v6.14"; python3 $L/mcpair.py GV614e_lat GSENSE7_lat_f$f push | grep -v '^GSENSE'
done
echo "#### GSENSE7_DONE"
