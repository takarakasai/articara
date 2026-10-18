set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
L=log/claude_g1rand_20260925; BIN=$L/kyo46rs_walk_tu48; C=log/claude_g1champion_20261022v614/controls.json
C8=0@0,8@0,11@0,3@270,5@270,7@90,11@180,14@180
for f in 15 20; do X="\"SENSE_FILTER_HZ\":\"$f\""
  TAG=GSENSE5_p_f$f BASE=$C BIN=$BIN MODE=push N=48 W=24 SEED=20260924 EXTRA="{$X}" CONDS=$C8 python3 log/claude_g1rand_20260924/rand.py | sed "s/^/p f$f /"
  TAG=GSENSE5_lat_f$f BASE=$C BIN=$BIN MODE=push N=24 W=24 SEED=20260924 EXTRA="{$X}" CONDS=3@270,4@270,5@270,6@270,3@90,5@90,7@90 python3 log/claude_g1rand_20260924/rand.py | sed "s/^/lat f$f /"
  TAG=GSENSE5_s_f$f BASE=$C BIN=$BIN MODE=stairs N=48 W=24 SEED=20260924 EXTRA="{$X}" RISERS=0.020,0.040,0.060,0.080 python3 log/claude_g1rand_20260924/rand.py | sed "s/^/s f$f /"
  TAG=GSENSE5_desc_f$f STAIR_N=1 BASE=$C BIN=$BIN MODE=stairs N=48 W=24 SEED=20260924 START_X=0.40 EXTRA="{\"SPAWN_Z_EXTRA\":\"0.005\",$X}" RISERS=-0.020,-0.040,-0.060,-0.080 python3 log/claude_g1rand_20260924/rand.py | sed "s/^/desc f$f /"
  echo "=== f$f p vs v6.14"; python3 $L/mcpair.py GV614e_p GSENSE5_p_f$f push | grep -v '^GSENSE'
  echo "=== f$f lat vs v6.14"; python3 $L/mcpair.py GV614e_lat GSENSE5_lat_f$f push | grep -v '^GSENSE'
  echo "=== f$f stairs vs v6.14"; python3 $L/mcpair.py GV614g_s GSENSE5_s_f$f | grep -v '^GSENSE'
  echo "=== f$f desc vs v6.14"; python3 $L/mcpair.py GV614g_desc GSENSE5_desc_f$f | grep -v '^GSENSE'
done
echo "#### GSENSE5_DONE"
