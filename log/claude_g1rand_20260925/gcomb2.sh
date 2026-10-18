set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
L=log/claude_g1rand_20260925; BIN=$L/kyo46rs_walk_tu48; C=log/claude_g1champion_20261022v614/controls.json
export PUSH_STAIR_X=30
for h in 0.010 0.020; do
  TAG=GCOMB2_bump$h BASE=$C BIN=$BIN MODE=push N=48 W=8 SEED=20260924 EXTRA="{\"TERRAIN_BUMPS\":\"6,$h,0.25,0.8,1.8\"}" CONDS=11@0,11@180,7@90,5@270 python3 log/claude_g1rand_20260924/rand.py | sed "s/^/bump$h /"
  echo "=== push on flat -> push on bumps $h (x 0.8-1.8)"; python3 $L/gpair.py GCOMB_flat GCOMB2_bump$h
done
echo "#### GCOMB2_DONE"
