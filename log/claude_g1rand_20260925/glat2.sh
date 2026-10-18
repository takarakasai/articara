set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
L=log/claude_g1rand_20260925; C=log/claude_g1champion_20261022v614/controls.json; BIN=$L/kyo46rs_walk_tu47
for v in 0.40 0.50; do ( export PUSH_STEP_OFFSET=6 PUSH_STAIR_X=30 T=30
  TAG=GLAT2_v614_${v#0.} BASE=$C BIN=$BIN MODE=push N=48 W=12 SEED=20260924 EXTRA="{\"VX2_SCHED\":\"1\",\"VX2\":\"$v\",\"VX2_AT\":\"8\"}" CONDS=11@0,11@180,7@90,5@270 python3 log/claude_g1rand_20260924/rand.py | sed "s/^/v614 $v /"
  TAG=GLAT2_lat_${v#0.} BASE=$C BIN=$BIN MODE=push N=48 W=12 SEED=20260924 EXTRA="{\"VX2_SCHED\":\"1\",\"VX2\":\"$v\",\"VX2_AT\":\"8\",\"PUSH_REPLAN_LAT\":\"1\",\"PUSH_REPLAN_LAT_VMIN\":\"0.40\"}" CONDS=11@0,11@180,7@90,5@270 python3 log/claude_g1rand_20260924/rand.py | sed "s/^/lat $v /" ); done
for v in 40 50; do echo "=== $v v6.14 -> lat(vmin 0.40)"; python3 $L/mcpair.py GLAT2_v614_$v GLAT2_lat_$v push | grep -v '^GLAT'; done
echo "#### GLAT2_DONE"
