set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
L=log/claude_g1rand_20260925; C=log/claude_g1champion_20261020v612/controls.json; BIN=$L/kyo46rs_walk_tu42
C8=0@0,8@0,11@0,3@270,5@270,7@90,11@180,14@180
X='{"PUSH_REPLAN":"0.03"}'
TAG=GV614b_p BASE=$C BIN=$BIN MODE=push N=48 W=12 SEED=20260924 EXTRA="$X" CONDS=$C8 python3 log/claude_g1rand_20260924/rand.py | sed "s/^/p /"
TAG=GV614b_lat BASE=$C BIN=$BIN MODE=push N=24 W=12 SEED=20260924 EXTRA="$X" CONDS=3@270,4@270,5@270,6@270,3@90,5@90,7@90 python3 log/claude_g1rand_20260924/rand.py | sed "s/^/lat /"
for v in 0.30 0.50; do ( export PUSH_STEP_OFFSET=6 PUSH_STAIR_X=30 T=30
  TAG=GV614b_spd_${v#0.} BASE=$C BIN=$BIN MODE=push N=48 W=12 SEED=20260924 EXTRA="{\"VX2_SCHED\":\"1\",\"VX2\":\"$v\",\"VX2_AT\":\"8\",\"PUSH_REPLAN\":\"0.03\"}" CONDS=11@0,11@180,7@90,5@270 python3 log/claude_g1rand_20260924/rand.py | sed "s/^/spd $v /" ); done
echo "=== p vs v6.12"; python3 $L/mcpair.py GREP_p_base GV614b_p push | grep -v '^GV'
echo "=== p vs v6.13"; python3 $L/mcpair.py GREP_p_v613c GV614b_p push | grep -v '^GV'
echo "=== lat vs v6.12"; python3 $L/mcpair.py GREP_lat_base GV614b_lat push | grep -v '^GV'
for v in 30 50; do echo "=== spd $v vs v6.12"; python3 $L/mcpair.py GSPD_v612_$v GV614b_spd_$v push | grep -v '^GV'; done
echo "#### GV614B_DONE"
