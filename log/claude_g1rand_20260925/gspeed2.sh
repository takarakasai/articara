set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
L=log/claude_g1rand_20260925; BIN=$L/kyo46rs_walk_tu38; C=log/claude_g1champion_20261020v612/controls.json
for v in 0.30 0.50; do
  ( export PUSH_STEP_OFFSET=6 PUSH_STAIR_X=30 T=30
    TAG=GSPD_v612_${v#0.} BASE=$C BIN=$BIN MODE=push N=48 W=12 SEED=20260924 EXTRA="{\"VX2_SCHED\":\"1\",\"VX2\":\"$v\",\"VX2_AT\":\"8\"}" CONDS=11@0,11@180,7@90,5@270 python3 log/claude_g1rand_20260924/rand.py | sed "s/^/v612 $v /" )
done
for v in 30 50; do echo "=== $v v611->v612"; python3 $L/mcpair.py GSPD_v611_$v GSPD_v612_$v push | grep -v '^GSPD'; echo "=== $v v612->v613"; python3 $L/mcpair.py GSPD_v612_$v GSPD_v613_$v push | grep -v '^GSPD'; done
echo "#### GSPD2_DONE"
