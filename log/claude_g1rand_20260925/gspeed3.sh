set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
L=log/claude_g1rand_20260925; BIN=$L/kyo46rs_walk_tu40; C=log/claude_g1champion_20261021v613/controls.json
for v in 0.30 0.40 0.50; do
  ( export PUSH_STEP_OFFSET=6 PUSH_STAIR_X=30 T=30
    TAG=GSPD_v613g_${v#0.} BASE=$C BIN=$BIN MODE=push N=48 W=12 SEED=20260924 EXTRA="{\"VX2_SCHED\":\"1\",\"VX2\":\"$v\",\"VX2_AT\":\"8\",\"MPC_FOOTSTEP_BACK_VMAX\":\"0.15\"}" CONDS=11@0,11@180,7@90,5@270 python3 log/claude_g1rand_20260924/rand.py | sed "s/^/v613g $v /" )
done
for v in 30 50; do echo "=== $v v612 -> v613g"; python3 $L/mcpair.py GSPD_v612_$v GSPD_v613g_$v push | grep -v '^GSPD'; done
echo "=== 40 v611 -> v613g"; python3 $L/mcpair.py GSPD_v611_40 GSPD_v613g_40 push | grep -v '^GSPD'
echo "#### GSPD3_DONE"
