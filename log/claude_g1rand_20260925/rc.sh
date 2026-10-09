set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
L=log/claude_g1rand_20260925; C=log/claude_g1champion_20261010v66/controls.json; BIN=$L/kyo46rs_walk_rs1
X='{"REF_RESET_INCL_CURRENT":"1"}'
TAG=RC_p BASE=$C BIN=$BIN MODE=push N=48 W=12 SEED=20260924 EXTRA="$X" CONDS=0@0,8@0,11@0,3@270,5@270,7@90,11@180,14@180 python3 log/claude_g1rand_20260924/rand.py
TAG=RC_lat BASE=$C BIN=$BIN MODE=push N=24 W=12 SEED=20260924 EXTRA="$X" CONDS=3@270,4@270,5@270,6@270,3@90,5@90,7@90 python3 log/claude_g1rand_20260924/rand.py
{ echo "=== RC (re-centre the swing under way) std push vs v6.6"; python3 $L/mcpair.py ZE0.035_p RC_p push; echo "=== lat vs v6.6"; python3 $L/mcpair.py ZE0.035_lat RC_lat push; echo "#### RC_DONE"; } > $L/rc_summary.out 2>&1
