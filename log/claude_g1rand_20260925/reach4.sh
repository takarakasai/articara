set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
L=log/claude_g1rand_20260925; C=log/claude_g1champion_20261010v67/controls.json; BIN=$L/kyo46rs_walk_rh5
X='{"MPC_REACH":"0.96","MPC_REACH_AHEAD":"0.0","PUSH_Z_DROP":"0"}'
TAG=RB96_p BASE=$C BIN=$BIN MODE=push N=48 W=12 SEED=20260924 EXTRA="$X" CONDS=0@0,8@0,11@0,3@270,5@270,7@90,11@180,14@180 python3 log/claude_g1rand_20260924/rand.py
TAG=RB96_lat BASE=$C BIN=$BIN MODE=push N=24 W=12 SEED=20260924 EXTRA="$X" CONDS=3@270,4@270,5@270,6@270,3@90,5@90,7@90 python3 log/claude_g1rand_20260924/rand.py
Y='{"MPC_REACH":"0.95","MPC_REACH_AHEAD":"0.05","PUSH_Z_DROP":"0"}'
TAG=RB955_p BASE=$C BIN=$BIN MODE=push N=48 W=12 SEED=20260924 EXTRA="$Y" CONDS=0@0,8@0,11@0,3@270,5@270,7@90,11@180,14@180 python3 log/claude_g1rand_20260924/rand.py
{ echo "=== 0.96/ahead 0: std push vs v6.7"; python3 $L/mcpair.py RC_p RB96_p push; echo "=== 0.96/ahead 0: lat vs v6.7"; python3 $L/mcpair.py RC_lat RB96_lat push; echo "=== 0.95/ahead 0.05: std push vs v6.7"; python3 $L/mcpair.py RC_p RB955_p push; echo "#### REACH4_DONE"; } > $L/reach4_summary.out 2>&1
