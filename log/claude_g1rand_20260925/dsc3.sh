set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
L=log/claude_g1rand_20260925; C=log/claude_g1champion_20261010v67/controls.json; BIN=$L/kyo46rs_walk_dc5
X='{"DS_CAPTURE":"1"}'
TAG=DCG_p BASE=$C BIN=$BIN MODE=push N=48 W=12 SEED=20260924 EXTRA="$X" CONDS=0@0,8@0,11@0,3@270,5@270,7@90,11@180,14@180 python3 log/claude_g1rand_20260924/rand.py
TAG=DCG_lat BASE=$C BIN=$BIN MODE=push N=24 W=12 SEED=20260924 EXTRA="$X" CONDS=3@270,4@270,5@270,6@270,3@90,5@90,7@90 python3 log/claude_g1rand_20260924/rand.py
{ echo "=== DS_CAPTURE (degenerate fix, margin 1 cm): std push vs v6.7"; python3 $L/mcpair.py RC_p DCG_p push; echo "=== lat vs v6.7"; python3 $L/mcpair.py RC_lat DCG_lat push; echo "#### DSC3_DONE"; } > $L/dsc3_summary.out 2>&1
