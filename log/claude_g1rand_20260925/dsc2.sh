set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
L=log/claude_g1rand_20260925; C=log/claude_g1champion_20261010v67/controls.json; BIN=$L/kyo46rs_walk_dc4
TAG=DCF_p BASE=$C BIN=$BIN MODE=push N=48 W=12 SEED=20260924 EXTRA='{"DS_CAPTURE":"1"}' CONDS=11@0,11@180,7@90,5@270 python3 log/claude_g1rand_20260924/rand.py
{ echo "=== DS_CAPTURE (ref gate + forward only) vs v6.7"; python3 $L/mcpair.py RC_p DCF_p push; echo "#### DSC2_DONE"; } > $L/dsc2_summary.out 2>&1
