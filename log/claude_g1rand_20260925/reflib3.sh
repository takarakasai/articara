set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
L=log/claude_g1rand_20260925; C=log/claude_g1champion_20261008v65/controls.json; BIN=$L/kyo46rs_walk_zd2
LIB7="/data/articara/trajopt/dw7_02.json,/data/articara/trajopt/dwk7_03.json,/data/articara/trajopt/dw7_04.json,/data/articara/trajopt/dw7_05.json,/data/articara/trajopt/dw7_06.json"
TAG=RL3_p BASE=$C BIN=$BIN MODE=push N=48 W=12 SEED=20260924 EXTRA="{\"REF_LIBRARY\":\"$LIB7\"}" CONDS=7@90,5@90 python3 log/claude_g1rand_20260924/rand.py
TAG=RL3_lat BASE=$C BIN=$BIN MODE=push N=24 W=12 SEED=20260924 EXTRA="{\"REF_LIBRARY\":\"$LIB7\"}" CONDS=3@270,4@270,5@270,6@270,3@90,5@90,7@90 python3 log/claude_g1rand_20260924/rand.py
{ echo "=== 7 N s-only library (5 phases) vs v6.5"; python3 $L/mcpair.py RFE_p RL3_p push; echo "=== lat"; python3 $L/mcpair.py RFE_lat RL3_lat push; echo "#### REFLIB3_DONE"; } > $L/reflib3_summary.out 2>&1
