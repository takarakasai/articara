set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
L=log/claude_g1rand_20260925; C=log/claude_g1champion_20261008v65/controls.json; BIN=$L/kyo46rs_walk_pr2; T=/data/articara/trajopt
LIB="$T/ew5_03.json,$T/ew5_05.json"
TAG=PM_lat BASE=$C BIN=$BIN MODE=push N=24 W=12 SEED=20260924 EXTRA="{\"REF_LIBRARY\":\"$LIB\",\"REF_TRIG_PRED\":\"0.04\"}" CONDS=3@270,4@270,5@270,6@270,3@90,5@90,7@90 python3 log/claude_g1rand_20260924/rand.py
TAG=PM_p BASE=$C BIN=$BIN MODE=push N=48 W=12 SEED=20260924 EXTRA="{\"REF_LIBRARY\":\"$LIB\",\"REF_TRIG_PRED\":\"0.04\",\"PUSH_Z_DROP\":\"0.035\"}" CONDS=0@0,8@0,11@0,3@270,5@270,7@90,11@180,14@180 python3 log/claude_g1rand_20260924/rand.py
{ echo "=== pred 0.04 + delay-0.04 refs: lat vs v6.5"; python3 $L/mcpair.py RFE_lat PM_lat push; echo "=== std push (with drop 0.035) vs v6.6 cand"; python3 $L/mcpair.py ZE0.035_p PM_p push; echo "#### PRED2_DONE"; } > $L/pred2_summary.out 2>&1
