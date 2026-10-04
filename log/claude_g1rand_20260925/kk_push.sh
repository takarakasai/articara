set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
L=log/claude_g1rand_20260925; BIN=$L/kyo46rs_walk_ev2; C=log/claude_g1champion_20261003v62/controls.json
until grep -q KK_DONE $L/kk_KB_summary.out 2>/dev/null; do sleep 30; done
X='{"CONTACT_EVENT":"1","STAIR_ABSOLUTE_TD":"1","TRACKING_CUBIC_FF":"1"}'
C8=0@0,8@0,11@0,3@270,5@270,7@90,11@180,14@180; LAT=3@270,4@270,5@270,6@270,3@90,5@90,7@90
TAG=KA_p BASE=$C BIN=$BIN MODE=push N=48 W=12 SEED=20260924 EXTRA="$X" CONDS=$C8 python3 log/claude_g1rand_20260924/rand.py
TAG=KA_lat BASE=$C BIN=$BIN MODE=push N=24 W=12 SEED=20260924 EXTRA="$X" CONDS=$LAT python3 log/claude_g1rand_20260924/rand.py
( export MU_LO=0.40 MU_HI=0.90 JITTER=0.03 DX=0.05; TAG=KAW_p BASE=$C BIN=$BIN MODE=push N=48 W=12 SEED=20260924 EXTRA="$X" CONDS=$C8 python3 log/claude_g1rand_20260924/rand.py )
{ python3 $L/mcpair.py WQF_p KA_p; python3 $L/mcpair.py WQF_lat KA_lat; python3 $L/mcpair.py WQFW_p KAW_p; echo "#### KKP_DONE"; } > $L/kk_push_summary.out 2>&1
