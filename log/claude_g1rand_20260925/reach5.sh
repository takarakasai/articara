set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
L=log/claude_g1rand_20260925; C=log/claude_g1champion_20261010v67/controls.json; BIN=$L/kyo46rs_walk_rh6
for m in 0.95 0.96; do
X="{\"MPC_REACH\":\"$m\",\"MPC_REACH_AHEAD\":\"0.0\",\"MPC_REACH_SAGITTAL\":\"1\",\"PUSH_Z_DROP\":\"0\"}"
TAG=RS${m}_p BASE=$C BIN=$BIN MODE=push N=48 W=12 SEED=20260924 EXTRA="$X" CONDS=11@180,11@0 python3 log/claude_g1rand_20260924/rand.py
TAG=RS${m}_lat BASE=$C BIN=$BIN MODE=push N=24 W=12 SEED=20260924 EXTRA="$X" CONDS=4@270,5@90,6@270,7@90 python3 log/claude_g1rand_20260924/rand.py
done
{ for m in 0.95 0.96; do echo "=== sagittal $m/0: push vs v6.7"; python3 $L/mcpair.py RC_p RS${m}_p push; echo "=== sagittal $m/0: lat vs v6.7"; python3 $L/mcpair.py RC_lat RS${m}_lat push; done; echo "#### REACH5_DONE"; } > $L/reach5_summary.out 2>&1
