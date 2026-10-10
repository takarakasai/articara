set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
L=log/claude_g1rand_20260925; C=log/claude_g1champion_20261010v67/controls.json; BIN=$L/kyo46rs_walk_rh4
for m in 0.95 0.96; do for ah in 0.0 0.05; do
TAG=RA${m}_${ah}_p BASE=$C BIN=$BIN MODE=push N=48 W=12 SEED=20260924 EXTRA="{\"MPC_REACH\":\"$m\",\"MPC_REACH_AHEAD\":\"$ah\",\"PUSH_Z_DROP\":\"0\"}" CONDS=11@180,11@0 python3 log/claude_g1rand_20260924/rand.py
done; done
{ for m in 0.95 0.96; do for ah in 0.0 0.05; do echo "=== MPC_REACH $m AHEAD $ah (PUSH_Z_DROP off) vs v6.7"; python3 $L/mcpair.py RC_p RA${m}_${ah}_p push; done; done; echo "#### REACH2_DONE"; } > $L/reach2_summary.out 2>&1
