set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
L=log/claude_g1rand_20260925; C=log/claude_g1champion_20261010v67/controls.json; BIN=$L/kyo46rs_walk_rh3
for m in 0.95 0.96 0.97; do for ml in 0.3 0.5; do
TAG=RH${m}_${ml}_p BASE=$C BIN=$BIN MODE=push N=48 W=12 SEED=20260924 EXTRA="{\"MPC_REACH\":\"$m\",\"MPC_REACH_MIN_LOAD\":\"$ml\",\"PUSH_Z_DROP\":\"0\"}" CONDS=11@180,11@0 python3 log/claude_g1rand_20260924/rand.py
done; done
{ for m in 0.95 0.96 0.97; do for ml in 0.3 0.5; do echo "=== MPC_REACH $m MIN_LOAD $ml (PUSH_Z_DROP off) vs v6.7"; python3 $L/mcpair.py RC_p RH${m}_${ml}_p push; done; done; echo "#### REACH1_DONE"; } > $L/reach1_summary.out 2>&1
