set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
C=log/claude_g1bdbaseline_20260926/controls_v6_candidate_H.json; BIN=log/claude_g1rand_20260925/kyo46rs_walk_pcfb
for w in 2e-3 2e-2; do
  echo "##### v6H + MPC_W_WRENCH=$w: 押し 8 条件"; TAG=PCFBWR${w}_p BASE=$C BIN=$BIN MODE=push N=24 W=13 SEED=20260924 EXTRA="{\"MPC_W_WRENCH\":\"$w\"}" CONDS=0@0,8@0,11@0,3@270,5@270,7@90,11@180,14@180 python3 log/claude_g1rand_20260924/rand.py
done
echo "#### PCFBWR_DONE"
