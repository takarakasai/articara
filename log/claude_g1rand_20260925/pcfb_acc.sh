set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
C=log/claude_g1bdbaseline_20260926/controls_v6_candidate_pcfb.json; BIN=log/claude_g1rand_20260925/kyo46rs_walk_pcfb
for a in 60 600; do
  echo "##### v6 + MPC_W_ACC=$a: 押し 8 条件"; TAG=PCFBA${a}_p BASE=$C BIN=$BIN MODE=push N=24 W=10 SEED=20260924 EXTRA="{\"MPC_W_ACC\":\"$a\"}" CONDS=0@0,8@0,11@0,3@270,5@270,7@90,11@180,14@180 python3 log/claude_g1rand_20260924/rand.py
done
echo "#### PCFBA_DONE"
