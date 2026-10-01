set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
C=log/claude_g1bdbaseline_20260926/controls_v6_candidate_H.json; BIN=log/claude_g1rand_20260925/kyo46rs_walk_pcfb
run () { echo "##### $1"; TAG=$2 BASE=$C BIN=$BIN MODE=push N=48 W=11 SEED=20260924 EXTRA="$3" CONDS=11@180,11@0 python3 log/claude_g1rand_20260924/rand.py; }
run "v6H + W_WRENCH 2e-3" B48_wr3 '{"MPC_W_WRENCH":"2e-3"}' &
run "v6H + W_WRENCH 2e-2" B48_wr2 '{"MPC_W_WRENCH":"2e-2"}' &
run "v6H + SOLE_AUTHORITY=0" B48_auth0 '{"MPC_SOLE_AUTHORITY":"0"}' &
wait
echo "#### B48_DONE"
