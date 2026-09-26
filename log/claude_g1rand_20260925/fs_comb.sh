set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
A=log/claude_g1bdbaseline_20260926/controls_v5_candidate_A.json; BIN=log/claude_g1rand_20260925/kyo46rs_walk_m0probe
run () { echo "##### $1"; TAG=$2 BASE=$A BIN=$BIN MODE=push N=24 W=13 SEED=20260924 EXTRA="$3" CONDS=8@0,11@0,5@270,7@90 python3 log/claude_g1rand_20260924/rand.py; }
B='"MPC_FOOTSTEP_MAX_X":"0.15","MPC_FOOTSTEP_MAX_Y":"0.05","MPC_LANDING_CAPTURE_W":"100000"'
run "C1: A + 箱 15/5 + 捕捉 1e5"                 FX_c1 "{$B}"
run "C2: C1 + N=8"                              FX_c2 "{$B,\"STAIR_WRENCH_N\":\"8\"}"
run "C3: C2 + REF_CMD=0"                        FX_c3 "{$B,\"STAIR_WRENCH_N\":\"8\",\"MPC_COP_REF_CMD\":\"0\"}"
run "C4: C2 + 箱 y 8 cm"                         FX_c4 "{$B,\"STAIR_WRENCH_N\":\"8\",\"MPC_FOOTSTEP_MAX_Y\":\"0.08\"}"
run "C5: C2 + MOMENT=0"                         FX_c5 "{$B,\"STAIR_WRENCH_N\":\"8\",\"MPC_FOOTSTEP_MOMENT\":\"0\"}"
echo "#### FS_COMB_DONE"
