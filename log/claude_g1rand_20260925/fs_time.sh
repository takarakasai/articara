set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
A=log/claude_g1bdbaseline_20260926/controls_v5_candidate_A.json; BIN=log/claude_g1rand_20260925/kyo46rs_walk_ftprobe
run () { echo "##### $1"; TAG=$2 BASE=$A BIN=$BIN MODE=push N=24 W=13 SEED=20260924 EXTRA="$3" CONDS=8@0,11@0,5@270,7@90 python3 log/claude_g1rand_20260924/rand.py; }
B='"MPC_FOOTSTEP_MAX_X":"0.15","MPC_FOOTSTEP_MAX_Y":"0.08","MPC_LANDING_CAPTURE_W":"100000","STAIR_WRENCH_N":"8"'
T='"CONTACT_PLAN":"1","CP_MAX_X":"0","CP_MAX_Y":"0","CP_MAX_Y_IN":"0"'
run "FT1: C4 + 時間調整専用 contact_plan"            FT_1 "{$B,$T}"
run "FT2: FT1 + REF_CMD=0"                          FT_2 "{$B,$T,\"MPC_COP_REF_CMD\":\"0\"}"
run "FT3: FT1、CP_T_MIN=0.4（より短縮可）"            FT_3 "{$B,$T,\"CP_T_MIN\":\"0.4\"}"
echo "#### FS_TIME_DONE"
