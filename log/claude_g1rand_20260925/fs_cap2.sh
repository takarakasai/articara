set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
A=log/claude_g1bdbaseline_20260926/controls_v5_candidate_A.json; BIN=log/claude_g1rand_20260925/kyo46rs_walk_capprobe
run () { echo "##### $1"; TAG=$2 BASE=$A BIN=$BIN MODE=push N=24 W=5 SEED=20260924 EXTRA="$3" CONDS=8@0,5@270 python3 log/claude_g1rand_20260924/rand.py; }
run "A − REF_CMD、捕捉 0（着地コストのみ）" FE_c0   '{"MPC_COP_REF_CMD":"0","MPC_LANDING_CAPTURE_W":"0"}'
run "A − REF_CMD、捕捉 1e3"               FE_c1e3 '{"MPC_COP_REF_CMD":"0","MPC_LANDING_CAPTURE_W":"1000"}'
run "A − REF_CMD、N=8"                    FE_n8   '{"MPC_COP_REF_CMD":"0","STAIR_WRENCH_N":"8"}'
run "A − REF_CMD、着地コスト 1e3"          FE_w1e3 '{"MPC_COP_REF_CMD":"0","MPC_FOOTSTEP_W":"1000"}'
echo "#### FS_CAP2_DONE"
