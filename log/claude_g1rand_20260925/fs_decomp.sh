set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
V=log/claude_g1champion_20260927v41/controls.json; A=log/claude_g1bdbaseline_20260926/controls_v5_candidate_A.json; BIN=target/release/examples/kyo46rs_walk
run () { echo "##### $1"; TAG=$2 BASE=$3 BIN=$BIN MODE=push N=24 W=8 SEED=20260924 EXTRA="$4" CONDS=8@0,5@270 python3 log/claude_g1rand_20260924/rand.py; }
run "v4.1 + MPC_COP_REF_CMD=1"            FD_v41_cmd  $V '{"MPC_COP_REF_CMD":"1"}'
run "v4.1 + STAIR_WRENCH_N=16"            FD_v41_n16  $V '{"STAIR_WRENCH_N":"16"}'
run "v4.1 + CONTACT_PLAN=0（発見的調整なし）" FD_v41_cp0  $V '{"CONTACT_PLAN":"0"}'
run "A + MPC_COP_REF_CMD=0"               FD_A_cmd0   $A '{"MPC_COP_REF_CMD":"0"}'
run "A + MPC_COP_REF_CMD=0 + 箱 15/5"      FD_A_cmd0x15 $A '{"MPC_COP_REF_CMD":"0","MPC_FOOTSTEP_MAX_X":"0.15","MPC_FOOTSTEP_MAX_Y":"0.05"}'
echo "#### FS_DECOMP_DONE"
