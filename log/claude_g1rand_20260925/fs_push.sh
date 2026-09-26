set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
C=log/claude_g1bdbaseline_20260926/controls_v5_candidate_A.json; BIN=target/release/examples/kyo46rs_walk
run () { echo "##### $1"; TAG=$2 BASE=$C BIN=$BIN MODE=push N=24 W=13 SEED=20260924 EXTRA="$3" CONDS=8@0,11@0,5@270 python3 log/claude_g1rand_20260924/rand.py; }
run "A そのまま（箱 6/3）"           FP_base '{}'
run "A 箱 10/3"                    FP_x10  '{"MPC_FOOTSTEP_MAX_X":"0.10"}'
run "A 箱 15/5"                    FP_x15  '{"MPC_FOOTSTEP_MAX_X":"0.15","MPC_FOOTSTEP_MAX_Y":"0.05"}'
run "A 箱 15/5 + 捕捉 1e5"          FP_x15c '{"MPC_FOOTSTEP_MAX_X":"0.15","MPC_FOOTSTEP_MAX_Y":"0.05","MPC_LANDING_CAPTURE_W":"100000"}'
run "A 箱 15/5 + 着地コスト 0"        FP_x15w0 '{"MPC_FOOTSTEP_MAX_X":"0.15","MPC_FOOTSTEP_MAX_Y":"0.05","MPC_FOOTSTEP_W":"0"}'
echo "#### FS_PUSH_DONE"
