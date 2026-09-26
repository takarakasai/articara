set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
C=log/claude_g1bdbaseline_20260926/controls_v5_candidate_C.json; BIN=log/claude_g1rand_20260925/kyo46rs_walk_ftprobe
run () { echo "##### $1"; TAG=$2 BASE=$C BIN=$BIN MODE=push N=24 W=6 SEED=20260924 EXTRA="$3" CONDS=11@0 python3 log/claude_g1rand_20260924/rand.py; }
run "C + 箱 x 20 cm"        F11_x20  '{"MPC_FOOTSTEP_MAX_X":"0.20"}'
run "C + 捕捉 1e6"          F11_c1e6 '{"MPC_LANDING_CAPTURE_W":"1000000"}'
run "C + CP_T_MIN=0.4"      F11_tmin '{"CP_T_MIN":"0.4"}'
run "C + 着地コスト 10"      F11_w10  '{"MPC_FOOTSTEP_W":"10"}'
echo "#### FS_11_DONE"
