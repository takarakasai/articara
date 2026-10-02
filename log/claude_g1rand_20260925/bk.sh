set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
C=log/claude_g1champion_20260927v52/controls.json; BIN=log/claude_g1champion_20260927v52/kyo46rs_walk
run () { ( echo "##### $1"; TAG=$2 BASE=$C BIN=$BIN MODE=push N=48 W=8 SEED=20260924 EXTRA="$3" CONDS=11@180 python3 log/claude_g1rand_20260924/rand.py ) & }
run "箱 x 0.20"            BK_x20   '{"MPC_FOOTSTEP_MAX_X":"0.20"}'
run "箱 x 0.25"            BK_x25   '{"MPC_FOOTSTEP_MAX_X":"0.25"}'
run "CP_T_MIN 0.3"         BK_tmin  '{"CP_T_MIN":"0.3"}'
run "箱 0.25 + CP_T_MIN 0.3" BK_both '{"MPC_FOOTSTEP_MAX_X":"0.25","CP_T_MIN":"0.3"}'
wait
echo "#### BK_DONE"
