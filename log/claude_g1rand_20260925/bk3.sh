set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
C=log/claude_g1champion_20260927v52/controls.json; BIN=log/claude_g1rand_20260925/kyo46rs_walk_latemg
run () { ( echo "##### $1"; TAG=$2 BASE=$C BIN=$BIN MODE=push N=48 W=8 SEED=20260924 EXTRA="$3" CONDS=11@180 python3 log/claude_g1rand_20260924/rand.py ) & }
run "延長可 箱0.35"           BK3_ext    '{"CP_T_MAX":"1.5","MPC_FOOTSTEP_MAX_X":"0.35"}'
run "延長可 箱0.35 捕捉床0.5"  BK3_extfl  '{"CP_T_MAX":"1.5","MPC_FOOTSTEP_MAX_X":"0.35","MPC_FOOTSTEP_CAPTURE_FLOOR":"0.5"}'
run "箱0.35 捕捉床0.5（延長なし）" BK3_fl   '{"MPC_FOOTSTEP_MAX_X":"0.35","MPC_FOOTSTEP_CAPTURE_FLOOR":"0.5"}'
wait
echo "#### BK3_DONE"
