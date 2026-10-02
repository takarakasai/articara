set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
C=log/claude_g1champion_20260927v52/controls.json; BIN=log/claude_g1rand_20260925/kyo46rs_walk_floor
run () { ( echo "##### $1"; TAG=$2 BASE=$C BIN=$BIN MODE=push N=48 W=6 SEED=20260924 EXTRA="$3" CONDS=11@180,11@0 python3 log/claude_g1rand_20260924/rand.py ) & }
run "捕捉床 0.5 箱 0.25" FL05x25_p '{"MPC_FOOTSTEP_CAPTURE_FLOOR":"0.5","MPC_FOOTSTEP_MAX_X":"0.25"}'
run "捕捉床 0.8 箱 0.25" FL08x25_p '{"MPC_FOOTSTEP_CAPTURE_FLOOR":"0.8","MPC_FOOTSTEP_MAX_X":"0.25"}'
run "捕捉床 0.5 箱 0.15" FL05x15_p '{"MPC_FOOTSTEP_CAPTURE_FLOOR":"0.5"}'
wait
echo "#### FLOOR_DONE"
