set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
C=log/claude_g1champion_20260927v52/controls.json; BIN=log/claude_g1champion_20260927v52/kyo46rs_walk
run () { ( echo "##### $1"; TAG=$2 BASE=$C BIN=$BIN MODE=push N=48 W=5 SEED=20260924 EXTRA="$3" CONDS=7@90,5@270 python3 log/claude_g1rand_20260924/rand.py ) & }
run "内側 5 cm"  LAT_in5  '{"MPC_FOOTSTEP_MAX_Y_IN":"0.05"}'
run "内側 8 cm"  LAT_in8  '{"MPC_FOOTSTEP_MAX_Y_IN":"0.08"}'
run "内側 12 cm" LAT_in12 '{"MPC_FOOTSTEP_MAX_Y_IN":"0.12"}'
wait
echo "#### LAT_DONE"
