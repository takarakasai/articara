set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
C=log/claude_g1champion_20260927v52/controls.json; BIN=log/claude_g1rand_20260925/kyo46rs_walk_capn
run () { ( echo "##### $1"; TAG=$2 BASE=$C BIN=$BIN MODE=push N=48 W=6 SEED=20260924 EXTRA="$3" CONDS=11@180,11@0 python3 log/claude_g1rand_20260924/rand.py ) & }
run "捕捉地平 16"          CN16_p    '{"MPC_LANDING_CAPTURE_N":"16"}'
run "捕捉地平 24"          CN24_p    '{"MPC_LANDING_CAPTURE_N":"24"}'
run "捕捉地平 24 + 箱 0.25" CN24x25_p '{"MPC_LANDING_CAPTURE_N":"24","MPC_FOOTSTEP_MAX_X":"0.25"}'
wait
echo "#### CAPN_DONE"
