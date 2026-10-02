set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
C=log/claude_g1champion_20260927v52/controls.json; BIN=log/claude_g1champion_20260927v52/kyo46rs_walk
( echo "##### 内側 15 cm（MAX_Y 0.15）"; TAG=LAT2_in15 BASE=$C BIN=$BIN MODE=push N=48 W=6 SEED=20260924 EXTRA='{"MPC_FOOTSTEP_MAX_Y_IN":"0.15","MPC_FOOTSTEP_MAX_Y":"0.15"}' CONDS=7@90 python3 log/claude_g1rand_20260924/rand.py ) &
( echo "##### 外側 15 cm のみ（MAX_Y 0.15）"; TAG=LAT2_out15 BASE=$C BIN=$BIN MODE=push N=48 W=6 SEED=20260924 EXTRA='{"MPC_FOOTSTEP_MAX_Y":"0.15"}' CONDS=7@90 python3 log/claude_g1rand_20260924/rand.py ) &
wait
echo "#### LAT2_DONE"
