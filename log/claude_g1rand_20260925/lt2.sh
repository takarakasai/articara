set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
C=log/claude_g1champion_20260927v52/controls.json; BIN=log/claude_g1rand_20260925/kyo46rs_walk_ltime5
( echo "##### TIME N=16"; TAG=LT48N16_p BASE=$C BIN=$BIN MODE=push N=48 W=10 SEED=20260924 EXTRA='{"MPC_LANDING_TIME":"1","CONTACT_PLAN":"0","STAIR_WRENCH_N":"16"}' CONDS=11@0 python3 log/claude_g1rand_20260924/rand.py ) &
( echo "##### TIME N=16 両軸"; TAG=LT48N16XY_p BASE=$C BIN=$BIN MODE=push N=48 W=10 SEED=20260924 EXTRA='{"MPC_LANDING_TIME":"1","CONTACT_PLAN":"0","STAIR_WRENCH_N":"16","MPC_LANDING_TIME_FROM_X":"0"}' CONDS=11@0 python3 log/claude_g1rand_20260924/rand.py ) &
( echo "##### TIME 利得なし 箱0.08"; TAG=LT48G0_p BASE=$C BIN=$BIN MODE=push N=48 W=10 SEED=20260924 EXTRA='{"MPC_LANDING_TIME":"1","CONTACT_PLAN":"0","MPC_LANDING_TIME_GAIN":"0","MPC_FOOTSTEP_MAX_X":"0.08"}' CONDS=11@0 python3 log/claude_g1rand_20260924/rand.py ) &
wait
echo "#### LT2_DONE"
