set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
export MU_LO=0.40 MU_HI=0.90 JITTER=0.03 DX=0.05
BIN=log/claude_g1rand_20260925/kyo46rs_walk_sbprobe
V=log/claude_g1champion_20260927v41/controls.json; DJ=log/claude_g1bdbaseline_20260926/controls_v5_candidate_DJ.json
( TAG=NULLW_h BASE=$V BIN=$BIN MODE=stairs N=48 W=13 SEED=777 EXTRA='{"MPC_W_SMOOTH":"10.001"}' RISERS=0.020,0.040,0.060,0.080 python3 log/claude_g1rand_20260924/rand.py > log/claude_g1rand_20260925/ctrl_null.out 2>&1 ) &
( TAG=DJ0W_h BASE=$DJ BIN=$BIN MODE=stairs N=48 W=13 SEED=777 EXTRA='{"MPC_FOOTSTEP_MAX_X":"0","MPC_FOOTSTEP_MAX_Y":"0"}' RISERS=0.020,0.040,0.060,0.080 python3 log/claude_g1rand_20260924/rand.py > log/claude_g1rand_20260925/ctrl_dj0.out 2>&1 ) &
wait
echo "#### CTRL_DONE"
