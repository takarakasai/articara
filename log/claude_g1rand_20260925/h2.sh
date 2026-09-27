set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
export MU_LO=0.40 MU_HI=0.90 JITTER=0.03 DX=0.05
BIN=log/claude_g1rand_20260925/kyo46rs_walk_h2
V=log/claude_g1champion_20260927v41/controls.json; D=log/claude_g1bdbaseline_20260926/controls_v5_candidate_DJ.json
( TAG=TMW_h2 BASE=$V BIN=$BIN MODE=stairs N=48 W=13 SEED=778 EXTRA='{}' RISERS=0.020,0.040,0.060,0.080 python3 log/claude_g1rand_20260924/rand.py ) &
( TAG=DJW_h2 BASE=$D BIN=$BIN MODE=stairs N=48 W=13 SEED=778 EXTRA='{}' RISERS=0.020,0.040,0.060,0.080 python3 log/claude_g1rand_20260924/rand.py ) &
wait
echo "#### H2_DONE"
