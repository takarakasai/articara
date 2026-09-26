set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
echo "##### v1 + RISE_SOLE_POSE_MIN_DZ=0.04（浅い段は切り、深い段は残す）"
TAG=RPdz_s BASE=log/claude_g1bdbaseline_20260926/controls.json BIN=target/release/examples/kyo46rs_walk MODE=stairs N=48 W=10 SEED=20260924 \
  EXTRA='{"STAIR_RISE_SOLE_POSE_MIN_DZ":"0.04"}' RISERS=0.010,0.020,0.030,0.040,0.060,0.080 python3 log/claude_g1rand_20260924/rand.py
echo "#### RPDZ_DONE"
