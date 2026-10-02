set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}

C=log/claude_g1champion_20260927v52/controls.json; BIN=log/claude_g1champion_20260927v52/kyo46rs_walk
echo "##### 重み付き QP 全域: std 段"; TAG=WQ_s BASE=$C BIN=$BIN MODE=stairs N=48 W=6 SEED=20260924 EXTRA='{"WBC_WEIGHTED_OFF_STAIRS":"0"}' RISERS=0.020,0.040,0.060,0.080 python3 log/claude_g1rand_20260924/rand.py
echo "#### WQ_DONE"
