set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
BIN=target/release/examples/kyo46rs_walk
echo "##### 低い段の掃引 v1"; TAG=LR_v1 BASE=log/claude_g1bdbaseline_20260926/controls.json BIN=$BIN MODE=stairs N=48 W=13 SEED=20260924 EXTRA='{}' RISERS=0.000,0.010,0.020,0.030 python3 log/claude_g1rand_20260924/rand.py
echo "##### 低い段の掃引 v3.1"; TAG=LR_v31 BASE=log/claude_g1champion_20260924v31/controls.json BIN=$BIN MODE=stairs N=48 W=13 SEED=20260924 EXTRA='{"CP_TRACK":"1","CP_W_SHIFT":"40"}' RISERS=0.000,0.010,0.020,0.030 python3 log/claude_g1rand_20260924/rand.py
echo "#### LOWRISER_DONE"
