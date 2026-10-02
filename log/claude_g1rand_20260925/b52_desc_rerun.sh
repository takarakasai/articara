set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
C=log/claude_g1champion_20260927v52/controls.json; BIN=log/claude_g1rand_20260925/kyo46rs_walk_bench2
echo "##### B52 降段（SPAWN_Z_EXTRA 0.005、panic 分を再走）"; TAG=B52_desc BASE=$C BIN=$BIN MODE=stairs N=48 W=4 SEED=20260924 START_X=0.40 EXTRA='{"SPAWN_Z_EXTRA":"0.005"}' RISERS=-0.020,-0.040,-0.060 python3 log/claude_g1rand_20260924/rand.py
echo "#### B52_DESC_DONE"
