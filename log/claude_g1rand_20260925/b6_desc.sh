set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
L=log/claude_g1rand_20260925; C=log/claude_g1champion_20261003v6/controls.json; BIN=$L/kyo46rs_walk_sense
echo "##### v6 降段基準"; TAG=B6_desc BASE=$C BIN=$BIN MODE=stairs N=48 W=10 SEED=20260924 START_X=0.40 EXTRA='{"SPAWN_Z_EXTRA":"0.005"}' RISERS=-0.020,-0.040,-0.060,-0.080 python3 log/claude_g1rand_20260924/rand.py
echo "#### B6_DESC_DONE"
