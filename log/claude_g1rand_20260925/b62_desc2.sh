set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
L=log/claude_g1rand_20260925; C=log/claude_g1champion_20261003v62/controls.json; BIN=log/claude_g1champion_20261003v62/kyo46rs_walk
[ "$(grep -ac COM_HEIGHT_DESC_LEAD $BIN)" -ge 1 ] || { echo "binary lacks knob"; exit 1; }
X='{"SPAWN_Z_EXTRA":"0.005"}'
echo "##### v6.2 深い 1 段"; TAG=B62_desc_deep BASE=$C BIN=$BIN MODE=stairs N=48 W=10 SEED=20260924 START_X=0.40 EXTRA="$X" RISERS=-0.100,-0.120,-0.150 python3 log/claude_g1rand_20260924/rand.py
for n in 2 3; do echo "##### v6.2 連続降段 n=$n"; TAG=B62_desc_n$n STAIR_N=$n BASE=$C BIN=$BIN MODE=stairs N=48 W=10 SEED=20260924 START_X=0.40 EXTRA="$X" RISERS=-0.040,-0.060,-0.080 python3 log/claude_g1rand_20260924/rand.py; done
echo "#### B62_DESC2_DONE"
