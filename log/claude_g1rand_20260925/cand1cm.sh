set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
P=$1; EXTRA=$2; V4=log/claude_g1champion_20260926v4/controls.json; BIN=target/release/examples/kyo46rs_walk
W=13 scripts/g1_battery.sh $P $V4 "$EXTRA" std
echo "##### $P 低い段"; TAG=${P}_low BASE=$V4 BIN=$BIN MODE=stairs N=48 W=13 SEED=20260924 EXTRA="$EXTRA" RISERS=0.000,0.010,0.020,0.030 python3 log/claude_g1rand_20260924/rand.py
echo "#### CAND_DONE_$P"
