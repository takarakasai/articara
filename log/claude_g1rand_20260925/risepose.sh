set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
GRP=$1; V1=log/claude_g1bdbaseline_20260926/controls.json; BIN=target/release/examples/kyo46rs_walk
if [ "$GRP" = low ]; then
  echo "##### 低い段の掃引 v1 + RISE_SOLE_POSE=0"; TAG=RP_low BASE=$V1 BIN=$BIN MODE=stairs N=48 W=13 SEED=20260924 EXTRA='{"STAIR_RISE_SOLE_POSE":"0"}' RISERS=0.000,0.010,0.020,0.030 python3 log/claude_g1rand_20260924/rand.py
  echo "#### RP_LOW_DONE"
else
  W=13 scripts/g1_battery.sh RP $V1 '{"STAIR_RISE_SOLE_POSE":"0"}' std
  echo "#### RP_STD_DONE"
fi
