set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
P=$1; C=log/claude_g1bdbaseline_20260926/controls_v5_candidate_$1.json; BIN=target/release/examples/kyo46rs_walk
W=13 scripts/g1_battery.sh V5$P $C '{}' std
W=13 scripts/g1_battery.sh V5$P $C '{}' wide
echo "##### V5$P 低い段"; TAG=V5${P}_low BASE=$C BIN=$BIN MODE=stairs N=48 W=13 SEED=20260924 EXTRA='{}' RISERS=0.000,0.010,0.020,0.030 python3 log/claude_g1rand_20260924/rand.py
echo "#### V5CAND_DONE_$P"
