set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
C=log/claude_g1bdbaseline_20260926/controls_v5_candidate_D.json; BIN=log/claude_g1rand_20260925/kyo46rs_walk_ftprobe
echo "##### V5D 低い段"; TAG=V5D_low BASE=$C BIN=$BIN MODE=stairs N=48 W=13 SEED=20260924 EXTRA='{}' RISERS=0.000,0.010,0.020,0.030 python3 log/claude_g1rand_20260924/rand.py
echo "#### V5DLOW_DONE"
