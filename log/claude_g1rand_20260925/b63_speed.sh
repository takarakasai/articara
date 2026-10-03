set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
L=log/claude_g1rand_20260925; C=log/claude_g1champion_20261003v63/controls.json; BIN=log/claude_g1champion_20261003v63/kyo46rs_walk
[ "$(grep -ac VX2_RAMP $BIN)" -ge 1 ] || { echo "binary lacks knob"; exit 1; }
TAG=B63_speed BASE=$C BIN=$BIN MODE=flat N=48 W=12 SEED=20260924 T=40 FLAT_MIN_STEPS=24 EXTRA='{}' CONDS="VX2=0.16;VX2_AT=8,VX2=0.20;VX2_AT=8,VX2=0.25;VX2_AT=8,VX2=0.30;VX2_AT=8,VX2=0.35;VX2_AT=8,VX2=0.0;VX2_AT=8,VX2=0.06;VX2_AT=8" python3 log/claude_g1rand_20260924/rand.py
echo "#### B63_SPEED_DONE"
