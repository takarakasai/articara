set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
L=log/claude_g1rand_20260925; C=log/claude_g1champion_20261003v63/controls.json; BIN=$L/kyo46rs_walk_speed2
[ "$(grep -ac VX2_STRIDE_MAX $BIN)" -ge 1 ] || { echo "binary lacks knob"; exit 1; }
SP="VX2=0.20;VX2_AT=8,VX2=0.25;VX2_AT=8,VX2=0.30;VX2_AT=8,VX2=0.35;VX2_AT=8,VX2=0.45;VX2_AT=8,VX2=0.55;VX2_AT=8"
echo "##### 基準（遠い段、歩調固定）"; TAG=F63_speed FLAT_START_X=30 BASE=$C BIN=$BIN MODE=flat N=48 W=12 SEED=20260924 T=40 FLAT_MIN_STEPS=24 EXTRA='{}' CONDS="$SP" python3 log/claude_g1rand_20260924/rand.py
for sm in 0.20 0.25; do echo "##### 歩幅上限 $sm"; TAG=FC${sm}_speed FLAT_START_X=30 BASE=$C BIN=$BIN MODE=flat N=48 W=12 SEED=20260924 T=40 FLAT_MIN_STEPS=24 EXTRA="{\"VX2_STRIDE_MAX\":\"$sm\"}" CONDS="$SP" python3 log/claude_g1rand_20260924/rand.py; done
echo "#### SPEED2_DONE"
