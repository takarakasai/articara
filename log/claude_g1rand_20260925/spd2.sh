set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
L=log/claude_g1rand_20260925; C=log/claude_g1champion_20261013v69/controls.json; BIN=log/claude_g1champion_20261013v69/kyo46rs_walk
CS="VX2=0.45;VX2_AT=8,VX2=0.50;VX2_AT=8,VX2=0.55;VX2_AT=8,VX2=0.60;VX2_AT=8"
TAG=ST45_speed BASE=$C BIN=$BIN MODE=flat N=48 W=12 SEED=20260924 T=40 FLAT_MIN_STEPS=24 FLAT_START_X=30 EXTRA='{"VX2_STRIDE_MAX":"0.45","VX2_Z_DROP":"0.060"}' CONDS="$CS" python3 log/claude_g1rand_20260924/rand.py
TAG=ST40_speed BASE=$C BIN=$BIN MODE=flat N=48 W=12 SEED=20260924 T=40 FLAT_MIN_STEPS=24 FLAT_START_X=30 EXTRA='{"VX2_STRIDE_MAX":"0.40","VX2_Z_DROP":"0.060"}' CONDS="$CS" python3 log/claude_g1rand_20260924/rand.py
{ for tg in ST45_speed ST40_speed; do echo "== $tg"; for c in log/claude_g1rand_20260924/$tg/*/; do echo "  $(basename $c): $(cat $c/m*/run.log | grep -c SURVIVED)/48"; done; done; echo "#### SPD2_DONE"; } > $L/spd2_summary.out 2>&1
