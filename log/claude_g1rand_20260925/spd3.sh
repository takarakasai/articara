set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
L=log/claude_g1rand_20260925; C=log/claude_g1champion_20261013v69/controls.json; BIN=$L/kyo46rs_walk_sp3
CS="VX2=0.20;VX2_AT=8,VX2=0.30;VX2_AT=8,VX2=0.35;VX2_AT=8,VX2=0.40;VX2_AT=8,VX2=0.45;VX2_AT=8,VX2=0.50;VX2_AT=8,VX2=0.55;VX2_AT=8"
TAG=SCH_speed BASE=$C BIN=$BIN MODE=flat N=48 W=12 SEED=20260924 T=40 FLAT_MIN_STEPS=24 FLAT_START_X=30 EXTRA='{"VX2_SCHED":"1"}' CONDS="$CS" python3 log/claude_g1rand_20260924/rand.py
{ echo "== SCH_speed (VX2_SCHED=1)"; for c in log/claude_g1rand_20260924/SCH_speed/*/; do echo "  $(basename $c): $(cat $c/m*/run.log | grep -c SURVIVED)/48"; done; echo "#### SPD3_DONE"; } > $L/spd3_summary.out 2>&1
