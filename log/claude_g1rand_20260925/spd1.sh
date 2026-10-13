set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
L=log/claude_g1rand_20260925; C=log/claude_g1champion_20261013v69/controls.json; BIN=log/claude_g1champion_20261013v69/kyo46rs_walk
CS="VX2=0.30;VX2_AT=8,VX2=0.35;VX2_AT=8,VX2=0.45;VX2_AT=8,VX2=0.50;VX2_AT=8,VX2=0.55;VX2_AT=8"
TAG=ST35A_speed BASE=$C BIN=$BIN MODE=flat N=48 W=12 SEED=20260924 T=40 FLAT_MIN_STEPS=24 FLAT_START_X=30 EXTRA='{"VX2_STRIDE_MAX":"0.35","VX2_Z_DROP":"0.024"}' CONDS="$CS" python3 log/claude_g1rand_20260924/rand.py
TAG=ST35B_speed BASE=$C BIN=$BIN MODE=flat N=48 W=12 SEED=20260924 T=40 FLAT_MIN_STEPS=24 FLAT_START_X=30 EXTRA='{"VX2_STRIDE_MAX":"0.35","VX2_Z_DROP":"0.040"}' CONDS="$CS" python3 log/claude_g1rand_20260924/rand.py
TAG=ST25_speed BASE=$C BIN=$BIN MODE=flat N=48 W=12 SEED=20260924 T=40 FLAT_MIN_STEPS=24 FLAT_START_X=30 EXTRA='{"VX2_STRIDE_MAX":"0.25","VX2_Z_DROP":"0.024"}' CONDS="VX2=0.55;VX2_AT=8" python3 log/claude_g1rand_20260924/rand.py
{ echo "=== stride 0.35 zdrop 0.024 vs stride 0.25 zdrop 0.024 (v6.4-era baseline VZ0.024)"; python3 $L/mcpair.py VZ0.024_speed ST35A_speed; echo "=== stride 0.35 zdrop 0.040"; python3 $L/mcpair.py VZ0.024_speed ST35B_speed; echo "=== 0.55 with stride 0.25 (v6.9)"; for c in log/claude_g1rand_20260924/ST25_speed/*/; do echo "$(basename $c): $(cat $c/m*/run.log | grep -c SURVIVED)/48"; done; echo "#### SPD1_DONE"; } > $L/spd1_summary.out 2>&1
