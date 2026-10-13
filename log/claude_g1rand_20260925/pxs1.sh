set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
L=log/claude_g1rand_20260925; C=log/claude_g1champion_20261013v69/controls.json; BIN=$L/kyo46rs_walk_sp3
export PUSH_STEP_OFFSET=6 PUSH_STAIR_X=30 T=30
CP=11@0,11@180,7@90,5@270
TAG=PXS012_p BASE=$C BIN=$BIN MODE=push N=48 W=12 SEED=20260924 EXTRA='{}' CONDS=$CP python3 log/claude_g1rand_20260924/rand.py
TAG=PXS030_p BASE=$C BIN=$BIN MODE=push N=48 W=12 SEED=20260924 EXTRA='{"VX2_SCHED":"1","VX2":"0.30","VX2_AT":"8"}' CONDS=$CP python3 log/claude_g1rand_20260924/rand.py
TAG=PXS050_p BASE=$C BIN=$BIN MODE=push N=48 W=12 SEED=20260924 EXTRA='{"VX2_SCHED":"1","VX2":"0.50","VX2_AT":"8"}' CONDS=$CP python3 log/claude_g1rand_20260924/rand.py
{ for tg in PXS012_p PXS030_p PXS050_p; do echo "== $tg"; for c in log/claude_g1rand_20260924/$tg/*/; do echo "  $(basename $c): $(cat $c/m*/run.log | grep -c SURVIVED)/48 (fired $(cat $c/m*/run.log | grep -c 'PUSH at'))"; done; done; echo "=== 0.30 vs 0.12"; python3 $L/mcpair.py PXS012_p PXS030_p push; echo "=== 0.50 vs 0.12"; python3 $L/mcpair.py PXS012_p PXS050_p push; echo "#### PXS1_DONE"; } > $L/pxs1_summary.out 2>&1
