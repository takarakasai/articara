set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
L=log/claude_g1rand_20260925; C=log/claude_g1champion_20261013v69/controls.json; BIN=$L/kyo46rs_walk_hs3
C8=0@0,8@0,11@0,3@270,5@270,7@90,11@180,14@180
TAG=RET_p BASE=$C BIN=$BIN MODE=push N=48 W=12 SEED=20260924 EXTRA='{"PUSH_Z_DROP":"0"}' CONDS=$C8 python3 log/claude_g1rand_20260924/rand.py
( export MU_LO=0.40 MU_HI=0.90 JITTER=0.03 DX=0.05; TAG=RETW_p BASE=$C BIN=$BIN MODE=push N=48 W=12 SEED=20260924 EXTRA='{"PUSH_Z_DROP":"0"}' CONDS=$C8 python3 log/claude_g1rand_20260924/rand.py )
{ echo "=== v6.9 without PUSH_Z_DROP vs v6.9 (std)"; python3 $L/mcpair.py HSX_p RET_p push; echo "=== wide"; python3 $L/mcpair.py HSXW_p RETW_p push; echo "#### RET1_DONE"; } > $L/ret1_summary.out 2>&1
