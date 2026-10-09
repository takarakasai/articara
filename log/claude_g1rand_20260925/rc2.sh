set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
L=log/claude_g1rand_20260925; C=log/claude_g1champion_20261010v66/controls.json; BIN=$L/kyo46rs_walk_rs1
X='{"REF_RESET_INCL_CURRENT":"1"}'; C8=0@0,8@0,11@0,3@270,5@270,7@90,11@180,14@180
( export MU_LO=0.40 MU_HI=0.90 JITTER=0.03 DX=0.05; TAG=RCW_p BASE=$C BIN=$BIN MODE=push N=48 W=12 SEED=20260924 EXTRA="$X" CONDS=$C8 python3 log/claude_g1rand_20260924/rand.py )
HC=11@0,5@270,6@270,7@90,11@180,5@90
TAG=ZE_hold_p BASE=$C BIN=$BIN MODE=push N=48 W=12 SEED=777 EXTRA='{}' CONDS=$HC python3 log/claude_g1rand_20260924/rand.py
TAG=RC_hold_p BASE=$C BIN=$BIN MODE=push N=48 W=12 SEED=777 EXTRA="$X" CONDS=$HC python3 log/claude_g1rand_20260924/rand.py
TAG=ZE_desc_deep STAIR_N=1 BASE=$C BIN=$BIN MODE=stairs N=48 W=12 SEED=20260924 START_X=0.40 EXTRA='{"SPAWN_Z_EXTRA":"0.005"}' RISERS=-0.100,-0.120,-0.150 python3 log/claude_g1rand_20260924/rand.py
TAG=RC_desc_deep STAIR_N=1 BASE=$C BIN=$BIN MODE=stairs N=48 W=12 SEED=20260924 START_X=0.40 EXTRA='{"SPAWN_Z_EXTRA":"0.005","REF_RESET_INCL_CURRENT":"1"}' RISERS=-0.100,-0.120,-0.150 python3 log/claude_g1rand_20260924/rand.py
{ echo "=== wide push vs v6.6"; python3 $L/mcpair.py ZE0.035W_p RCW_p push; echo "=== holdout seed 777 push vs v6.6"; python3 $L/mcpair.py ZE_hold_p RC_hold_p push; echo "=== deep descent vs v6.6"; python3 $L/mcpair.py ZE_desc_deep RC_desc_deep; echo "#### RC2_DONE"; } > $L/rc2_summary.out 2>&1
