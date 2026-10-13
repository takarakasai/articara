set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
L=log/claude_g1rand_20260925; C=log/claude_g1champion_20261011v68/controls.json; BIN=$L/kyo46rs_walk_hs2
TAG=HSN_p BASE=$C BIN=$BIN MODE=push N=48 W=12 SEED=20260924 EXTRA='{"MOM_CMP":"1","MOM_CMP_SIGN":"-1","MOM_CMP_MAX":"20"}' CONDS=11@180,11@0,7@90,5@270 python3 log/claude_g1rand_20260924/rand.py
TAG=HSNH_p BASE=$C BIN=$BIN MODE=push N=48 W=12 SEED=20260924 EXTRA='{"MOM_CMP":"1","MOM_CMP_SIGN":"-1","MOM_CMP_MAX":"20","MOM_CMP_HMAX":"1","MOM_CMP_KH":"2"}' CONDS=11@180,11@0,7@90,5@270 python3 log/claude_g1rand_20260924/rand.py
{ echo "=== MOM_CMP sign -1 max 20 vs v6.8"; python3 $L/mcpair.py DCX_p HSN_p push; echo "=== + HMAX 1 KH 2 vs v6.8"; python3 $L/mcpair.py DCX_p HSNH_p push; echo "#### HS1_DONE"; } > $L/hs1_summary.out 2>&1
