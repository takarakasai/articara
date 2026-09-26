set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
BIN=target/release/examples/kyo46rs_walk
V31=log/claude_g1champion_20260924v31/controls.json
A='"CP_TRACK":"1","CP_W_SHIFT":"40","MPC_NODE_CENTRE":"1","WRENCH_MPC_APPLY":"1","STAIR_WRENCH_COUPLED":"0","MPC_SOLE_AUTHORITY":"1","MPC_NET_COP_W":"10","MPC_W_SMOOTH":"10"'
echo "##### ステップ1: 溶接 v3.1 + 筋A 押し 8条件"
TAG=R_A_p BASE=$V31 BIN=$BIN MODE=push N=48 W=12 SEED=20260924 EXTRA="{$A}" \
  CONDS=0@0,8@0,11@0,3@270,5@270,7@90,11@180,14@180 python3 log/claude_g1rand_20260924/rand.py
echo "##### ステップ1: 横の段階化"
TAG=R_A_lat BASE=$V31 BIN=$BIN MODE=push N=24 W=12 SEED=20260924 EXTRA="{$A}" \
  CONDS=3@270,4@270,5@270,6@270,3@90,5@90,7@90 python3 log/claude_g1rand_20260924/rand.py
echo "#### STEP1DONE"
