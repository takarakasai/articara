set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
BIN=target/release/examples/kyo46rs_walk
GRP=$1
A2='"MPC_NODE_CENTRE":"1","WRENCH_MPC_APPLY":"1","STAIR_WRENCH_COUPLED":"0","MPC_SOLE_AUTHORITY":"1","MPC_NET_COP_W":"10","MPC_W_SMOOTH":"10","MPC_SEED_PREVIOUS":"2"'
F=log/claude_g1combo_base.json
if [ "$GRP" = 1 ]; then
  echo "##### ホールドアウト 777: 3a なし"
  TAG=V_h_no3a BASE=$F BIN=$BIN MODE=stairs N=48 W=13 SEED=777 EXTRA="{$A2,\"MPC_COP_REF_CMD\":\"0\"}" RISERS=0.020,0.040,0.060,0.080 python3 log/claude_g1rand_20260924/rand.py
  echo "##### ホールドアウト 777: 3a あり"
  TAG=V_h_3a BASE=$F BIN=$BIN MODE=stairs N=48 W=13 SEED=777 EXTRA="{$A2,\"MPC_COP_REF_CMD\":\"1\"}" RISERS=0.020,0.040,0.060,0.080 python3 log/claude_g1rand_20260924/rand.py
else
  echo "##### 押し 8条件: 3a なし"
  TAG=V_p_no3a BASE=$F BIN=$BIN MODE=push N=48 W=13 SEED=20260924 EXTRA="{$A2,\"MPC_COP_REF_CMD\":\"0\"}" CONDS=0@0,8@0,11@0,3@270,5@270,7@90,11@180,14@180 python3 log/claude_g1rand_20260924/rand.py
  echo "##### 横段階化: 3a なし"
  TAG=V_lat_no3a BASE=$F BIN=$BIN MODE=push N=24 W=13 SEED=20260924 EXTRA="{$A2,\"MPC_COP_REF_CMD\":\"0\"}" CONDS=3@270,4@270,5@270,6@270,3@90,5@90,7@90 python3 log/claude_g1rand_20260924/rand.py
fi
echo "#### FINAL_DONE_$GRP"
