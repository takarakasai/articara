set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
BIN=target/release/examples/kyo46rs_walk
GRP=$1
A2='"MPC_NODE_CENTRE":"1","WRENCH_MPC_APPLY":"1","STAIR_WRENCH_COUPLED":"0","MPC_SOLE_AUTHORITY":"1","MPC_NET_COP_W":"10","MPC_W_SMOOTH":"10","MPC_SEED_PREVIOUS":"2"'
F=log/claude_g1combo_base.json; W31=log/claude_g1champion_20260924v31/controls.json
if [ "$GRP" = 1 ]; then
  echo "##### 自由腕 筋A SEED=2 +3a  押し 8条件"
  TAG=U_f_p BASE=$F BIN=$BIN MODE=push N=48 W=13 SEED=20260924 EXTRA="{$A2,\"MPC_COP_REF_CMD\":\"1\"}" \
    CONDS=0@0,8@0,11@0,3@270,5@270,7@90,11@180,14@180 python3 log/claude_g1rand_20260924/rand.py
  echo "##### 同 横段階化"
  TAG=U_f_lat BASE=$F BIN=$BIN MODE=push N=24 W=13 SEED=20260924 EXTRA="{$A2,\"MPC_COP_REF_CMD\":\"1\"}" \
    CONDS=3@270,4@270,5@270,6@270,3@90,5@90,7@90 python3 log/claude_g1rand_20260924/rand.py
else
  echo "##### 溶接 筋A SEED=2 +3a  段"
  TAG=U_w_s BASE=$W31 BIN=$BIN MODE=stairs N=48 W=13 SEED=20260924 EXTRA="{\"CP_TRACK\":\"1\",\"CP_W_SHIFT\":\"40\",$A2,\"MPC_COP_REF_CMD\":\"1\"}" \
    RISERS=0.020,0.040,0.060,0.080 python3 log/claude_g1rand_20260924/rand.py
  echo "##### 自由腕 筋A SEED=2 3a抜き  段（アブレーション）"
  TAG=U_f_no3a BASE=$F BIN=$BIN MODE=stairs N=48 W=13 SEED=20260924 EXTRA="{$A2,\"MPC_COP_REF_CMD\":\"0\"}" \
    RISERS=0.020,0.040,0.060,0.080 python3 log/claude_g1rand_20260924/rand.py
fi
echo "#### BRANCHA_DONE_$GRP"
