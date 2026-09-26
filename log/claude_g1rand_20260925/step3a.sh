set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
BIN=target/release/examples/kyo46rs_walk
WHICH=$1
A='"MPC_NODE_CENTRE":"1","WRENCH_MPC_APPLY":"1","STAIR_WRENCH_COUPLED":"0","MPC_SOLE_AUTHORITY":"1","MPC_NET_COP_W":"10","MPC_W_SMOOTH":"10","MPC_COP_REF_CMD":"1"'
if [ "$WHICH" = w ]; then
  echo "##### 3a: 溶接 v3.1 + 筋A + 参照=p_cmd  段"
  TAG=S3a_w BASE=log/claude_g1champion_20260924v31/controls.json BIN=$BIN MODE=stairs N=48 W=13 SEED=20260924 \
    EXTRA="{\"CP_TRACK\":\"1\",\"CP_W_SHIFT\":\"40\",$A}" RISERS=0.020,0.040,0.060,0.080 python3 log/claude_g1rand_20260924/rand.py
else
  echo "##### 3a: 自由腕5+保持 + 筋A + 参照=p_cmd  段"
  TAG=S3a_f BASE=log/claude_g1combo_base.json BIN=$BIN MODE=stairs N=48 W=13 SEED=20260924 \
    EXTRA="{$A}" RISERS=0.020,0.040,0.060,0.080 python3 log/claude_g1rand_20260924/rand.py
fi
echo "#### DONE3a_$WHICH"
