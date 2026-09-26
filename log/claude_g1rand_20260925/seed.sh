set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
BIN=target/release/examples/kyo46rs_walk
GRP=$1
A='"MPC_NODE_CENTRE":"1","WRENCH_MPC_APPLY":"1","STAIR_WRENCH_COUPLED":"0","MPC_SOLE_AUTHORITY":"1","MPC_NET_COP_W":"10","MPC_COP_REF_CMD":"1"'
F=log/claude_g1combo_base.json; W31=log/claude_g1champion_20260924v31/controls.json
run () { echo "##### $1"; TAG=$2 BASE=$3 BIN=$BIN MODE=stairs N=48 W=13 SEED=20260924 EXTRA="$4" RISERS=0.020,0.040,0.060,0.080 python3 log/claude_g1rand_20260924/rand.py; }
if [ "$GRP" = 1 ]; then
  run "自由腕 筋A SMOOTH=10 SEED=first"  T_f_s1   $F "{$A,\"MPC_W_SMOOTH\":\"10\",\"MPC_SEED_PREVIOUS\":\"1\"}"
  run "自由腕 筋A SMOOTH=10 SEED=always" T_f_s2   $F "{$A,\"MPC_W_SMOOTH\":\"10\",\"MPC_SEED_PREVIOUS\":\"2\"}"
else
  run "自由腕 筋A SMOOTH=1"              T_f_sm1  $F "{$A,\"MPC_W_SMOOTH\":\"1\"}"
  run "溶接 筋A SMOOTH=10 SEED=first"    T_w_s1   $W31 "{\"CP_TRACK\":\"1\",\"CP_W_SHIFT\":\"40\",$A,\"MPC_W_SMOOTH\":\"10\",\"MPC_SEED_PREVIOUS\":\"1\"}"
fi
echo "#### SEEDDONE_$GRP"
