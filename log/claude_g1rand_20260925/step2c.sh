set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
BIN=target/release/examples/kyo46rs_walk; V1=log/claude_g1bdbaseline_20260926/controls.json
X='"MPC_DCM_W":"50000","MPC_XI_REF":"1"'
GRP=$1
if [ "$GRP" = s ]; then
  echo "##### 段: XI_REF + 乗り移りは計画"; TAG=X2_xi0g BASE=$V1 BIN=$BIN MODE=stairs N=48 W=13 SEED=20260924 EXTRA="{$X,\"MPC_XI_REF_TRANSFER_PLAN\":\"1\"}" RISERS=0.020,0.040,0.060,0.080 python3 log/claude_g1rand_20260924/rand.py
else
  echo "##### 押し（平地）: XI_REF"; TAG=X2_xi0_p BASE=$V1 BIN=$BIN MODE=push N=48 W=13 SEED=20260924 EXTRA="{$X}" CONDS=0@0,8@0,11@0,3@270,5@270,7@90,11@180,14@180 python3 log/claude_g1rand_20260924/rand.py
fi
echo "#### STEP2C_DONE_$GRP"
