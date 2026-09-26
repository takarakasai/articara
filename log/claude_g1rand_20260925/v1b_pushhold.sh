set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
BIN=target/release/examples/kyo46rs_walk; C8=0@0,8@0,11@0,3@270,5@270,7@90,11@180,14@180
echo "##### 押し ホールドアウト 777: v1b"; TAG=v1b_ph BASE=log/claude_g1bdbaseline_20260926/controls_v1b.json BIN=$BIN MODE=push N=48 W=13 SEED=777 EXTRA='{}' CONDS=$C8 python3 log/claude_g1rand_20260924/rand.py
echo "##### 押し ホールドアウト 777: v3.1"; TAG=v31_ph BASE=log/claude_g1champion_20260924v31/controls.json BIN=$BIN MODE=push N=48 W=13 SEED=777 EXTRA='{"CP_TRACK":"1","CP_W_SHIFT":"40"}' CONDS=$C8 python3 log/claude_g1rand_20260924/rand.py
echo "#### V1B_PUSHHOLD_DONE"
