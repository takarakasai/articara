set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
A=log/claude_g1bdbaseline_20260926/controls_v5_candidate_A.json; BIN=log/claude_g1rand_20260925/kyo46rs_walk_capprobe
cp target/release/examples/kyo46rs_walk $BIN
run () { echo "##### $1"; TAG=$2 BASE=$A BIN=$BIN MODE=push N=24 W=4 SEED=20260924 EXTRA="$3" CONDS=8@0,5@270 python3 log/claude_g1rand_20260924/rand.py; }
run "A + REF_CMD 上限 3 cm"  FC_cap03 '{"MPC_COP_REF_CMD_MAX":"0.03"}'
run "A + REF_CMD 上限 5 cm"  FC_cap05 '{"MPC_COP_REF_CMD_MAX":"0.05"}'
echo "#### FS_CAP_DONE"
