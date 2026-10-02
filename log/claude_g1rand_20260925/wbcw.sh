set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
C=log/claude_g1champion_20260927v52/controls.json; BIN=log/claude_g1rand_20260925/kyo46rs_walk_ltime5
run () { ( echo "##### $1"; TAG=$2 BASE=$C BIN=$BIN MODE=push N=48 W=6 SEED=20260924 EXTRA="$3" CONDS=11@180 python3 log/claude_g1rand_20260924/rand.py ) & }
B='"MPC_CAPTURE_LEX":"1","MPC_PCMD":"1","MPC_PCMD_FB":"1"'
run "LEX + REG 1e-2"     WB_reg2  "{$B,\"WBCW_REGULARISE\":\"1e-2\"}"
run "LEX + REG 1e-1"     WB_reg1  "{$B,\"WBCW_REGULARISE\":\"1e-1\"}"
run "LEX + CONTACT 1.0"  WB_con1  "{$B,\"WBCW_CONTACT_CONES\":\"1.0\"}"
run "LEX + MOMENTUM 0.01" WB_mom   "{$B,\"WBCW_MOMENTUM\":\"0.01\"}"
run "LEX + COUPLED=1"    WB_cpl   "{$B,\"STAIR_WRENCH_COUPLED\":\"1\"}"
wait
echo "#### WBCW_DONE"
