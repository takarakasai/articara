set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
C=log/claude_g1champion_20260927v52/controls.json; BIN=log/claude_g1rand_20260925/kyo46rs_walk_pcfb
E='{"MPC_PCMD":"0.5","MPC_PCMD_FB":"1","MPC_CAPTURE_COP_W":"10000000"}'
echo "##### PCMD=0.5+FB 1e7: 押し 8 条件"; TAG=PCFBH_p BASE=$C BIN=$BIN MODE=push N=24 W=13 SEED=20260924 EXTRA="$E" CONDS=0@0,8@0,11@0,3@270,5@270,7@90,11@180,14@180 python3 log/claude_g1rand_20260924/rand.py
echo "#### PCFBH_DONE"
