set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
A=log/claude_g1bdbaseline_20260926/controls_v5_candidate_A.json; BIN=log/claude_g1rand_20260925/kyo46rs_walk_m0probe
E='{"MPC_COP_REF_CMD":"0","MPC_FOOTSTEP_MOMENT":"0"}'
echo "##### A − REF_CMD、MOMENT=0: 押し"; TAG=FM_p BASE=$A BIN=$BIN MODE=push N=24 W=6 SEED=20260924 EXTRA="$E" CONDS=8@0,5@270 python3 log/claude_g1rand_20260924/rand.py
echo "##### A − REF_CMD、MOMENT=0: std 段"; TAG=FM_s BASE=$A BIN=$BIN MODE=stairs N=24 W=6 SEED=20260924 EXTRA="$E" RISERS=0.020,0.040,0.060,0.080 python3 log/claude_g1rand_20260924/rand.py
echo "##### A − REF_CMD、MOMENT=0、捕捉 1e3: 押し"; TAG=FMc_p BASE=$A BIN=$BIN MODE=push N=24 W=6 SEED=20260924 EXTRA='{"MPC_COP_REF_CMD":"0","MPC_FOOTSTEP_MOMENT":"0","MPC_LANDING_CAPTURE_W":"1000"}' CONDS=8@0,5@270 python3 log/claude_g1rand_20260924/rand.py
echo "#### FS_M0_DONE"
