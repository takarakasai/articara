set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
C=log/claude_g1bdbaseline_20260926/controls_v5_candidate_C.json; BIN=log/claude_g1rand_20260925/kyo46rs_walk_ftprobe
st () { echo "##### $1"; TAG=$2 BASE=$C BIN=$BIN MODE=stairs N=24 W=13 SEED=20260924 EXTRA="$3" RISERS=0.020,0.040,0.060,0.080 python3 log/claude_g1rand_20260924/rand.py; }
st "C − 時間調整 CP（CONTACT_PLAN=0）"   CS_cp0  '{"CONTACT_PLAN":"0"}'
st "C、箱 6/3"                          CS_box  '{"MPC_FOOTSTEP_MAX_X":"0.06","MPC_FOOTSTEP_MAX_Y":"0.03"}'
st "C、捕捉 1e4"                         CS_c1e4 '{"MPC_LANDING_CAPTURE_W":"10000"}'
st "C、N=16"                             CS_n16  '{"STAIR_WRENCH_N":"16"}'
st "C、捕捉 1e6"                         CS_c1e6 '{"MPC_LANDING_CAPTURE_W":"1000000"}'
echo "##### C + 捕捉 1e6: 押し 3 条件"; TAG=F11b_c1e6 BASE=$C BIN=$BIN MODE=push N=24 W=13 SEED=20260924 EXTRA='{"MPC_LANDING_CAPTURE_W":"1000000"}' CONDS=8@0,5@270,7@90 python3 log/claude_g1rand_20260924/rand.py
echo "#### FS_CST_DONE"
