set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
export MU_LO=0.40 MU_HI=0.90 JITTER=0.03 DX=0.05
D=log/claude_g1bdbaseline_20260926/controls_v5_candidate_D.json; BIN=log/claude_g1rand_20260925/kyo46rs_walk_ftprobe
run () { echo "##### $1"; TAG=$2 BASE=$D BIN=$BIN MODE=stairs N=48 W=13 SEED=777 EXTRA="$3" RISERS=0.020,0.040,0.060,0.080 python3 log/claude_g1rand_20260924/rand.py; }
run "D − 時間調整 CP" DH_cp0 '{"CONTACT_PLAN":"0"}'
run "D、箱 10/5"      DH_box '{"MPC_FOOTSTEP_MAX_X":"0.10","MPC_FOOTSTEP_MAX_Y":"0.05"}'
echo "#### DH1_DONE"
