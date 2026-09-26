set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
export MU_LO=0.40 MU_HI=0.90 JITTER=0.03 DX=0.05
D=log/claude_g1bdbaseline_20260926/controls_v5_candidate_D.json; BIN=log/claude_g1rand_20260925/kyo46rs_walk_ftprobe
run () { echo "##### $1"; TAG=$2 BASE=$D BIN=$BIN MODE=stairs N=48 W=13 SEED=777 EXTRA="$3" RISERS=0.020,0.040,0.060,0.080 python3 log/claude_g1rand_20260924/rand.py; }
run "D、捕捉 1e7"      DH_c1e7 '{"MPC_LANDING_CAPTURE_W":"10000000"}'
run "D、箱 x 6 / y 8"  DH_x06  '{"MPC_FOOTSTEP_MAX_X":"0.06"}'
echo "#### DH2_DONE"
