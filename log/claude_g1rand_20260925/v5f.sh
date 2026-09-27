set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
C=log/claude_g1bdbaseline_20260926/controls_v5f_candidate.json; BIN=log/claude_g1rand_20260925/kyo46rs_walk_fwdprobe
( W=11 BIN=$BIN scripts/g1_battery.sh V5F $C '{}' std ) &
( W=11 BIN=$BIN scripts/g1_battery.sh V5F $C '{}' wide ) &
( ( export MU_LO=0.40 MU_HI=0.90 JITTER=0.03 DX=0.05; TAG=V5FW_h2 BASE=$C BIN=$BIN MODE=stairs N=48 W=8 SEED=778 EXTRA='{}' RISERS=0.020,0.040,0.060,0.080 python3 log/claude_g1rand_20260924/rand.py )
  TAG=V5F_low BASE=$C BIN=$BIN MODE=stairs N=48 W=8 SEED=20260924 EXTRA='{}' RISERS=0.000,0.010,0.020,0.030 python3 log/claude_g1rand_20260924/rand.py ) &
wait
echo "#### V5F_DONE"
