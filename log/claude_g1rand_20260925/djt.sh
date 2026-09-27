set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
C=log/claude_g1bdbaseline_20260926/controls_v5_candidate_DJT.json; BIN=log/claude_g1rand_20260925/kyo46rs_walk_djt
( W=11 BIN=$BIN scripts/g1_battery.sh DJT $C '{}' std ) &
( W=11 BIN=$BIN scripts/g1_battery.sh DJT $C '{}' wide ) &
( ( export MU_LO=0.40 MU_HI=0.90 JITTER=0.03 DX=0.05; TAG=DJTW_h2 BASE=$C BIN=$BIN MODE=stairs N=48 W=5 SEED=778 EXTRA='{}' RISERS=0.020,0.040,0.060,0.080 python3 log/claude_g1rand_20260924/rand.py )
  TAG=DJT_low BASE=$C BIN=$BIN MODE=stairs N=48 W=5 SEED=20260924 EXTRA='{}' RISERS=0.000,0.010,0.020,0.030 python3 log/claude_g1rand_20260924/rand.py ) &
wait
echo "#### DJT_DONE"
