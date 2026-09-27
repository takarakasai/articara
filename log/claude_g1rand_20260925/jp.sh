set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
D=log/claude_g1bdbaseline_20260926/controls_v5_candidate_D.json; BIN=log/claude_g1rand_20260925/kyo46rs_walk_jprobe
( export MU_LO=0.40 MU_HI=0.90 JITTER=0.03 DX=0.05
  echo "##### D+J 1e6: wide holdout"; TAG=DJW_h BASE=$D BIN=$BIN MODE=stairs N=48 W=13 SEED=777 EXTRA='{"MPC_LANDING_CAPTURE_J":"1"}' RISERS=0.020,0.040,0.060,0.080 python3 log/claude_g1rand_20260924/rand.py ) &
( echo "##### D+J 1e6: 押し 4 条件"; TAG=DJ_p4 BASE=$D BIN=$BIN MODE=push N=24 W=13 SEED=20260924 EXTRA='{"MPC_LANDING_CAPTURE_J":"1"}' CONDS=8@0,11@0,5@270,7@90 python3 log/claude_g1rand_20260924/rand.py
  echo "##### D+J 1e5: 押し 4 条件"; TAG=DJ5_p4 BASE=$D BIN=$BIN MODE=push N=24 W=13 SEED=20260924 EXTRA='{"MPC_LANDING_CAPTURE_J":"1","MPC_LANDING_CAPTURE_W":"100000"}' CONDS=8@0,11@0,5@270,7@90 python3 log/claude_g1rand_20260924/rand.py ) &
( echo "##### D+J 1e6: std 段"; TAG=DJ_s24 BASE=$D BIN=$BIN MODE=stairs N=24 W=13 SEED=20260924 EXTRA='{"MPC_LANDING_CAPTURE_J":"1"}' RISERS=0.020,0.040,0.060,0.080 python3 log/claude_g1rand_20260924/rand.py
  echo "##### D+J 1e5: std 段"; TAG=DJ5_s24 BASE=$D BIN=$BIN MODE=stairs N=24 W=13 SEED=20260924 EXTRA='{"MPC_LANDING_CAPTURE_J":"1","MPC_LANDING_CAPTURE_W":"100000"}' RISERS=0.020,0.040,0.060,0.080 python3 log/claude_g1rand_20260924/rand.py ) &
wait
echo "#### JP_DONE"
