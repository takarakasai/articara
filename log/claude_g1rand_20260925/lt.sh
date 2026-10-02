set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
C=log/claude_g1champion_20260927v52/controls.json; BIN=log/claude_g1rand_20260925/kyo46rs_walk_ltime5
for bx in 0.15 0.08; do
  E="{\"MPC_LANDING_TIME\":\"1\",\"CONTACT_PLAN\":\"0\",\"MPC_FOOTSTEP_MAX_X\":\"$bx\"}"
  ( echo "##### TIME box $bx: 11@180/11@0 N=48"; TAG=LT48X${bx}_p BASE=$C BIN=$BIN MODE=push N=48 W=8 SEED=20260924 EXTRA="$E" CONDS=11@180,11@0 python3 log/claude_g1rand_20260924/rand.py ) &
  ( echo "##### TIME box $bx: std 段 N=24"; TAG=LTX${bx}_s BASE=$C BIN=$BIN MODE=stairs N=24 W=6 SEED=20260924 EXTRA="$E" RISERS=0.020,0.040,0.060,0.080 python3 log/claude_g1rand_20260924/rand.py ) &
done
( echo "##### CONTACT_PLAN=0 のみ: 11@180/11@0 N=48"; TAG=CP048_p BASE=$C BIN=$BIN MODE=push N=48 W=4 SEED=20260924 EXTRA='{"CONTACT_PLAN":"0"}' CONDS=11@180,11@0 python3 log/claude_g1rand_20260924/rand.py ) &
wait
echo "#### LT_DONE"
