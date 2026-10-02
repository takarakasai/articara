set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
C=log/claude_g1champion_20260927v52/controls.json; BIN=log/claude_g1champion_20260927v52/kyo46rs_walk
E='{"MOM_REF_MPC":"1"}'
( echo "##### MOM_REF_MPC: 押し 8 条件"; TAG=MOM_p BASE=$C BIN=$BIN MODE=push N=24 W=6 SEED=20260924 EXTRA="$E" CONDS=0@0,8@0,11@0,3@270,5@270,7@90,11@180,14@180 python3 log/claude_g1rand_20260924/rand.py ) &
( echo "##### MOM_REF_MPC: std 段"; TAG=MOM_s BASE=$C BIN=$BIN MODE=stairs N=24 W=4 SEED=20260924 EXTRA="$E" RISERS=0.020,0.040,0.060,0.080 python3 log/claude_g1rand_20260924/rand.py ) &
wait
echo "#### MOM_DONE"
