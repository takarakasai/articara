set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
C=log/claude_g1champion_20260927v52/controls.json; BIN=log/claude_g1rand_20260925/kyo46rs_walk_terrain
export MU_LO=0.40 MU_HI=0.90 JITTER=0.03 DX=0.05
for w in 1 10; do
  ( echo "##### 地形コスト $w: wide holdout"; TAG=TRW${w}W_h BASE=$C BIN=$BIN MODE=stairs N=48 W=8 SEED=777 EXTRA="{\"MPC_FOOTSTEP_TERRAIN_W\":\"$w\"}" RISERS=0.020,0.040,0.060,0.080 python3 log/claude_g1rand_20260924/rand.py ) &
done
( echo "##### 地形コスト 1: wide 段A"; TAG=TRW1W_s BASE=$C BIN=$BIN MODE=stairs N=48 W=8 SEED=20260924 EXTRA='{"MPC_FOOTSTEP_TERRAIN_W":"1"}' RISERS=0.020,0.040,0.060,0.080 python3 log/claude_g1rand_20260924/rand.py ) &
wait
echo "#### TERR_DONE"
