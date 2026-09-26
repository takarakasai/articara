set -u
cd /home/takara/work/dp/articara
# 2a/2b のバッテリが終わってから開始（CPU）
until grep -q STEP4AB_DONE log/claude_g1rand_20260925/step4ab.out 2>/dev/null; do sleep 120; done
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
export CEM_RISERS=0.020,0.040,0.060,0.080 CEM_SURVIVE_BONUS=2000 CEM_SURVIVE_MIN_X=2.5 CEM_DISTANCE_WEIGHT=300
export CEM_SITUATIONS=1 CEM_SIT_SEED=20260925 CEM_SIT_MU_LO=0.40 CEM_SIT_MU_HI=0.90 CEM_SIT_JITTER=0.03 CEM_SIT_DX=0.05
export CEM_PUSH_CONDS=5@270,5@270,5@270,7@90,7@90,11@0,11@0,11@180 CEM_PUSH_MEMBERS=8
# CEM8: v4（STAIR_RISE_SOLE_POSE=0）を中心に、全段等重み・広い封筒。20 mm を 80 mm と交換しないための探索。同じ --out で再開可。
python3 scripts/kyo46rs_cem_transfer.py \
  --base log/claude_g1armcem8_base.json --binary target/release/examples/kyo46rs_walk \
  --out log/claude_g1armcem8_20260927 --gens 8 --pop 20 --elite 6 --jobs 26 --ensemble 24 --aggregate mean --seed 20260927 \
  --space-json log/claude_g1armcem8_space.json
echo "#### CEM8_DONE"
