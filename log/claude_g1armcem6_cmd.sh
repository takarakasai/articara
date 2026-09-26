set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
export CEM_RISERS=0.020,0.040,0.060,0.080 CEM_SURVIVE_BONUS=2000 CEM_SURVIVE_MIN_X=2.5 CEM_DISTANCE_WEIGHT=300
export CEM_SITUATIONS=1 CEM_SIT_SEED=20260925
export CEM_PUSH_CONDS=5@270,5@270,5@270,7@90,7@90,11@0,11@0,11@180 CEM_PUSH_MEMBERS=8
# ステップ 2: MPC レンチを重み付き目的（連成 ON、権限なし）として歩行用に調整。同じ --out で再開可。
python3 scripts/kyo46rs_cem_transfer.py \
  --base log/claude_g1armcem6_base.json \
  --binary target/release/examples/kyo46rs_walk \
  --out log/claude_g1armcem6_20260926 \
  --gens 8 --pop 24 --elite 7 --jobs 14 --ensemble 24 --aggregate mean --seed 20260926 \
  --space-json log/claude_g1armcem6_space.json
