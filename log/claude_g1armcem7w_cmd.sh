set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
export CEM_RISERS=0.020,0.020,0.040,0.060,0.080 CEM_SURVIVE_BONUS=2000 CEM_SURVIVE_MIN_X=2.5 CEM_DISTANCE_WEIGHT=300
export CEM_SITUATIONS=1 CEM_SIT_SEED=20260925 CEM_SIT_MU_LO=0.40 CEM_SIT_MU_HI=0.90 CEM_SIT_JITTER=0.03 CEM_SIT_DX=0.05
export CEM_PUSH_CONDS=5@270,5@270,5@270,7@90,7@90,11@0,11@0,11@180 CEM_PUSH_MEMBERS=8
# ステップ 1（広い封筒版）: v1 構成を固定、20 mm を重く、状況は wide（摩擦 0.40–0.90・ジッタ 0.03・段 ±5 cm）。
python3 scripts/kyo46rs_cem_transfer.py \
  --base log/claude_g1armcem7_base.json --binary target/release/examples/kyo46rs_walk \
  --out log/claude_g1armcem7w_20260926 --gens 8 --pop 20 --elite 6 --jobs 13 --ensemble 24 --aggregate mean --seed 20260926 \
  --space-json log/claude_g1armcem7_space.json
