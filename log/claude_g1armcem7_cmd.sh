set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
export CEM_RISERS=0.020,0.020,0.040,0.060,0.080 CEM_SURVIVE_BONUS=2000 CEM_SURVIVE_MIN_X=2.5 CEM_DISTANCE_WEIGHT=300
export CEM_SITUATIONS=1 CEM_SIT_SEED=20260925
export CEM_PUSH_CONDS=5@270,5@270,5@270,7@90,7@90,11@0,11@0,11@180 CEM_PUSH_MEMBERS=8
# ステップ 1: v1 構成（筋 A + SEED=2）を固定し、20 mm を重く（段 16 本のうち 20 mm が 6〜7 本）。同じ --out で再開可。
python3 scripts/kyo46rs_cem_transfer.py \
  --base log/claude_g1armcem7_base.json --binary target/release/examples/kyo46rs_walk \
  --out log/claude_g1armcem7_20260926 --gens 8 --pop 20 --elite 6 --jobs 13 --ensemble 24 --aggregate mean --seed 20260926 \
  --space-json log/claude_g1armcem7_space.json
