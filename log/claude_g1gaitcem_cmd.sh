set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
export CEM_RISERS=0.020,0.040,0.060,0.080 CEM_SURVIVE_BONUS=2000 CEM_SURVIVE_MIN_X=2.5 CEM_DISTANCE_WEIGHT=300
export CEM_SITUATIONS=1 CEM_SIT_SEED=20261002 CEM_SIT_MU_LO=0.40 CEM_SIT_MU_HI=0.90 CEM_SIT_JITTER=0.03 CEM_SIT_DX=0.05
# 押し 8 本: 弱点 3 機構（後ろ 11、支持足側の横 7、遅い位相の前 11）を重く
export CEM_PUSH_CONDS=11@180,11@180,7@90,7@90,11@0,11@0,5@270,8@0 CEM_PUSH_MEMBERS=8
# 歩容定数の CEM（計画 1）: v5.2 を中心に 18 軸。バイナリは v5.2 チャンピオンのスナップショット。同じ --out で再開可。
python3 scripts/kyo46rs_cem_transfer.py \
  --base log/claude_g1champion_20260927v52/controls.json --binary log/claude_g1champion_20260927v52/kyo46rs_walk \
  --out log/claude_g1gaitcem_20261002 --gens 5 --pop 20 --elite 6 --jobs 26 --ensemble 24 --aggregate mean --seed 20261002 \
  --space-json log/claude_g1gaitcem_space.json --set STAIRS=1,0.020,0.211,2.00
echo "#### GAITCEM_DONE"
