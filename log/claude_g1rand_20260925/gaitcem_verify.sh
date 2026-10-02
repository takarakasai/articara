set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
C=log/claude_g1gaitcem_20261002/BEST/controls.json; BIN=log/claude_g1champion_20260927v52/kyo46rs_walk
python3 - <<'PY'
import json
# BEST/controls.json は 1 個体の実行用（STAIRS/PUSH/TRAJ_CSV 等を含む）: 実行固有キーを落として候補にする
c=json.load(open('log/claude_g1gaitcem_20261002/BEST/controls.json'))
for k in ('TRAJ_CSV','SUBSTEP_LIMIT_AUDIT_JSON','PUSH_IMPULSE','PUSH_DEG','PUSH_STEP','PUSH_AT','PUSH_SUPPORT','INIT_JITTER','INIT_SEED','MU_GROUND'): c.pop(k,None)
c.pop('STAIRS',None)  # バッテリ側が与える（v5.2 の controls にも無い）
json.dump(c,open('log/claude_g1bdbaseline_20260926/controls_v6_candidate_gait.json','w'),indent=1)
PY
C=log/claude_g1bdbaseline_20260926/controls_v6_candidate_gait.json
W=13 BIN=$BIN scripts/g1_battery.sh V6G $C '{}' std
W=13 BIN=$BIN scripts/g1_battery.sh V6G $C '{}' wide
echo "##### V6G 低い段"; TAG=V6G_low BASE=$C BIN=$BIN MODE=stairs N=48 W=13 SEED=20260924 EXTRA='{}' RISERS=0.000,0.010,0.020,0.030 python3 log/claude_g1rand_20260924/rand.py
echo "#### V6G_DONE"
