set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
python3 - <<'PY'
import json
c=json.load(open('log/claude_g1champion_20260927v52/controls.json')); c["WBC_WEIGHTED_OFF_STAIRS"]="0"
json.dump(c,open('log/claude_g1bdbaseline_20260926/controls_v6_candidate_wq.json','w'),indent=1)
PY
C=log/claude_g1bdbaseline_20260926/controls_v6_candidate_wq.json; BIN=log/claude_g1champion_20260927v52/kyo46rs_walk
W=${W:-13} BIN=$BIN scripts/g1_battery.sh WQF $C '{}' std
W=${W:-13} BIN=$BIN scripts/g1_battery.sh WQF $C '{}' wide
echo "##### WQF 低い段"; TAG=WQF_low BASE=$C BIN=$BIN MODE=stairs N=48 W=${W:-13} SEED=20260924 EXTRA='{}' RISERS=0.000,0.010,0.020,0.030 python3 log/claude_g1rand_20260924/rand.py
echo "#### WQF_DONE"
