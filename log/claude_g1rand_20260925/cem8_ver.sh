set -u
cd /home/takara/work/dp/articara
until grep -q CEM8_DONE log/claude_g1armcem8_20260927.out 2>/dev/null; do sleep 180; done
python3 - <<'PY'
import json
b=json.load(open('log/claude_g1armcem8_20260927/BEST/controls.json'))
for k in ('TRAJ_CSV','SUBSTEP_LIMIT_AUDIT_JSON','STAIRS','INIT_JITTER','INIT_SEED','MU_GROUND','PUSH_IMPULSE','PUSH_DEG','PUSH_STEP','PUSH_AT','PUSH_SUPPORT'): b.pop(k,None)
json.dump(b,open('log/claude_g1armcem8_verify.json','w'),indent=1)
PY
W=13 scripts/g1_battery.sh C8 log/claude_g1armcem8_verify.json '{}' std &
W=13 scripts/g1_battery.sh C8 log/claude_g1armcem8_verify.json '{}' wide &
wait
S=/tmp/claude-1000/-home-takara-work-dp-articara/c7814efe-5799-450c-ad10-d7119cecb8a9/scratchpad; [ -f $S/mcpair.py ] || { mkdir -p $S; cp scripts/g1_paired.py $S/mcpair.py; }
echo "=== CEM8 対 v4.1（std: 段A/holdout/押し/横）==="; python3 $S/mcpair.py TM_s C8_s; python3 $S/mcpair.py TM_h C8_h | tail -1; python3 $S/mcpair.py TM_p C8_p push | tail -1; python3 $S/mcpair.py TM_lat C8_lat push | tail -1
echo "=== CEM8 対 v4.1（wide）==="; python3 $S/mcpair.py TMW_s C8W_s | tail -1; python3 $S/mcpair.py TMW_h C8W_h | tail -1; python3 $S/mcpair.py TMW_p C8W_p push | tail -1; python3 $S/mcpair.py TMW_lat C8W_lat push | tail -1
echo "#### CEM8_VER_DONE"
