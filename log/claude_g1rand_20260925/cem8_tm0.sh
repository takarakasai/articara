cd /home/takara/work/dp/articara
until grep -q CEM8_VER_DONE log/claude_g1rand_20260925/cem8_ver.out 2>/dev/null; do sleep 120; done
W=13 scripts/g1_battery.sh C8tm log/claude_g1armcem8_verify.json '{"STAIR_TOE_MARGIN":"0"}' std
S=/tmp/claude-1000/-home-takara-work-dp-articara/c7814efe-5799-450c-ad10-d7119cecb8a9/scratchpad; [ -f $S/mcpair.py ] || { mkdir -p $S; cp scripts/g1_paired.py $S/mcpair.py; }
echo "=== CEM8+TOE_MARGIN=0 対 v4.1（std）==="; python3 $S/mcpair.py TM_s C8tm_s; python3 $S/mcpair.py TM_h C8tm_h | tail -1; python3 $S/mcpair.py TM_p C8tm_p push | tail -1; python3 $S/mcpair.py TM_lat C8tm_lat push | tail -1
echo "#### CEM8_TM0_DONE"
