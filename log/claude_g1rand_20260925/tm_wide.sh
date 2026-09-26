cd /home/takara/work/dp/articara
until grep -q CAND_DONE_TM log/claude_g1rand_20260925/cand_TM.out 2>/dev/null; do sleep 60; done
W=13 scripts/g1_battery.sh TM log/claude_g1champion_20260926v4/controls.json '{"STAIR_TOE_MARGIN":"0"}' wide
echo "#### TM_WIDE_DONE"
