set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
L=log/claude_g1rand_20260925; C=log/claude_g1champion_20261003v6/controls.json; BIN=$L/kyo46rs_walk_sense
until grep -q SF_DONE $L/sfilt_summary.out 2>/dev/null; do sleep 60; done
W=10 BIN=$BIN scripts/g1_battery.sh SF40 $C '{"SENSE_FILTER_HZ":"40"}' std
{ date +%H:%M; echo "=== フィルタ 40 Hz 対 v6: std 段A / holdout / 押し / 横 ==="; python3 $L/mcpair.py WQF_s SF40_s; python3 $L/mcpair.py WQF_h SF40_h | tail -1; python3 $L/mcpair.py WQF_p SF40_p push | tail -1; python3 $L/mcpair.py WQF_lat SF40_lat push | tail -1; echo "#### SF40_DONE"; } > $L/sf40_summary.out 2>&1
