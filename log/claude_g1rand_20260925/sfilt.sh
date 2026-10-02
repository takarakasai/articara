set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
L=log/claude_g1rand_20260925; C=log/claude_g1champion_20261003v6/controls.json; BIN=$L/kyo46rs_walk_sense
for k in SENSE_FILTER_HZ DCM_LEAD_S; do [ "$(grep -ac $k $BIN)" -ge 1 ] || { echo "binary lacks $k"; exit 1; }; done
export FLAT_MIN_STEPS=15
CONDS="SENSE_DELAY_TICKS=2,SENSE_DELAY_TICKS=4,SENSE_DELAY_TICKS=8,SENSE_NOISE_QD=0.1;SENSE_NOISE_V=0.03;SENSE_NOISE_W=0.03,SENSE_NOISE_QD=0.3;SENSE_NOISE_V=0.1;SENSE_NOISE_W=0.1,SENSE_DELAY_TICKS=4;SENSE_NOISE_QD=0.1;SENSE_NOISE_V=0.03;SENSE_NOISE_W=0.03"
for hz in 20 40; do
  echo "##### v6 + FILTER $hz Hz: センサ"; TAG=SF${hz}_sense BASE=$C BIN=$BIN MODE=flat N=48 W=6 SEED=20260924 T=25 EXTRA="{\"SENSE_FILTER_HZ\":\"$hz\"}" CONDS="$CONDS" python3 log/claude_g1rand_20260924/rand.py
done
W=10 BIN=$BIN scripts/g1_battery.sh SF20 $C '{"SENSE_FILTER_HZ":"20"}' std
{ date +%H:%M; echo "=== フィルタ 20 Hz 対 v6: std 段A / holdout / 押し / 横 ==="; python3 $L/mcpair.py WQF_s SF20_s | tail -1; python3 $L/mcpair.py WQF_h SF20_h | tail -1; python3 $L/mcpair.py WQF_p SF20_p push; python3 $L/mcpair.py WQF_lat SF20_lat push | tail -1
echo "=== センサ基準（v6 基準は B52_sense と同一）==="; grep -E "生存" $L/bench_b52_rerun_sense.txt 2>/dev/null; echo "#### SF_DONE"; } > $L/sfilt_summary.out 2>&1
