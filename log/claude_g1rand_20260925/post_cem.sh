set -u
cd /home/takara/work/dp/articara
until grep -q "GAITCEM_DONE" log/claude_g1gaitcem_20261002.out; do sleep 300; done
( bash log/claude_g1rand_20260925/gaitcem_verify.sh > log/claude_g1rand_20260925/gaitcem_verify.out 2>&1 ) &
( bash log/claude_g1rand_20260925/bench_ext.sh B52 log/claude_g1champion_20260927v52/controls.json > log/claude_g1rand_20260925/bench_b52.out 2>&1 ) &
wait
date +%H:%M
echo "=== 歩容 CEM BEST（V6G）対 v5.2: std 段A / holdout / 押し / 横 ==="
python3 /tmp/claude-1000/-home-takara-work-dp-articara/c7814efe-5799-450c-ad10-d7119cecb8a9/scratchpad/mcpair.py LX_s V6G_s | tail -1; python3 /tmp/claude-1000/-home-takara-work-dp-articara/c7814efe-5799-450c-ad10-d7119cecb8a9/scratchpad/mcpair.py LX_h V6G_h | tail -1; python3 /tmp/claude-1000/-home-takara-work-dp-articara/c7814efe-5799-450c-ad10-d7119cecb8a9/scratchpad/mcpair.py LX_p V6G_p push; python3 /tmp/claude-1000/-home-takara-work-dp-articara/c7814efe-5799-450c-ad10-d7119cecb8a9/scratchpad/mcpair.py LX_lat V6G_lat push | tail -1
echo "--- wide 段A / holdout / 押し / 横"
python3 /tmp/claude-1000/-home-takara-work-dp-articara/c7814efe-5799-450c-ad10-d7119cecb8a9/scratchpad/mcpair.py LXW_s V6GW_s | tail -1; python3 /tmp/claude-1000/-home-takara-work-dp-articara/c7814efe-5799-450c-ad10-d7119cecb8a9/scratchpad/mcpair.py LXW_h V6GW_h | tail -1; python3 /tmp/claude-1000/-home-takara-work-dp-articara/c7814efe-5799-450c-ad10-d7119cecb8a9/scratchpad/mcpair.py LXW_p V6GW_p push | tail -1; python3 /tmp/claude-1000/-home-takara-work-dp-articara/c7814efe-5799-450c-ad10-d7119cecb8a9/scratchpad/mcpair.py LXW_lat V6GW_lat push | tail -1
echo "--- 低い段"; python3 /tmp/claude-1000/-home-takara-work-dp-articara/c7814efe-5799-450c-ad10-d7119cecb8a9/scratchpad/mcpair.py LX_low V6G_low | tail -1
echo "=== v5.2 の新 4 系統基準（B52）==="; grep -E "^#####|生存" log/claude_g1rand_20260925/bench_b52.out | sed -E 's/転倒時刻の種類.*//'
echo "#### POST_CEM_DONE"
