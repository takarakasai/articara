set -u
cd /home/takara/work/dp/articara
# 再開した CEM ドライバの終了（DONE 行が 2 本目）を待つ
until [ "$(grep -c GAITCEM_DONE log/claude_g1gaitcem_20261002.out)" -ge 2 ]; do sleep 120; done
bash log/claude_g1rand_20260925/gaitcem_verify.sh > log/claude_g1rand_20260925/gaitcem_verify.out 2>&1
{ date +%H:%M; echo "=== 歩容 CEM BEST（V6G）対 v5.2: std 段A / holdout / 押し / 横 ==="
python3 log/claude_g1rand_20260925/mcpair.py LX_s V6G_s | tail -1; python3 log/claude_g1rand_20260925/mcpair.py LX_h V6G_h | tail -1; python3 log/claude_g1rand_20260925/mcpair.py LX_p V6G_p push; python3 log/claude_g1rand_20260925/mcpair.py LX_lat V6G_lat push | tail -1
echo "--- wide 段A / holdout / 押し / 横"; python3 log/claude_g1rand_20260925/mcpair.py LXW_s V6GW_s | tail -1; python3 log/claude_g1rand_20260925/mcpair.py LXW_h V6GW_h | tail -1; python3 log/claude_g1rand_20260925/mcpair.py LXW_p V6GW_p push | tail -1; python3 log/claude_g1rand_20260925/mcpair.py LXW_lat V6GW_lat push | tail -1
echo "--- 低い段"; python3 log/claude_g1rand_20260925/mcpair.py LX_low V6G_low | tail -1; echo "#### POST_CEM2_DONE"; } > log/claude_g1rand_20260925/post_cem2.out 2>&1
