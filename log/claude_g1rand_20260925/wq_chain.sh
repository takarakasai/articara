set -u
cd /home/takara/work/dp/articara
until grep -q WQ_DONE log/claude_g1rand_20260925/wq.out; do sleep 60; done
W=6 bash log/claude_g1rand_20260925/wq_full.sh > log/claude_g1rand_20260925/wq_full.out 2>&1
L=log/claude_g1rand_20260925
{ date +%H:%M; echo "=== 重み付き QP 全域（WQF）対 v5.2: std 段A / holdout / 押し / 横 ==="
python3 $L/mcpair.py LX_s WQF_s | tail -1; python3 $L/mcpair.py LX_h WQF_h | tail -1; python3 $L/mcpair.py LX_p WQF_p push; python3 $L/mcpair.py LX_lat WQF_lat push | tail -1
echo "--- wide 段A / holdout / 押し / 横"; python3 $L/mcpair.py LXW_s WQFW_s | tail -1; python3 $L/mcpair.py LXW_h WQFW_h | tail -1; python3 $L/mcpair.py LXW_p WQFW_p push | tail -1; python3 $L/mcpair.py LXW_lat WQFW_lat push | tail -1
echo "--- 低い段"; python3 $L/mcpair.py LX_low WQF_low | tail -1; echo "#### WQ_CHAIN_DONE"; } > $L/wq_chain.out 2>&1
