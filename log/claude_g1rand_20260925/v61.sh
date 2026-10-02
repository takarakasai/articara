set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
C=log/claude_g1bdbaseline_20260926/controls_v61_candidate_gait_wq.json; BIN=log/claude_g1champion_20261003v6/kyo46rs_walk; L=log/claude_g1rand_20260925
W=10 BIN=$BIN scripts/g1_battery.sh V61 $C '{}' std
W=10 BIN=$BIN scripts/g1_battery.sh V61 $C '{}' wide
echo "##### V61 低い段"; TAG=V61_low BASE=$C BIN=$BIN MODE=stairs N=48 W=10 SEED=20260924 EXTRA='{}' RISERS=0.000,0.010,0.020,0.030 python3 log/claude_g1rand_20260924/rand.py
{ date +%H:%M; echo "=== V6.1（CEM 歩容 + WQ）対 v6（WQF）: std 段A / holdout / 押し / 横 ==="
python3 $L/mcpair.py WQF_s V61_s | tail -1; python3 $L/mcpair.py WQF_h V61_h | tail -1; python3 $L/mcpair.py WQF_p V61_p push; python3 $L/mcpair.py WQF_lat V61_lat push | tail -1
echo "--- wide"; python3 $L/mcpair.py WQFW_s V61W_s | tail -1; python3 $L/mcpair.py WQFW_h V61W_h | tail -1; python3 $L/mcpair.py WQFW_p V61W_p push | tail -1; python3 $L/mcpair.py WQFW_lat V61W_lat push | tail -1
echo "--- 低い段"; python3 $L/mcpair.py WQF_low V61_low | tail -1; echo "#### V61_DONE"; } > $L/v61_summary.out 2>&1
