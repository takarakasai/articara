set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
L=log/claude_g1rand_20260925; C=log/claude_g1champion_20261020v612/controls.json; BIN=$L/kyo46rs_walk_tu43
for tr in 0.05 0.08; do n=t${tr#0.}
  TAG=GV614d_$n BASE=$C BIN=$BIN MODE=push N=48 W=12 SEED=20260924 EXTRA="{\"PUSH_REPLAN\":\"$tr\",\"PUSH_REPLAN_BACK_ONLY\":\"1\",\"PUSH_REPLAN_VMAX\":\"0.15\"}" CONDS=11@180,14@180 python3 log/claude_g1rand_20260924/rand.py | sed "s/^/$n /"
  python3 $L/mcpair.py GREP_p_base GV614d_$n push | grep -v '^GV'
done
echo "#### GV614D_DONE"
