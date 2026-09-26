set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
D=log/claude_g1bdbaseline_20260926/controls_v5_candidate_D.json; BIN=log/claude_g1rand_20260925/kyo46rs_walk_fzprobe
( echo "##### D + FREEZE 0.7: std 押し"; TAG=FZ07_p BASE=$D BIN=$BIN MODE=push N=48 W=13 SEED=20260924 EXTRA='{"MPC_FOOTSTEP_FREEZE_FRAC":"0.7"}' CONDS=0@0,8@0,11@0,3@270,5@270,7@90,11@180,14@180 python3 log/claude_g1rand_20260924/rand.py ) &
( echo "##### D + FREEZE 0.7: std 段A"; TAG=FZ07_s BASE=$D BIN=$BIN MODE=stairs N=48 W=13 SEED=20260924 EXTRA='{"MPC_FOOTSTEP_FREEZE_FRAC":"0.7"}' RISERS=0.020,0.040,0.060,0.080 python3 log/claude_g1rand_20260924/rand.py ) &
wait
echo "#### FZ2_DONE"
