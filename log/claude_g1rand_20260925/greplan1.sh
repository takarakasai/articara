set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
L=log/claude_g1rand_20260925; C=log/claude_g1champion_20261020v612/controls.json; BIN=$L/kyo46rs_walk_tu35
C8=0@0,8@0,11@0,3@270,5@270,7@90,11@180,14@180
for v in base:'{}' a:'{"PLAN_REANCHOR_TD":"0.30","MPC_FOOTSTEP_SHIFT_REST":"1","MPC_FOOTSTEP_CAPTURE_FLOOR":"0.5","MPC_FOOTSTEP_MAX_X":"0.25"}' b:'{"PLAN_REANCHOR_TD":"0.30","MPC_FOOTSTEP_SHIFT_REST":"1","MPC_FOOTSTEP_CAPTURE_FLOOR":"0.5"}'; do n=${v%%:*}; x=${v#*:}
  TAG=GREP_p_$n BASE=$C BIN=$BIN MODE=push N=48 W=12 SEED=20260924 EXTRA="$x" CONDS=$C8 python3 log/claude_g1rand_20260924/rand.py | sed "s/^/p $n /"
  TAG=GREP_lat_$n BASE=$C BIN=$BIN MODE=push N=24 W=12 SEED=20260924 EXTRA="$x" CONDS=3@270,4@270,5@270,6@270,3@90,5@90,7@90 python3 log/claude_g1rand_20260924/rand.py | sed "s/^/lat $n /"
done
for n in a b; do python3 $L/mcpair.py GREP_p_base GREP_p_$n push; python3 $L/mcpair.py GREP_lat_base GREP_lat_$n push; done
echo "#### GREP1_DONE"
