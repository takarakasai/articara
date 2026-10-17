set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
L=log/claude_g1rand_20260925; C=log/claude_g1champion_20261020v612/controls.json; BIN=$L/kyo46rs_walk_tu35
FL='"MPC_FOOTSTEP_CAPTURE_FLOOR":"0.5"'; TD='"PLAN_REANCHOR_TD":"0.30"'; RS='"MPC_FOOTSTEP_SHIFT_REST":"1"'
for v in fl:"{$FL}" td:"{$TD}" rs:"{$RS}" tdrs:"{$TD,$RS}" fltd:"{$FL,$TD}" flrs:"{$FL,$RS}"; do n=${v%%:*}; x=${v#*:}
  TAG=GREP2_$n BASE=$C BIN=$BIN MODE=push N=48 W=12 SEED=20260924 EXTRA="$x" CONDS=11@0,14@180 python3 log/claude_g1rand_20260924/rand.py | sed "s/^/$n /"
done
echo "#### GREP2_DONE"
