set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
L=log/claude_g1rand_20260925; C=log/claude_g1champion_20261020v612/controls.json; BIN=$L/kyo46rs_walk_tu38
X='"MPC_FOOTSTEP_SHIFT_REST":"1","MPC_FOOTSTEP_HOLD_SHIFT":"0.06","MPC_FOOTSTEP_CAPTURE_FLOOR":"0.5","MPC_FOOTSTEP_BACK_ONLY":"1"'
for p in 0 0.02; do n=p${p#0.}
  TAG=GREP12_$n BASE=$C BIN=$BIN MODE=push N=48 W=12 SEED=20260924 EXTRA="{$X,\"MPC_FOOTSTEP_BACK_PERSIST\":\"$p\"}" CONDS=11@180,14@180,5@270,7@90 python3 log/claude_g1rand_20260924/rand.py | sed "s/^/$n /"
done
echo "#### GREP12_DONE"
