set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
L=log/claude_g1rand_20260925; C=log/claude_g1champion_20261020v612/controls.json; BIN=$L/kyo46rs_walk_tu36
for v in rsj:'{"MPC_FOOTSTEP_SHIFT_REST":"1"}' rsjx:'{"MPC_FOOTSTEP_SHIFT_REST":"1","MPC_FOOTSTEP_MAX_X":"0.25"}'; do n=${v%%:*}; x=${v#*:}
  TAG=GREP4_$n BASE=$C BIN=$BIN MODE=push N=48 W=12 SEED=20260924 EXTRA="$x" CONDS=11@0,11@180,14@180,7@90,5@270 python3 log/claude_g1rand_20260924/rand.py | sed "s/^/$n /"
done
echo "#### GREP4_DONE"
