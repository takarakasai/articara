set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
L=log/claude_g1rand_20260925; C=log/claude_g1champion_20261020v611/controls.json; BIN=$L/kyo46rs_walk_tu34
C8=0@0,8@0,11@0,3@270,5@270,7@90,11@180,14@180
X='"MPC_FOOTSTEP_CROSS_Y":"0.20","MPC_FOOTSTEP_CROSS_FLOOR":"0.5","MPC_FOOTSTEP_CROSS_HOLD_Z":"0.05"'
for v in e02:'' e03:',"MPC_FOOTSTEP_CROSS_EX":"0.03"'; do n=${v%%:*}; x=${v#*:}
  TAG=GCROSS_p_$n BASE=$C BIN=$BIN MODE=push N=48 W=12 SEED=20260924 EXTRA="{$X$x}" CONDS=$C8 python3 log/claude_g1rand_20260924/rand.py | sed "s/^/p $n /"
  TAG=GCROSS_lat_$n BASE=$C BIN=$BIN MODE=push N=24 W=12 SEED=20260924 EXTRA="{$X$x}" CONDS=3@270,4@270,5@270,6@270,3@90,5@90,7@90 python3 log/claude_g1rand_20260924/rand.py | sed "s/^/lat $n /"
done
for n in e02 e03; do python3 $L/mcpair.py GROT_p_base GCROSS_p_$n push; python3 $L/mcpair.py GROT_lat_base GCROSS_lat_$n push; done
echo "#### GCROSS1_DONE"
