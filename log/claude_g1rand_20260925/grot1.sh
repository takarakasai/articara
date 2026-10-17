set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
L=log/claude_g1rand_20260925; C=log/claude_g1champion_20261020v611/controls.json; BIN=$L/kyo46rs_walk_tu30
C8=0@0,8@0,11@0,3@270,5@270,7@90,11@180,14@180
for v in base:'{}' rot3:'{"SWING_ROT":"3"}'; do n=${v%%:*}; x=${v#*:}
  TAG=GROT_p_$n BASE=$C BIN=$BIN MODE=push N=48 W=12 SEED=20260924 EXTRA="$x" CONDS=$C8 python3 log/claude_g1rand_20260924/rand.py | sed "s/^/p $n /"
  TAG=GROT_lat_$n BASE=$C BIN=$BIN MODE=push N=24 W=12 SEED=20260924 EXTRA="$x" CONDS=3@270,4@270,5@270,6@270,3@90,5@90,7@90 python3 log/claude_g1rand_20260924/rand.py | sed "s/^/lat $n /"
done
python3 $L/mcpair.py GROT_p_base GROT_p_rot3 push; python3 $L/mcpair.py GROT_lat_base GROT_lat_rot3 push
echo "#### GROT1_DONE"
