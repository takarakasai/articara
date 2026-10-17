set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
L=log/claude_g1rand_20260925; C=log/claude_g1champion_20261020v611/controls.json; BIN=$L/kyo46rs_walk_tu32
C8=0@0,8@0,11@0,3@270,5@270,7@90,11@180,14@180
for f in 0.80 0.90; do n=f${f#0.}
  TAG=GFRAC_p_$n BASE=$C BIN=$BIN MODE=push N=48 W=12 SEED=20260924 EXTRA="{\"REF_TRIG_MAX_FRAC\":\"$f\"}" CONDS=$C8 python3 log/claude_g1rand_20260924/rand.py | sed "s/^/p $n /"
  TAG=GFRAC_lat_$n BASE=$C BIN=$BIN MODE=push N=24 W=12 SEED=20260924 EXTRA="{\"REF_TRIG_MAX_FRAC\":\"$f\"}" CONDS=3@270,4@270,5@270,6@270,3@90,5@90,7@90 python3 log/claude_g1rand_20260924/rand.py | sed "s/^/lat $n /"
done
for n in f80 f90; do python3 $L/mcpair.py GROT_p_base GFRAC_p_$n push; python3 $L/mcpair.py GROT_lat_base GFRAC_lat_$n push; done
echo "#### GFRAC1_DONE"
