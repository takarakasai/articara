set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
L=log/claude_g1rand_20260925; BIN=$L/kyo46rs_walk_tu48; C=log/claude_g1champion_20261022v614/controls.json
NOISE="SENSE_NOISE_QD=0.1;SENSE_NOISE_V=0.03;SENSE_NOISE_W=0.03"
for v in f40:'{"SENSE_FILTER_HZ":"40"}' f80:'{"SENSE_FILTER_HZ":"80"}' f20:'{"SENSE_FILTER_HZ":"20"}' both_f40:'{"SENSE_FILTER_HZ":"40","MOM_CMP":"0","REF_TRIG_DCM":"10"}' both_f80:'{"SENSE_FILTER_HZ":"80","MOM_CMP":"0","REF_TRIG_DCM":"10"}'; do n=${v%%:*}; x=${v#*:}
  MODE=flat N=48 W=16 SEED=20260924 T=25 TAG=GSENSE3_$n BASE=$C BIN=$BIN EXTRA="$x" CONDS="$NOISE" python3 log/claude_g1rand_20260924/rand.py | sed "s/^/noise $n /"
done
export PUSH_STAIR_X=30
for v in f40:'{"SENSE_FILTER_HZ":"40"}' f80:'{"SENSE_FILTER_HZ":"80"}'; do n=${v%%:*}; x=${v#*:}
  TAG=GSENSE3_push_$n BASE=$C BIN=$BIN MODE=push N=48 W=16 SEED=20260924 EXTRA="$x" CONDS=11@0,11@180,7@90,5@270 python3 log/claude_g1rand_20260924/rand.py | sed "s/^/push $n /"
  echo "=== clean push v6.14 -> filter $n"; python3 $L/gpair.py GCOMB_flat GSENSE3_push_$n
done
echo "#### GSENSE3_DONE"
