set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
L=log/claude_g1rand_20260925; C=log/claude_g1champion_20261008v65/controls.json; BIN=$L/kyo46rs_walk_pr1; T=/data/articara/trajopt
for pd in 0.03 0.05; do TAG=PB${pd}_p BASE=$C BIN=$BIN MODE=push N=48 W=12 SEED=20260924 EXTRA="{\"PUSH_Z_DROP\":\"0.035\",\"REF_TRIG_PRED\":\"$pd\"}" CONDS=11@180,14@180,11@0 python3 log/claude_g1rand_20260924/rand.py; done
until [ -f $T/chain13.done ]; do sleep 30; done
python3 - <<'PY'
import json
T='/data/articara/trajopt/'
for src,ph in [('ew5_03',0.3),('ew5_05',0.5)]:
    d=json.load(open(T+src+'.json')); d['swing_phase']=ph; json.dump(d,open(T+src+'.json','w'))
PY
for f in ew5_03 ew5_05; do echo "== $f: $(grep -E 'solve time|NOT' $T/$f.log | head -1 | cut -c1-60)"; done
LIB="$T/ew5_03.json,$T/ew5_05.json"
for pd in 0.04; do
 TAG=PL${pd}_lat BASE=$C BIN=$BIN MODE=push N=24 W=12 SEED=20260924 EXTRA="{\"REF_LIBRARY\":\"$LIB\",\"REF_TRIG_PRED\":\"$pd\"}" CONDS=3@270,4@270,5@270,6@270,3@90,5@90,7@90 python3 log/claude_g1rand_20260924/rand.py
 TAG=PL${pd}_p BASE=$C BIN=$BIN MODE=push N=48 W=12 SEED=20260924 EXTRA="{\"REF_LIBRARY\":\"$LIB\",\"REF_TRIG_PRED\":\"$pd\"}" CONDS=5@90,7@90,5@270 python3 log/claude_g1rand_20260924/rand.py
done
{ for pd in 0.03 0.05; do echo "=== drop 0.035 + pred $pd (back/front pushes) vs v6.6 cand"; python3 $L/mcpair.py ZE0.035_p PB${pd}_p push; done
  echo "=== lateral: pred 0.04 + delay-0.04 refs vs v6.5"; python3 $L/mcpair.py RFE_lat PL0.04_lat push; python3 $L/mcpair.py RFE_p PL0.04_p push; echo "#### PRED_DONE"; } > $L/pred_summary.out 2>&1
