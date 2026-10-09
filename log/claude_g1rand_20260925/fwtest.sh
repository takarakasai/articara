set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
L=log/claude_g1rand_20260925; C=log/claude_g1champion_20261010v66/controls.json; BIN=$L/kyo46rs_walk_fw1; T=/data/articara/trajopt
until [ -f $T/chain14.done ]; do sleep 30; done
python3 - <<'PY'
import json
T='/data/articara/trajopt/'
for src,ph in [('fw11_06',0.6),('fw11_07',0.7),('fw11_04',0.4),('dw5_07',0.7)]:
    try:
        d=json.load(open(T+src+'.json')); d['swing_phase']=ph; json.dump(d,open(T+src+'.json','w'))
    except Exception as e: print(src,'missing')
PY
for f in fw11_04 fw11_06 fw11_07 dw5_07; do echo "== $f: $(grep -E 'solve time|NOT' $T/$f.log | head -1 | cut -c1-60) | $(grep -E 'foot lands' $T/$f.log | tr '\n' ' ')"; done
D=log/claude_g1champion_20261010v66
LIBF="$D/ref_walk5_phase03.json,$D/ref_walk5_phase05.json,$T/fw11_04.json,$T/fw11_06.json,$T/fw11_07.json"
for thr in 0.03 0.045; do TAG=FW${thr}_p BASE=$C BIN=$BIN MODE=push N=48 W=12 SEED=20260924 EXTRA="{\"REF_LIBRARY\":\"$LIBF\",\"REF_TRIG_DCM_X\":\"$thr\",\"REF_TRIG_MAX_FRAC\":\"0.85\"}" CONDS=11@0,8@0,0@0,7@90 python3 log/claude_g1rand_20260924/rand.py; done
LIBL="$D/ref_walk5_phase03.json,$D/ref_walk5_phase05.json,$T/dw5_07.json"
TAG=LT7_p BASE=$C BIN=$BIN MODE=push N=48 W=12 SEED=20260924 EXTRA="{\"REF_LIBRARY\":\"$LIBL\",\"REF_TRIG_MAX_FRAC\":\"0.85\"}" CONDS=7@90,5@90 python3 log/claude_g1rand_20260924/rand.py
{ for thr in 0.03 0.045; do echo "=== forward refs, REF_TRIG_DCM_X $thr vs v6.6"; python3 $L/mcpair.py ZE0.035_p FW${thr}_p push; done; echo "=== late-phase lateral ref (0.7) + maxfrac 0.85 vs v6.6"; python3 $L/mcpair.py ZE0.035_p LT7_p push; echo "#### FWTEST_DONE"; } > $L/fwtest_summary.out 2>&1
