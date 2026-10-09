set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
L=log/claude_g1rand_20260925; C=log/claude_g1champion_20261008v65/controls.json; BIN=$L/kyo46rs_walk_zd1; T=/data/articara/trajopt
until [ -f $T/chain12.done ]; do sleep 30; done
python3 - <<'PY'
import json
T='/data/articara/trajopt/'
for src,ph in [('dw5_02',0.2),('dw5_04',0.4),('dw5_06',0.6),('dw7_02',0.2),('dw7_04',0.4),('dw7_05',0.5),('dw7_06',0.6)]:
    try:
        d=json.load(open(T+src+'.json')); d['swing_phase']=ph; json.dump(d,open(T+src+'.json','w'))
    except Exception as e: print(src,'missing',e)
PY
for f in dw7_05 dw7_04 dw5_04 dw7_02 dw5_02 dw7_06 dw5_06; do echo "== $f: $(grep -E 'solve time|NOT' $T/$f.log | head -1 | cut -c1-60)"; done
LIB="$T/dw5_02.json,$T/dwk_03.json,$T/dw5_04.json,$T/dwk_05.json,$T/dw5_06.json,$T/dw7_02.json,$T/dwk7_03.json,$T/dw7_04.json,$T/dw7_05.json,$T/dw7_06.json"
TAG=RL2_p BASE=$C BIN=$BIN MODE=push N=48 W=12 SEED=20260924 EXTRA="{\"REF_LIBRARY\":\"$LIB\"}" CONDS=5@90,7@90,11@180 python3 log/claude_g1rand_20260924/rand.py
TAG=RL2_lat BASE=$C BIN=$BIN MODE=push N=24 W=12 SEED=20260924 EXTRA="{\"REF_LIBRARY\":\"$LIB\"}" CONDS=3@270,4@270,5@270,6@270,3@90,5@90,7@90 python3 log/claude_g1rand_20260924/rand.py
{ echo "=== expanded library vs v6.5 (std push subset)"; python3 $L/mcpair.py RFE_p RL2_p push; echo "=== lat"; python3 $L/mcpair.py RFE_lat RL2_lat push; echo "#### REFLIB2_DONE"; } > $L/reflib2_summary.out 2>&1
