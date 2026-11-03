#!/bin/bash
# R8TC の 1.5 を j39（押しの始まりの重心と足の中央のずれ [push-start] を出す、他は同一）で回し直す。
D=/home/takara/work/dp/articara/log/claude_g1jump_20261021; B=$D/g1_flight_probe_j39
cd $D
python3 - <<'PY'
import json,glob,os,subprocess
from concurrent.futures import ThreadPoolExecutor
D='/home/takara/work/dp/articara/log/claude_g1jump_20261021'
def one(m):
    src=f'{D}/runs/R8TC/v1.5/{m}'; dst=f'{D}/runs/R9PS/v1.5/{m}'; os.makedirs(dst,exist_ok=True)
    e=json.load(open(f'{src}/env.json')); json.dump(e,open(f'{dst}/env.json','w'),indent=1)
    env=dict(os.environ); env.update(e); env['LD_LIBRARY_PATH']='/home/takara/.mujoco/mujoco-3.8.0/lib'
    r=subprocess.run([f'{D}/g1_flight_probe_j39'],env=env,capture_output=True,text=True,cwd='/home/takara/work/dp/articara')
    open(f'{dst}/run.log','w').write(r.stdout+r.stderr)
with ThreadPoolExecutor(16) as ex: list(ex.map(one,[f'm{k:02d}' for k in range(48)]))
PY
python3 rsumm.py R8TC R9PS
echo "#### JB58_DONE"
