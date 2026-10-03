"""Randomised verification harness (order 0 of the 2026-09-24 review synthesis).

A "member" is a SITUATION, not a rounding perturbation. Each member index k
picks, from one seeded stream:
  - INIT_JITTER/INIT_SEED : initial leg posture, +-JITTER rad per joint
  - MU_GROUND             : ground friction, uniform in [MU_LO, MU_HI]
  - stair start x         : 2.00 + uniform(-DX, DX)   (stairs runs only)
  - PUSH_STEP / PUSH_AT   : which step and how far into it (push runs only)
The old harnesses moved KD_COM by k*1e-9, which samples the chaotic spread
around ONE scenario; under the weighted QP (which is not chaotic) all members
then returned the same answer, i.e. N was really 1.

Env: TAG, BASE, BIN, N, W, EXTRA(JSON), MODE=stairs|push, RISERS or CONDS,
     K0 (first member index), T, JITTER, MU_LO, MU_HI, DX, SEED.
Writes TAG/<case>/m<k>/ and prints one line per case.
"""
import json, os, random, re, subprocess, statistics as st
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor

ROOT = Path('log/claude_g1rand_20260924')
D = ROOT / os.environ['TAG']; D.mkdir(parents=True, exist_ok=True)
BIN = Path(os.environ['BIN']).resolve()
MODE = os.environ.get('MODE', 'stairs')
# MODE=flat: CONDS are "KEY=VAL;KEY=VAL" control overrides per case (speed
# change, sensor delay/noise, ...), run on flat ground with the push harness's
# situation draw (no push). START_X: stair start (default 2.00); a descent
# needs ~0.40 so the platform reaches back under the spawn.
START_X = float(os.environ.get('START_X', '2.00'))
# STAIR_N / STAIR_TREAD: flight length and tread depth of the stairs runs
# (defaults reproduce the historical single 0.211 m step).
# FLAT_START_X: where the zero-riser "stair" of MODE=flat sits. The stair
# planner takes over the footfalls from its stand line on (fixed 0.199 m
# strides), so a speed benchmark must put it out of reach (e.g. 30).
FLAT_START_X = float(os.environ.get('FLAT_START_X', '2.00'))
STAIR_N = os.environ.get('STAIR_N', '1'); STAIR_TREAD = os.environ.get('STAIR_TREAD', '0.211')
c0 = json.load(open(os.environ['BASE']))
for k in ('TRAJ_CSV', 'SUBSTEP_LIMIT_AUDIT_JSON'):
    c0.pop(k, None)
c0.update(json.loads(os.environ.get('EXTRA', '{}')))
c0['T'] = os.environ.get('T', '40' if MODE == 'stairs' else '20')
c0['STAIR_LANDING_LEN'] = c0.get('STAIR_LANDING_LEN', '3.0')
N = int(os.environ.get('N', '48')); K0 = int(os.environ.get('K0', '0'))
JIT = os.environ.get('JITTER', '0.01')
MU_LO = float(os.environ.get('MU_LO', '0.60')); MU_HI = float(os.environ.get('MU_HI', '0.80'))
DX = float(os.environ.get('DX', '0.02')); SEED = int(os.environ.get('SEED', '20260924'))
CASES = (os.environ['RISERS'].split(',') if MODE == 'stairs'
         else [c for c in os.environ['CONDS'].split(',') if c] if MODE == 'flat'
         else [tuple(x.split('@')) for x in os.environ['CONDS'].split(',')])


def situation(k):
    """Everything member k varies. Keyed on (SEED, k) so the same member index
    is the same situation for every condition -- paired comparisons."""
    r = random.Random(f"{SEED}:{k}")
    return {'mu': round(r.uniform(MU_LO, MU_HI), 4),
            'dx': round(r.uniform(-DX, DX), 4),
            'step': r.choice([7, 8, 9]),
            'at': round(r.uniform(0.25, 0.75), 3)}


def one(a):
    case, k = a
    s = situation(k)
    name = (f'r{case}' if MODE == 'stairs' else case.replace('=', '').replace(';', '_').replace('.', 'p')
            if MODE == 'flat' else f'i{case[0]}_d{case[1]}')
    d = D / name / f'm{k:02d}'; d.mkdir(parents=True, exist_ok=True)
    lg = d / 'run.log'
    if lg.exists() and 'verdict:' in lg.read_text():
        return
    c = dict(c0)
    c['INIT_JITTER'] = JIT; c['INIT_SEED'] = str(k + 1); c['MU_GROUND'] = str(s['mu'])
    if MODE == 'stairs':
        c['STAIRS'] = f"{STAIR_N},{case},{STAIR_TREAD},{START_X + s['dx']:.4f}"
    elif MODE == 'flat':
        c['STAIRS'] = f"1,0.000,0.211,{FLAT_START_X + s['dx']:.4f}"
        for kv in case.split(';'):
            if kv:
                kk, vv = kv.split('=', 1)
                c[kk] = vv
    else:
        # Flat = the stair recipe with a zero riser (see push.py).
        c['STAIRS'] = f"1,0.000,0.211,{2.00 + s['dx']:.4f}"
        c['PUSH_IMPULSE'] = case[0]; c['PUSH_DEG'] = case[1]
        c['PUSH_STEP'] = str(s['step']); c['PUSH_AT'] = str(s['at'])
        c['PUSH_SUPPORT'] = os.environ.get('PUSH_SUPPORT', 'ss')
    (d / 'controls.json').write_text(json.dumps(c, indent=1))
    e = dict(os.environ); e.update({k2: str(v) for k2, v in c.items()})
    try:
        r = subprocess.run([str(BIN)], env=e, capture_output=True, text=True, timeout=1200)
        lg.write_text(r.stdout + r.stderr)
    except subprocess.TimeoutExpired as ex:
        # A hung solve must not take the whole battery down (2026-10-02: one
        # weighted-QP stair member ran 65 min, the executor re-raised and the
        # summary never printed). Count it as a fall at the timeout.
        out = (ex.stdout or b'') if isinstance(ex.stdout, bytes) else (ex.stdout or '')
        if isinstance(out, bytes):
            out = out.decode(errors='replace')
        lg.write_text(out + f'\n  FELL at t=TIMEOUT (1200 s wall)\n  verdict: TIMEOUT\n')


jobs = [(c, k) for c in CASES for k in range(K0, K0 + N)]
with ThreadPoolExecutor(max_workers=int(os.environ.get('W', '24'))) as p:
    list(p.map(one, jobs))
for case in CASES:
    name = (f'r{case}' if MODE == 'stairs' else case.replace('=', '').replace(';', '_').replace('.', 'p')
            if MODE == 'flat' else f'i{case[0]}_d{case[1]}')
    surv = 0; tf = []; fired = 0
    for k in range(K0, K0 + N):
        t = (D / name / f'm{k:02d}' / 'run.log').read_text()
        if 'verdict: SURVIVED' in t:
            m_steps = re.search(r'steps taken: (\d+)', t)
            if MODE != 'flat' or int(os.environ.get('FLAT_MIN_STEPS', '0')) <= (int(m_steps.group(1)) if m_steps else 0):
                surv += 1
        m = re.search(r'FELL at t=([0-9.]+)', t)
        if m:
            tf.append(float(m.group(1)))
        if re.search(r'push: [0-9.]+ N\*s', t):
            fired += 1
    extra = f' 発火 {fired}/{N}' if MODE == 'push' else ''
    print(f'  {name}: 生存 {surv}/{N}  転倒中央値 {st.median(tf) if tf else 0:.2f}s'
          f'  転倒時刻の種類 {len(set(round(x,3) for x in tf))}{extra}', flush=True)
