#!/usr/bin/env python3
"""Randomised jump battery for examples/g1_flight_probe.

Each member k varies (keyed on SEED:k, so the same k is the same situation
for every condition and configuration -> paired comparisons):
  INIT_SEED / INIT_JITTER (leg posture), MU_GROUND, T_LAUNCH (crouch start),
  PLANT_MASS_SCALE.

Usage:
  TAG=... BIN=... EXTRA='{"K":"V",...}' CONDS=1.5,2.0 N=48 W=16 python3 jrand.py
  python3 jrand.py cmp TAG_A TAG_B
Results: log/claude_g1jump_20261021/runs/<TAG>/v<cond>/mNN/run.log
"""
import json, math, os, random, re, subprocess, sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

ROOT = Path('/home/takara/work/dp/articara')
RUNS = ROOT / 'log/claude_g1jump_20261021/runs'
BASE = {
    'ROBOT': 'g1', 'WBC_STRATEGY': 'weighted', 'T': '7',
    'FLIGHT_FEET': '1', 'LAND_ABSORB': '1', 'LAND_T': '0.20', 'JUMP_RELEASE': '1',
    'MOM_TASK': '1', 'WBCW_CONTACT': '1', 'WBCW_MOMENTUM': '1', 'CROUCH': '0.18', 'JUMP_Z_TO': '0.0',
}


def situation(seed, k):
    r = random.Random(f"{seed}:{k}")
    return {'INIT_SEED': str(k + 1), 'INIT_JITTER': '0.01',
            'MU_GROUND': f"{r.uniform(0.6, 0.8):.4f}",
            'T_LAUNCH': f"{r.uniform(0.8, 1.5):.3f}",
            'PLANT_MASS_SCALE': f"{r.uniform(0.95, 1.05):.4f}"}


def parse(log):
    t = log.read_text() if log.exists() else ''
    v = re.search(r'verdict:\s*(\S+)', t)
    lo = re.search(r'\[lift-off\] t=\S+ com \([^)]*\) v \([^,]*, ([^)]*)\).*?h_ang \(([^,]*),([^,]*),', t)
    td = re.search(r'\[touchdown\][^\n]*feet \[(\w+), (\w+)\]', t)
    return {'verdict': v.group(1) if v else None,
            'vto': float(lo.group(1)) if lo else None,
            'hy': float(lo.group(3)) if lo else None,
            'both': (td.group(1) == 'true' and td.group(2) == 'true') if td else None}


def load(tag):
    out = {}
    for log in (RUNS / tag).glob('*/m*/run.log'):
        out[(log.parent.parent.name, log.parent.name)] = parse(log)
    return out


def cmp(a, b):
    A, B = load(a), load(b)
    ks = sorted(set(A) & set(B))
    U = D = 0
    for c in sorted({k[0] for k in ks}):
        kk = [k for k in ks if k[0] == c]
        sa = sum(A[k]['verdict'] == 'LANDED' for k in kk); sb = sum(B[k]['verdict'] == 'LANDED' for k in kk)
        u = sum(1 for k in kk if B[k]['verdict'] == 'LANDED' and A[k]['verdict'] != 'LANDED')
        d = sum(1 for k in kk if A[k]['verdict'] == 'LANDED' and B[k]['verdict'] != 'LANDED')
        z = (u - d) / math.sqrt(u + d) if u + d else 0.0
        print(f"  {c:8s} {sa:3d} -> {sb:3d}  改善{u:3d} 悪化{d:3d}  z={z:+5.2f}")
        U += u; D += d
    z = (U - D) / math.sqrt(U + D) if U + D else 0.0
    print(f"  {'総合':8s} {sum(A[k]['verdict']=='LANDED' for k in ks):3d} -> {sum(B[k]['verdict']=='LANDED' for k in ks):3d}  改善{U:3d} 悪化{D:3d}  z={z:+5.2f}")


def main():
    if len(sys.argv) > 1 and sys.argv[1] == 'cmp':
        return cmp(sys.argv[2], sys.argv[3])
    tag = os.environ['TAG']; binp = os.environ['BIN']
    extra = json.loads(os.environ.get('EXTRA', '{}'))
    conds = os.environ.get('CONDS', '1.5').split(',')
    n = int(os.environ.get('N', '48')); w = int(os.environ.get('W', '16'))
    seed = int(os.environ.get('SEED', '31415'))
    jobs = [(c, k) for c in conds for k in range(n)]

    def one(job):
        c, k = job
        d = RUNS / tag / f"v{c}" / f"m{k:02d}"; d.mkdir(parents=True, exist_ok=True)
        log = d / 'run.log'
        if log.exists() and 'verdict:' in log.read_text():
            return
        env = dict(os.environ); env.update(BASE); env.update(situation(seed, k)); env['JUMP_V'] = c
        env.update({kk: str(vv) for kk, vv in extra.items()})
        if 'T_LAUNCH_ADD' in extra:  # e.g. one-leg hops: the shift and lift come first
            env['T_LAUNCH'] = f"{float(env['T_LAUNCH']) + float(extra['T_LAUNCH_ADD']):.3f}"
        env['LD_LIBRARY_PATH'] = '/home/takara/.mujoco/mujoco-3.8.0/lib'
        for drop in ('TRAJ_CSV', 'VERBOSE'):
            env.pop(drop, None)
        (d / 'env.json').write_text(json.dumps({kk: env[kk] for kk in list(BASE) + list(situation(seed, k)) + ['JUMP_V'] + list(extra)}, indent=1))
        r = subprocess.run([binp], env=env, capture_output=True, text=True, cwd=ROOT, timeout=1200)
        log.write_text(r.stdout + r.stderr)

    with ThreadPoolExecutor(w) as ex:
        list(ex.map(one, jobs))
    res = load(tag)
    for c in conds:
        rs = [res[(f"v{c}", f"m{k:02d}")] for k in range(n)]
        landed = sum(r['verdict'] == 'LANDED' for r in rs)
        nofl = sum(r['verdict'] == 'NO' or r['verdict'] == 'NOFLIGHT' or r['verdict'] is None for r in rs)
        vto = sorted(r['vto'] for r in rs if r['vto'] is not None)
        hy = sorted(abs(r['hy']) for r in rs if r['hy'] is not None)
        both = sum(1 for r in rs if r['both'])
        med = lambda x: x[len(x) // 2] if x else float('nan')
        print(f"{tag} JUMP_V={c}: 着地 {landed}/{n}  飛ばず {nofl}  離地速度中央値 {med(vto):.3f}  |h_y|中央値 {med(hy):.2f}  両足同時着地 {both}/{n}")


if __name__ == '__main__':
    main()
