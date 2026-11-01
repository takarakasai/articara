import json, re, subprocess, os, sys, glob, random
from concurrent.futures import ThreadPoolExecutor
G='log/claude_g1rand_20260924'; BIN='log/claude_g1rand_20260925/kyo46rs_walk_tu65'; O=sys.argv[1]
fx=json.load(open('/home/takara/work/dp/misa-locomotion/misa-biped/tests/fixtures/kyo46rs_v4.json'))
src=open('/home/takara/work/dp/misa-locomotion/misa-biped/src/kv.rs').read(); fn=src[src.index('pub fn config_from_kv'):]
cfgkeys=(set(re.findall(r'"([A-Z][A-Z0-9_]+)"', fn))|{'friction_mu'})-{'WALKER_PRESET'}
cls={}
for l in open('doc/from_claude/20261029_knob_inventory.md').read().splitlines():
    if not l.startswith('| '): continue
    cols=[x.strip() for x in l.strip().strip('|').split('|')]
    if len(cols)>=5 and re.fullmatch(r'`?[A-Za-z][A-Za-z0-9_*]*`?', cols[0]): cls[cols[0].strip('`')]=cols[4]
kclass=lambda k: cls.get('WBCW_*') if k.startswith('WBCW_') else cls.get(k)
random.seed(1)
picks=[]
for b in ['kp','khold','kw','kdesc','kflat','kbump05','kbump10','ksense']:
    for s in 'not':
        ms=sorted(glob.glob(f'{G}/KFAM_kl9{s}_{b}/*/m*'))
        picks.append(random.choice(ms))
picks=picks[::4]+[p for p in picks if 'kdesc' in p or 'kbump05' in p]
picks=sorted(set(picks))
env0=dict(os.environ); env0['LD_LIBRARY_PATH']='/home/takara/.mujoco/mujoco-3.8.0/lib'
def run(c, out):
    e=dict(env0); e.update({k:str(v) for k,v in c.items()})
    r=subprocess.run([BIN],env=e,capture_output=True,text=True); open(out,'w').write(r.stdout+r.stderr)
def one(p):
    c=json.load(open(p+'/controls.json')); id=p.replace(G+'/','').replace('/','__')
    drop=[k for k in c if kclass(k) in {'1','2','1k'} and (k in cfgkeys or k.startswith('WBCW_'))]
    c2={k:v for k,v in c.items() if k not in drop}; c2['WALKER_PRESET']='kyo46rs_v4'
    run(c, f'{O}/{id}.full.log'); run(c2, f'{O}/{id}.preset.log')
    ref=open(p+'/run.log').read(); a=open(f'{O}/{id}.full.log').read(); b=open(f'{O}/{id}.preset.log').read()
    return f"{id}: full==family {a==ref}  preset==full {a==b}  dropped {len(drop)}"
with ThreadPoolExecutor(18) as ex:
    for l in ex.map(one, picks): print(l, flush=True)
