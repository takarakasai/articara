import re,glob,sys,statistics as st
D='/home/takara/work/dp/articara/log/claude_g1jump_20261021/runs'
for tag in sys.argv[1:]:
  for vd in sorted(glob.glob(f'{D}/{tag}/v*')):
    R=[]
    for f in sorted(glob.glob(f'{vd}/m*/run.log')):
      t=open(f).read()
      v=re.search(r'verdict: (\S+)',t); a=re.search(r'first touchdown vz (\S+) m/s',t); b=re.search(r'max tilt after touchdown: (\S+) rad, min CoM z after touchdown (\S+)',t)
      p=re.search(r'\[touchdown\].*?\n(?:.*\n){0,3}?\s+t=\s*\S+ nc=\d com=\S+ vz=\S+ roll=(\S+) pitch=(\S+)',t)
      x=re.search(r'final CoM dx (\S+)',t)
      if not(v and a and b): continue
      R.append(dict(o=v.group(1)=='LANDED' and 'L' or ('F' if x and float(x.group(1))>0 else 'B'),vz=float(a.group(1)),tilt=float(b.group(1)),zmin=float(b.group(2)),pitch=float(p.group(2)) if p else float('nan')))
    for o in 'LFB':
      rr=[r for r in R if r['o']==o]
      if rr: print(f"{tag:8s} {vd.split('/')[-1]:5s} {o} n={len(rr):2d} td vz {st.median(r['vz'] for r in rr):+.2f}  zmin {st.median(r['zmin'] for r in rr):.3f}  pitch~70ms {st.median(r['pitch'] for r in rr):+.3f}")
