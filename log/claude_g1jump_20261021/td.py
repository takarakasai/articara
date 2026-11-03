import re,glob,sys,statistics as st
D='/home/takara/work/dp/articara/log/claude_g1jump_20261021/runs'
for tag in sys.argv[1:]:
  for vd in sorted(glob.glob(f'{D}/{tag}/v*')):
    rows=[]
    for f in sorted(glob.glob(f'{vd}/m*/run.log')):
      t=open(f).read()
      lo=re.search(r'\[lift-off\] t=(\S+) com \((\S+), z (\S+)\) v \((\S+), (\S+)\), foot x \["(\S+)", "(\S+)"\]',t)
      td=re.search(r'\[touchdown\] t=(\S+) after (\S+) s, com \((\S+), z (\S+)\) v \((\S+), (\S+)\), foot x \["(\S+)", "(\S+)"\]',t)
      v=re.search(r'verdict: (\S+)',t); x=re.search(r'final CoM dx (\S+)',t)
      if not(lo and td and v): continue
      fx=float(x.group(1)) if x else 0
      rows.append(dict(ok=v.group(1)=='LANDED',fb=('F' if fx>0 else 'B'),T=float(td.group(2)),zlo=float(lo.group(3)),ztd=float(td.group(4)),vz=float(lo.group(5)),vx=float(lo.group(4)),
        ftd=(float(td.group(7))+float(td.group(8)))/2-float(td.group(3))))
    if not rows: continue
    m=lambda k,c: st.median([r[k] for r in rows if c(r)]) if [r for r in rows if c(r)] else float('nan')
    for name,c in [('LANDED',lambda r:r['ok']),('FELL-F',lambda r:not r['ok'] and r['fb']=='F'),('FELL-B',lambda r:not r['ok'] and r['fb']=='B')]:
      n=sum(1 for r in rows if c(r))
      if n: print(f"{tag:12s} {vd.split('/')[-1]:5s} {name:6s} n={n:2d} T {m('T',c):.3f} vz {m('vz',c):+.2f} vx {m('vx',c):+.2f} z_lo {m('zlo',c):.3f} z_td {m('ztd',c):.3f} dz {m('ztd',c)-m('zlo',c):+.3f} foot-com@td {m('ftd',c):+.3f}")
