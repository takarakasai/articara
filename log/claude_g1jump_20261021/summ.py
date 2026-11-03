import re,glob,statistics as st,sys,os
D='/home/takara/work/dp/articara/log/claude_g1jump_20261021/runs'
base=sys.argv[1]; tags=sys.argv[2:]
def load(tag):
  out={}
  for f in glob.glob(f'{D}/{tag}/v*/m*/run.log'):
    t=open(f).read(); c=f.split('/')[-3]; m=f.split('/')[-2]
    v=re.search(r'verdict: (\S+)',t); v=v.group(1) if v else '?'
    x=re.search(r'forward: lift-off vx (\S+), touchdown CoM dx (\S+), final CoM dx (\S+)',t)
    lo=re.search(r'\[lift-off\][^\n]*h_ang \(([^,]+),([^,]+),[^\n]*pitch (\S+)',t)
    fb=None
    if x and v=='FELL': fb='F' if float(x.group(3))>0 else 'B'
    out[(c,m)]=dict(v=v,vx=float(x.group(1)) if x and x.group(1)!='NaN' else None,dx=float(x.group(3)) if x else None,hy=float(lo.group(2)) if lo else None,pitch=float(lo.group(3)) if lo else None,fb=fb)
  return out
B=load(base)
med=lambda a: st.median(a) if a else float('nan')
for tag in [base]+tags:
  A=load(tag); 
  for c in sorted({k[0] for k in A}):
    ks=sorted(k for k in A if k[0]==c and k in B)
    L=[A[k]['v'] for k in ks]
    up=sum(1 for k in ks if A[k]['v']=='LANDED' and B[k]['v']!='LANDED'); dn=sum(1 for k in ks if A[k]['v']!='LANDED' and B[k]['v']=='LANDED')
    vx=[A[k]['vx'] for k in ks if A[k]['vx'] is not None]; dx=[A[k]['dx'] for k in ks if A[k]['v']=='LANDED']
    hy=[A[k]['hy'] for k in ks if A[k]['hy'] is not None]; pt=[A[k]['pitch'] for k in ks if A[k]['pitch'] is not None]
    fb=[A[k]['fb'] for k in ks if A[k]['fb']]
    print(f"{tag:9s} {c}: LANDED {L.count('LANDED')}/{len(ks)} (vs {base} +{up}/-{dn}) fell F{fb.count('F')} B{fb.count('B')} | lo vx {med(vx):+.3f} h_y {med(hy):+.2f} pitch {med(pt):+.2f} | final dx med {med(dx):+.3f} max {max(dx) if dx else float('nan'):+.3f}")
