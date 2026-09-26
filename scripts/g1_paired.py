#!/usr/bin/env python3
"""対応あり McNemar。使い方: mcpair.py <基準TAG> <候補TAG> [stairs|push]"""
import glob,os,re,math,sys
R='log/claude_g1rand_20260924'; mode=sys.argv[3] if len(sys.argv)>3 else 'stairs'
pat='r*' if mode=='stairs' else 'i*'
def surv(tag):
    o={}
    for rd in sorted(glob.glob(f'{R}/{tag}/{pat}')):
        for md in sorted(glob.glob(rd+'/m*')):
            try: v=re.search(r'verdict:\s*(\w+)',open(md+'/run.log').read())
            except OSError: continue
            if v: o[(os.path.basename(rd),os.path.basename(md))]=v.group(1)=='SURVIVED'
    return o
a=surv(sys.argv[1]); b=surv(sys.argv[2]); ks=sorted(set(a)&set(b)); U=D=0
print(f"{sys.argv[2]} 対 {sys.argv[1]}  （共通 {len(ks)} 本）")
for c in sorted({k[0] for k in ks}):
    kk=[k for k in ks if k[0]==c]
    u=sum(1 for k in kk if b[k] and not a[k]); d=sum(1 for k in kk if a[k] and not b[k])
    z=(u-d)/math.sqrt(u+d) if u+d else 0.0
    print(f"  {c:10s} {sum(a[k] for k in kk):3d} → {sum(b[k] for k in kk):3d}  改善{u:3d} 悪化{d:3d}  z={z:+5.2f}"); U+=u; D+=d
print(f"  {'総合':10s} {sum(a[k] for k in ks):3d} → {sum(b[k] for k in ks):3d}  改善{U:3d} 悪化{D:3d}  z={(U-D)/math.sqrt(U+D) if U+D else 0:+5.2f}")
