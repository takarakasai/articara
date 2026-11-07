import re,glob,sys,statistics as st,collections
# 角度を狙う回す跳躍の集計: ttsumm.py TAG...（立って終えた本、狙いとの差、回した回数）
D='/home/takara/work/dp/articara/log/claude_g1jump_20261021/runs'
for tag in sys.argv[1:]:
  for c in sorted({f.split('/')[-3] for f in glob.glob(f'{D}/{tag}/v*/m*/run.log')}):
    E=[];J=collections.Counter();n=0;stand=0
    for f in sorted(glob.glob(f'{D}/{tag}/{c}/m*/run.log')):
      t=open(f).read(); n+=1
      v=re.search(r'verdict: (\S+)',t); m=re.search(r'turn target (\S+) deg: soles turned (\S+) deg at the end \(error (\S+)\), (\d+) turning jumps, reached (\S+)',t)
      if not(v and m and v.group(1)=='LANDED'): continue
      stand+=1; e=float(m.group(3)); E.append(e); J[int(m.group(4))]+=1
    a=[abs(e) for e in E]
    print(f"{tag:8s} {c}: standing {stand}/{n} | |err|<=5 {sum(x<=5 for x in a)} <=10 {sum(x<=10 for x in a)} | err med {st.median(E) if E else float('nan'):+.1f} |err| p90 {sorted(a)[int(0.9*len(a))-1] if a else float('nan'):.1f} max {max(a) if a else float('nan'):.1f} | jumps {dict(sorted(J.items()))}")
