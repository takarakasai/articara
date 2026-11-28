#!/usr/bin/env python3
"""spd.py DIR ...: for each condition of a kr05.sh / wr05.sh output dir, survivors and the speed the survivors
achieved after the ramp (the run log's 'achieved after the ramp ... (N%)' of the commanded vx): median and the
count under 40 %. Survival alone hides a robot that stands but marks time (11-28: kyo's payload 46 / 48 at 10..16 %)."""
import re, sys, pathlib, statistics as st
for d in map(pathlib.Path, sys.argv[1:]):
    print(f"== {d}")
    for c in sorted(p for p in d.iterdir() if p.is_dir()):
        v = []; n = 0
        for f in c.glob('st*_s*.log'):
            n += 1; t = f.read_text(errors='replace')
            if 'verdict: SURVIVED' not in t: continue
            m = re.search(r'achieved after the ramp.*?\((-?\d+)%\)', t)
            if m: v.append(int(m.group(1)))
        med = f"{st.median(v):5.1f}" if v else "    -"
        print(f"{c.name:<8} {len(v):>3} / {n:<3} speed median {med} %  under 40 %: {sum(x < 40 for x in v)}")
