"""One run's subtitle for the turning-jump video: jumps done and the soles'
total turn (unwrapped over the [yaw] lines).  python3 rtsumm1.py run.log"""
import re, sys, math
t = open(sys.argv[1]).read(); fell = 'verdict: FELL' in t
n = len(re.findall(r'\[jump-again\]', t)) + (0 if fell else 1)
acc, prev = 0.0, None
for a, b in re.findall(r'\[yaw\] t=\S+ nc \d pelvis \S+ feet (\S+) (\S+)', t):
    y = math.atan2(math.sin(float(a)) + math.sin(float(b)), math.cos(float(a)) + math.cos(float(b)))
    if prev is not None: acc += (y - prev + math.pi) % (2 * math.pi) - math.pi
    prev = y
print(f'{n} jumps landed, feet turned {math.degrees(acc):+.0f} deg' + (' (falls)' if fell else ''))
