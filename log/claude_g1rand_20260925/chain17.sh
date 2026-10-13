cd /home/takara/work/dp/articara
P=/data/articara/venv_trajopt/bin/python; T=/data/articara/trajopt
X="--landed --swing-z 0 --swing-x 0.14 --width 0.12 --end-width 0.105 --cross-x 0.05 --phase 1.0 --delay 0 --impulse 7 --tmin 0.15,0.05,0.25,0.20 --tmax 0.45,0.40,0.45,0.60 --max-iter 2500 --com-x 0.04 --com-y 0.078 --vy 0.10 --vx 0.12"
timeout 9000 $P -u scripts/trajopt/push_recovery.py $X --cross 0.09 --land-xmax 0.14 --out $T/ld7_e.json > $T/ld7_e.log 2>&1
timeout 9000 $P -u scripts/trajopt/push_recovery.py $X --cross 0.09 --land-xmax 0.18 --vy 0.14 --com-y 0.085 --out $T/ld7_f.json > $T/ld7_f.log 2>&1
echo CHAIN17_DONE > $T/chain17.done
