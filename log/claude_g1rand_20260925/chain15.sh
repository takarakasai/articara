cd /home/takara/work/dp/articara
P=/data/articara/venv_trajopt/bin/python; T=/data/articara/trajopt
X="--landed --swing-z 0 --swing-x 0.14 --width 0.12 --end-width 0.105 --cross 0.03 --cross-x 0.05 --phase 1.0 --delay 0 --vx 0.12 --impulse 7 --tmin 0.15,0.05,0.25,0.20 --max-iter 2500"
timeout 9000 $P -u scripts/trajopt/push_recovery.py $X --com-x 0.04 --com-y 0.078 --vy 0.10 --out $T/ld7_a.json > $T/ld7_a.log 2>&1
timeout 9000 $P -u scripts/trajopt/push_recovery.py $X --com-x 0.04 --com-y 0.09 --vy 0.16 --out $T/ld7_b.json > $T/ld7_b.log 2>&1
echo CHAIN15_DONE > $T/chain15.done
