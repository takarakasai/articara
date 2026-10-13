cd /home/takara/work/dp/articara
P=/data/articara/venv_trajopt/bin/python; T=/data/articara/trajopt
until [ -f $T/chain20.done ]; do sleep 30; done
X="--impulse 7 --impulse-deg 90 --vx 0.50 --width 0.108 --end-width 0.105 --cross 0.03 --cross-x 0.05 --delay 0.08 --max-iter 2500 --tmin 0.15,0.05,0.25,0.20 --tmax 0.60,0.40,0.60,0.60 --zcom 0.624"
timeout 9000 $P -u scripts/trajopt/push_recovery.py $X --phase 0.3 --swing-x -0.31 --swing-z 0.025 --com-x -0.045 --out $T/v50_7_03.json > $T/v50_7_03.log 2>&1
timeout 9000 $P -u scripts/trajopt/push_recovery.py $X --phase 0.5 --swing-x -0.06 --swing-z 0.056 --com-x 0.0 --out $T/v50_7_05.json > $T/v50_7_05.log 2>&1
echo CHAIN21_DONE > $T/chain21.done
