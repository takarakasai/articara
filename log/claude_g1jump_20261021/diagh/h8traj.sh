#!/bin/bash
# H8 の 2 本（2.0 m00 着地、1.5 m01 転倒）を env.json のまま流し直して軌跡を取る（2026-11-02）。
cd /home/takara/work/dp/articara
source ./setup-mujoco.sh >/dev/null
D=log/claude_g1jump_20261021; O=$D/diagh
one() { env $(python3 -c "import json,sys; print(' '.join(f'{k}={v}' for k,v in json.load(open(sys.argv[1])).items()))" $D/runs/H8/$1/$2/env.json) TRAJ_CSV=$O/H8_$1_$2.csv $D/g1_flight_probe_j25 > $O/H8_$1_$2.log 2>&1; }
one v2.0 m00 & one v1.5 m01 & wait
for x in v2.0/m00 v1.5/m01; do n=${x/\//_}; diff <(grep -v "^\s*$" $D/runs/H8/$x/run.log) <(grep -v "^\s*$" $O/H8_$n.log) > /dev/null && echo "$n same" || echo "$n DIFF"; done
echo "#### H8TRAJ_DONE"
