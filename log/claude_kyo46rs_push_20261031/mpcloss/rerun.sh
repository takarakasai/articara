#!/bin/bash
# rerun.sh CONTROLS.json OUTNAME [KEY=VAL ...]: 家族の個体を軌跡付きで再走（ml5、TRAJ_CSV）
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
B=${BIN:-log/claude_g1rand_20260925/kyo46rs_walk_ml5}; O=log/claude_kyo46rs_push_20261031/mpcloss
c=$1; n=$2; shift 2
( eval "$(python3 -c "import json,shlex;[print('export %s=%s'%(k,shlex.quote(str(v)))) for k,v in json.load(open('$c')).items()]")"; [ $# -gt 0 ] && export "$@"; export TRAJ_CSV=$O/$n.csv; $B > $O/$n.log 2>&1 )
