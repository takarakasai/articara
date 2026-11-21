#!/bin/bash
# runwalk.sh CONTROLS.json OUT K=V ... : run the walker (tu68) with the controls plus overrides (TERRAIN_BUMPS dropped)
source /home/takara/work/dp/articara/setup-mujoco.sh >/dev/null 2>&1
C=$1; O=$2; shift 2
cd /home/takara/work/dp/articara
B=${WALK_BIN:-log/claude_g1rand_20260925/kyo46rs_walk_tu68}
env -i PATH=$PATH HOME=$HOME LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib $(python3 -c "
import json,sys;c=json.load(open('$C'));c.pop('TERRAIN_BUMPS',None);print(' '.join(f'{k}={v}' for k,v in c.items()))") "$@" $B > $O 2>&1
echo "$(basename $O): $(grep -o 'verdict: \S*' $O) $(grep -c 'touchdown\|land' $O)"
