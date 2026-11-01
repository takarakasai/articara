#!/bin/bash
# ablate.sh: dm0 の損の 12 個体で、キーを 1 つずつ変える（2026-11-01）。A="名前:KEY=VAL ..." と BIN で渡す
cd /home/takara/work/dp/articara
P=log/claude_kyo46rs_push_20261031/mpcloss; F=log/claude_g1rand_20260924
export BIN=${BIN:-log/claude_g1rand_20260925/kyo46rs_walk_ml5}
M="kbump05/T28/m28:n kbump05/T28/m14:n kbump05/T28/m16:o kbump05/T28/m35:n kbump05/T28/m39:o kbump05/T28/m45:n kbump05/T28/m47:n kdesc/r-0.020/m02:n kdesc/r-0.020/m03:n kdesc/r-0.020/m06:n kdesc/r-0.020/m08:n kdesc/r-0.020/m14:n"
A="${A:?}"
mkdir -p $P/abl
N=0
for a in $A; do an=${a%%:*}; kv=${a#*:}
  for m in $M; do b=${m%%:*}; s=${m#*:}; bn=$(echo $b | tr '/' '_')
    ( d=$P/abl; c=$F/KFAM_dm0${s}_$b/controls.json
      eval "$(python3 -c "import json,shlex;[print('export %s=%s'%(k,shlex.quote(str(v)))) for k,v in json.load(open('$c')).items()]")"; export $kv
      LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib $BIN > $d/${an}_${bn}.log 2>&1 ) &
    N=$((N+1)); [ $((N % 24)) = 0 ] && wait
  done
done
wait
echo "#### ABLATE_DONE $A"
