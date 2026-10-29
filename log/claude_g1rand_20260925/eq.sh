#!/bin/bash
# eq.sh BIN OUT : 家族の見本 76 本＋kyo46rs 20 本（eqlist.txt）を BIN で回し、TRAJ_CSV とログを OUT に置く。DROP=K1,K2 でキーを外す。
# 比較は eqcmp.sh REF NEW（CSV のバイト一致）。実行中にこのファイルを編集しないこと（bash が読み進めて壊れる）。
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
L=/home/takara/work/dp/articara/log/claude_g1rand_20260925
BIN=$(readlink -f $1); OUT=$2; mkdir -p $OUT
one() {
  c=$1; id=$(echo $c | sed 's#log/claude_g1rand_20260924/##; s#/controls.json##; s#/#__#g')
  ( eval "$(python3 -c "import json,shlex,sys;import os;D=set(filter(None,os.environ.get('DROP','').split(',')));[print('export %s=%s'%(k,shlex.quote(str(v)))) for k,v in json.load(open('$c')).items() if k not in D]")"
    export TRAJ_CSV=$OUT/$id.csv; $BIN > $OUT/$id.log 2>&1 )
}
export -f one; export BIN OUT DROP
xargs -P 24 -I{} bash -c 'one {}' < ${LIST:-$L/eqlist.txt}
echo "#### EQ_DONE $OUT"
