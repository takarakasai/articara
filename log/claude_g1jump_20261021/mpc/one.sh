#!/bin/bash
# one.sh ENVJSON BIN OUT [K=V ...]: run one member with overrides
J=$1; B=$2; O=$3; shift 3
eval "$(python3 -c "import json,sys,shlex; d=json.load(open('$J')); print(' '.join('export %s=%s'%(k,shlex.quote(v)) for k,v in d.items()))")"
for kv in "$@"; do export "$kv"; done
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
$B > $O 2>&1
