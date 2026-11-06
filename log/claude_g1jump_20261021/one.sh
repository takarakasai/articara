#!/bin/bash
# 1 本を記録の env で回す: one.sh 出力ログ runs/のディレクトリ [追加の VAR=値 ...]（BIN で実行ファイル、既定 j41）
D=/home/takara/work/dp/articara/log/claude_g1jump_20261021
out=$1; src=$D/runs/$2; shift 2
env $(python3 -c "import json,sys; print(' '.join(f'{k}={v}' for k,v in json.load(open(sys.argv[1])).items()))" $src/env.json) "$@" ${BIN:-$D/g1_flight_probe_j41} > $out 2>&1
