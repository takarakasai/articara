#!/usr/bin/env bash
# 標準バッテリ: 段（SEED=20260924）・ホールドアウト（777）・押し 8 条件・横段階化を 1 コマンドで。
#   scripts/g1_battery.sh <PREFIX> <controls.json> '<EXTRA json>' [std|wide]
# 出力: log/claude_g1rand_20260924/<PREFIX>_{s,h,p,lat}/。wide は封筒を広げる
# （摩擦 0.40–0.90、初期ジッタ 0.03 rad、段位置 ±5 cm）。基準はタグ suffix で対応あり比較する。
set -u
cd "$(dirname "$0")/.."
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
P=$1; BASE=$2; EXTRA=${3:-'{}'}; REG=${4:-std}
BIN=${BIN:-target/release/examples/kyo46rs_walk}; W=${W:-13}; N=${N:-48}
if [ "$REG" = wide ]; then export MU_LO=0.40 MU_HI=0.90 JITTER=0.03 DX=0.05; P="${P}W"; fi
R=0.020,0.040,0.060,0.080
C8=0@0,8@0,11@0,3@270,5@270,7@90,11@180,14@180
LAT=3@270,4@270,5@270,6@270,3@90,5@90,7@90
run () { echo "##### $P $1"; TAG="${P}_$1" BASE=$BASE BIN=$BIN MODE=$2 N=$3 W=$W SEED=$4 EXTRA="$EXTRA" RISERS=$R CONDS=$5 \
  python3 log/claude_g1rand_20260924/rand.py; }
run s   stairs $N 20260924 ""
run h   stairs $N 777      ""
run p   push   $N 20260924 $C8
run lat push   24 20260924 $LAT
echo "#### BATTERY_DONE_$P"
