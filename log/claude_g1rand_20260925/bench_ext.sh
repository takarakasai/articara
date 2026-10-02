# 新 4 系統の基準値（v5.2、N=48）: 降段 / 速度変更 / センサ遅延・雑音。斜面は未対応。
# 使い方: bash bench_ext.sh <PREFIX> <controls.json> [BIN]
set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
P=$1; C=$2;
# ノブがバイナリに無ければ全部 48/48 の無意味な基準になる（2026-10-02 に発生）
for k in SENSE_DELAY_TICKS VX2 SPAWN_Z_EXTRA; do [ "$(grep -ac $k ${3:-log/claude_g1rand_20260925/kyo46rs_walk_bench2})" -ge 1 ] || { echo "binary lacks $k"; exit 1; }; done
export FLAT_MIN_STEPS=${FLAT_MIN_STEPS:-15} BIN=${3:-log/claude_g1rand_20260925/kyo46rs_walk_bench2}; W=${W:-13}
echo "##### $P 降段（start 0.40、1 段）"; TAG=${P}_desc BASE=$C BIN=$BIN MODE=stairs N=48 W=$W SEED=20260924 START_X=0.40 EXTRA='{"SPAWN_Z_EXTRA":"0.005"}' RISERS=-0.020,-0.040,-0.060 python3 log/claude_g1rand_20260924/rand.py
echo "##### $P 速度変更（歩 8 から）"; TAG=${P}_speed BASE=$C BIN=$BIN MODE=flat N=48 W=$W SEED=20260924 T=25 EXTRA='{}' CONDS="VX2=0.25;VX2_AT=8,VX2=0.35;VX2_AT=8,VX2=0.0;VX2_AT=8,VX2=0.0;VX2_AT=8;VX2_RAMP=0" python3 log/claude_g1rand_20260924/rand.py
echo "##### $P センサ遅延・雑音"; TAG=${P}_sense BASE=$C BIN=$BIN MODE=flat N=48 W=$W SEED=20260924 T=25 EXTRA='{}' CONDS="SENSE_DELAY_TICKS=2,SENSE_DELAY_TICKS=4,SENSE_NOISE_QD=0.1;SENSE_NOISE_V=0.03;SENSE_NOISE_W=0.03,SENSE_DELAY_TICKS=2;SENSE_NOISE_QD=0.1;SENSE_NOISE_V=0.03;SENSE_NOISE_W=0.03" python3 log/claude_g1rand_20260924/rand.py
echo "#### BENCH_EXT_DONE_$P"
