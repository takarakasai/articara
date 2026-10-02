# 強条件の基準（判別力を持たせる）
set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
P=$1; C=$2;
# ノブがバイナリに無ければ全部 48/48 の無意味な基準になる（2026-10-02 に発生）
for k in SENSE_DELAY_TICKS VX2 SPAWN_Z_EXTRA; do [ "$(grep -ac $k ${3:-log/claude_g1rand_20260925/kyo46rs_walk_bench2})" -ge 1 ] || { echo "binary lacks $k"; exit 1; }; done
export FLAT_MIN_STEPS=${FLAT_MIN_STEPS:-15} BIN=${3:-log/claude_g1rand_20260925/kyo46rs_walk_bench2}; W=${W:-6}
echo "##### $P 速度変更（強）"; TAG=${P}_speed2 BASE=$C BIN=$BIN MODE=flat N=48 W=$W SEED=20260924 T=25 EXTRA='{}' CONDS="VX2=0.45;VX2_AT=8,VX2=0.55;VX2_AT=8,VX2=0.45;VX2_AT=8;VX2_RAMP=0" python3 log/claude_g1rand_20260924/rand.py
echo "##### $P センサ（強）"; TAG=${P}_sense2 BASE=$C BIN=$BIN MODE=flat N=48 W=$W SEED=20260924 T=25 EXTRA='{}' CONDS="SENSE_DELAY_TICKS=8,SENSE_DELAY_TICKS=16,SENSE_NOISE_QD=0.3;SENSE_NOISE_V=0.1;SENSE_NOISE_W=0.1,SENSE_DELAY_TICKS=8;SENSE_NOISE_QD=0.3;SENSE_NOISE_V=0.1;SENSE_NOISE_W=0.1" python3 log/claude_g1rand_20260924/rand.py
echo "#### BENCH_EXT2_DONE_$P"
