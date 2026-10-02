# 強条件の基準（判別力を持たせる）
set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
P=$1; C=$2; BIN=${3:-log/claude_g1champion_20260927v52/kyo46rs_walk}; W=${W:-6}
echo "##### $P 速度変更（強）"; TAG=${P}_speed2 BASE=$C BIN=$BIN MODE=flat N=48 W=$W SEED=20260924 T=25 EXTRA='{}' CONDS="VX2=0.45;VX2_AT=8,VX2=0.55;VX2_AT=8,VX2=0.45;VX2_AT=8;VX2_RAMP=0" python3 log/claude_g1rand_20260924/rand.py
echo "##### $P センサ（強）"; TAG=${P}_sense2 BASE=$C BIN=$BIN MODE=flat N=48 W=$W SEED=20260924 T=25 EXTRA='{}' CONDS="SENSE_DELAY_TICKS=8,SENSE_DELAY_TICKS=16,SENSE_NOISE_QD=0.3;SENSE_NOISE_V=0.1;SENSE_NOISE_W=0.1,SENSE_DELAY_TICKS=8;SENSE_NOISE_QD=0.3;SENSE_NOISE_V=0.1;SENSE_NOISE_W=0.1" python3 log/claude_g1rand_20260924/rand.py
echo "#### BENCH_EXT2_DONE_$P"
