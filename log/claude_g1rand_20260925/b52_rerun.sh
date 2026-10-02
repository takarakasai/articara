set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
L=log/claude_g1rand_20260925; C=log/claude_g1champion_20260927v52/controls.json; BIN=$L/kyo46rs_walk_bench2
for k in SENSE_DELAY_TICKS VX2; do [ "$(grep -ac $k $BIN)" -ge 1 ] || { echo "binary lacks $k"; exit 1; }; done
export FLAT_MIN_STEPS=15
echo "##### B52 速度変更"; TAG=B52_speed BASE=$C BIN=$BIN MODE=flat N=48 W=6 SEED=20260924 T=25 EXTRA='{}' CONDS="VX2=0.25;VX2_AT=8,VX2=0.35;VX2_AT=8,VX2=0.45;VX2_AT=8,VX2=0.0;VX2_AT=8;VX2_RAMP=0" python3 log/claude_g1rand_20260924/rand.py
echo "##### B52 センサ"; TAG=B52_sense BASE=$C BIN=$BIN MODE=flat N=48 W=6 SEED=20260924 T=25 EXTRA='{}' CONDS="SENSE_DELAY_TICKS=2,SENSE_DELAY_TICKS=4,SENSE_DELAY_TICKS=8,SENSE_NOISE_QD=0.1;SENSE_NOISE_V=0.03;SENSE_NOISE_W=0.03,SENSE_NOISE_QD=0.3;SENSE_NOISE_V=0.1;SENSE_NOISE_W=0.1,SENSE_DELAY_TICKS=4;SENSE_NOISE_QD=0.1;SENSE_NOISE_V=0.03;SENSE_NOISE_W=0.03" python3 log/claude_g1rand_20260924/rand.py
echo "#### B52_RERUN_DONE"
