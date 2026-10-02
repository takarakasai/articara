set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
L=log/claude_g1rand_20260925; C=log/claude_g1champion_20261003v6/controls.json; BIN=$L/kyo46rs_walk_sense
export FLAT_MIN_STEPS=15
CONDS="SENSE_DELAY_TICKS=2,SENSE_DELAY_TICKS=4,SENSE_DELAY_TICKS=8,SENSE_DELAY_TICKS=12,SENSE_NOISE_QD=0.1;SENSE_NOISE_V=0.03;SENSE_NOISE_W=0.03,SENSE_NOISE_QD=0.3;SENSE_NOISE_V=0.1;SENSE_NOISE_W=0.1,SENSE_DELAY_TICKS=4;SENSE_NOISE_QD=0.1;SENSE_NOISE_V=0.03;SENSE_NOISE_W=0.03"
echo "##### v6 基準（フィルタなし）: センサ"; TAG=B6_sense BASE=$C BIN=$BIN MODE=flat N=48 W=6 SEED=20260924 T=25 EXTRA='{}' CONDS="$CONDS" python3 log/claude_g1rand_20260924/rand.py
echo "#### B6_SENSE_DONE"
