set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
L=log/claude_g1rand_20260925; C=$L/kyo_desc1_v2.json; BIN=$L/kyo46rs_walk_tu29
export MODE=stairs START_X=0.40 STAIR_TREAD=0.110 T=28 N=12 W=12 SEED=20260924 DX=0.01 RISERS=0.000,-0.020,-0.030,-0.040,-0.052
REC='"Z_COM_OFFSET":"-0.04","SWING_ROT":"3","STAIR_ABSOLUTE_TD":"1","STAIR_RISE_SOLE_POSE":"1"'
TAG=KDESC1_base BASE=$C BIN=$BIN EXTRA='{}' python3 log/claude_g1rand_20260924/rand.py
TAG=KDESC1_rec BASE=$C BIN=$BIN EXTRA="{$REC}" python3 log/claude_g1rand_20260924/rand.py
TAG=KDESC1_recb BASE=$C BIN=$BIN EXTRA="{$REC,\"STAIR_FULL_SOLE\":\"0\",\"STAIR_TD_BIAS\":\"-0.015\"}" python3 log/claude_g1rand_20260924/rand.py
echo "#### KDESC1_DONE"
