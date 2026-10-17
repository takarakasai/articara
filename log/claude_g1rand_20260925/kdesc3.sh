set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
L=log/claude_g1rand_20260925; C=$L/kyo_desc1_v2.json; BIN=$L/kyo46rs_walk_tu29
export MODE=stairs START_X=0.40 STAIR_TREAD=0.110 T=28 N=12 W=12 SEED=20260924 DX=0.01 RISERS=-0.010,-0.020,-0.030,-0.040,-0.052
run() { TAG=KDESC3_$1 BASE=$C BIN=$BIN EXTRA="$2" python3 log/claude_g1rand_20260924/rand.py | sed "s/^/$1 /"; }
run r3         '{"SWING_ROT":"3"}'
run o2r3       '{"Z_COM_OFFSET":"-0.02","SWING_ROT":"3"}'
run o2r3abs    '{"Z_COM_OFFSET":"-0.02","SWING_ROT":"3","STAIR_ABSOLUTE_TD":"1"}'
run o2r3lead   '{"Z_COM_OFFSET":"-0.02","SWING_ROT":"3","COM_HEIGHT_DESC_LEAD":"1.0"}'
run o2r3leadabs '{"Z_COM_OFFSET":"-0.02","SWING_ROT":"3","COM_HEIGHT_DESC_LEAD":"1.0","STAIR_ABSOLUTE_TD":"1"}'
run r3lead     '{"SWING_ROT":"3","COM_HEIGHT_DESC_LEAD":"1.0","STAIR_ABSOLUTE_TD":"1"}'
echo "#### KDESC3_DONE"
