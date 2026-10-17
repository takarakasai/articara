set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
L=log/claude_g1rand_20260925; C=$L/kyo_desc1_v2.json; BIN=$L/kyo46rs_walk_tu29
export MODE=stairs START_X=0.40 STAIR_TREAD=0.110 T=28 N=12 W=12 SEED=20260924 DX=0.01 RISERS=0.000,-0.010,-0.020,-0.030,-0.040,-0.052
run() { TAG=KDESC5_$1 BASE=$C BIN=$BIN EXTRA="{\"STAIR_SWING_FF\":\"0\",\"STAIR_ABSOLUTE_TD\":\"1\",\"SWING_ROT\":\"3\"$2}" python3 log/claude_g1rand_20260924/rand.py | sed "s/^/$1 /"; }
run r3        ''
run o2        ',"Z_COM_OFFSET":"-0.02"'
run o2lead    ',"Z_COM_OFFSET":"-0.02","COM_HEIGHT_DESC_LEAD":"1.0"'
run lead      ',"COM_HEIGHT_DESC_LEAD":"1.0"'
run lead05    ',"COM_HEIGHT_DESC_LEAD":"0.5"'
run o2reach   ',"Z_COM_OFFSET":"-0.02","COM_HEIGHT_DESC_REACH":"0.90"'
echo "#### KDESC5_DONE"
