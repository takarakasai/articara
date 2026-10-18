set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
L=log/claude_g1rand_20260925; BIN=$L/kyo46rs_walk_tu48; C=log/claude_g1champion_20261022v614/controls.json
NOISE="SENSE_NOISE_QD=0.1;SENSE_NOISE_V=0.03;SENSE_NOISE_W=0.03"
for v in base:'{}' nomom:'{"MOM_CMP":"0"}' noref:'{"REF_TRIG_DCM":"10"}' both:'{"MOM_CMP":"0","REF_TRIG_DCM":"10"}'; do n=${v%%:*}; x=${v#*:}
  MODE=flat N=48 W=8 SEED=20260924 T=25 TAG=GSENSE2_$n BASE=$C BIN=$BIN EXTRA="$x" CONDS="$NOISE" python3 log/claude_g1rand_20260924/rand.py | sed "s/^/$n /"
done
echo "#### GSENSE2_DONE"
