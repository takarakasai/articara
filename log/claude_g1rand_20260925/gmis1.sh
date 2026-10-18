set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
L=log/claude_g1rand_20260925; BIN=$L/kyo46rs_walk_tu48; C=log/claude_g1champion_20261022v614/controls.json
F="MODE=flat N=48 W=24 SEED=20260924 T=40 FLAT_MIN_STEPS=24 FLAT_START_X=30"
CONDS="T=40,PLANT_MASS_SCALE=0.9,PLANT_MASS_SCALE=1.1,PLANT_MASS_SCALE=1.2,PLANT_TORSO_MASS_SCALE=1.3,PLANT_PAYLOAD_KG=3;PLANT_PAYLOAD_Z=0.2,PLANT_PAYLOAD_KG=5;PLANT_PAYLOAD_Z=0.2,PLANT_PAYLOAD_KG=3;PLANT_PAYLOAD_X=0.1;PLANT_PAYLOAD_Z=0.2,TORQUE_SCALE=0.85,ACT_DELAY_TICKS=1,ACT_DELAY_TICKS=2,ACT_DELAY_TICKS=4"
env $F TAG=GMIS_flat BASE=$C BIN=$BIN EXTRA='{}' CONDS="$CONDS" python3 log/claude_g1rand_20260924/rand.py | sed 's/^/flat /'
export PUSH_STAIR_X=30
for v in base:'{}' m11:'{"PLANT_MASS_SCALE":"1.1"}' p5:'{"PLANT_PAYLOAD":"5,0,0,0.2"}' ts85:'{"TORQUE_SCALE":"0.85"}' ad2:'{"ACT_DELAY_TICKS":"2"}'; do n=${v%%:*}; x=${v#*:}
  TAG=GMIS_push_$n BASE=$C BIN=$BIN MODE=push N=48 W=24 SEED=20260924 EXTRA="$x" CONDS=11@0,11@180,7@90,5@270 python3 log/claude_g1rand_20260924/rand.py | sed "s/^/push $n /"; done
for n in m11 p5 ts85 ad2; do echo "=== push nominal -> push $n"; python3 $L/gpair.py GMIS_push_base GMIS_push_$n; done
echo "#### GMIS1_DONE"
