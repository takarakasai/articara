set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
L=log/claude_g1rand_20260925; BIN=$L/kyo46rs_walk_tu48; C=log/claude_g1champion_20261022v614/controls.json
P4=11@0,11@180,7@90,5@270
export PUSH_STAIR_X=30
run() { TAG=GCOMB_$1 BASE=$C BIN=$BIN MODE=push N=48 W=24 SEED=20260924 EXTRA="$2" CONDS=$P4 python3 log/claude_g1rand_20260924/rand.py | sed "s/^/$1 /"; }
run flat '{}'
run slope3 '{"TERRAIN_SLOPE":"3,1.0,4.0","STEP_DZ_MIN":"0.03","TERRAIN_PITCH":"1"}'
run slope-3 '{"TERRAIN_SLOPE":"-3,1.0,4.0","STEP_DZ_MIN":"0.03","TERRAIN_PITCH":"1","SPAWN_Z_EXTRA":"0.005"}'
run bump10 '{"TERRAIN_BUMPS":"6,0.010,0.25,1.0,4.0"}'
run noise '{"SENSE_NOISE_QD":"0.1","SENSE_NOISE_V":"0.03","SENSE_NOISE_W":"0.03"}'
run delay4 '{"SENSE_DELAY_TICKS":"4"}'
for t in slope3 slope-3 bump10 noise delay4; do echo "=== push on flat -> push on $t"; python3 $L/gpair.py GCOMB_flat GCOMB_$t; done
echo "#### GCOMB1_DONE"
