set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
L=log/claude_g1rand_20260925; BIN=$L/kyo46rs_walk_tu45
arm() { n=$1; C=$2
  F="MODE=flat N=48 W=14 SEED=20260924 T=40 FLAT_MIN_STEPS=24 FLAT_START_X=30"
  for h in 0.010 0.020 0.030; do env $F TAG=GTER_${n}_bump$h BASE=$C BIN=$BIN EXTRA="{\"TERRAIN_BUMPS\":\"6,$h,0.25,1.0,4.0\"}" CONDS="T=40" python3 log/claude_g1rand_20260924/rand.py | sed "s/^/$n bump$h /"; done
  for h in 0.020 0.030; do env $F TAG=GTER_${n}_rough$h BASE=$C BIN=$BIN EXTRA="{\"TERRAIN_BUMPS\":\"6,$h,0.25,1.0,4.0\",\"ROUGH_MODE\":\"1\"}" CONDS="T=40" python3 log/claude_g1rand_20260924/rand.py | sed "s/^/$n rough$h /"; done
  for d in 3 -3; do ex=""; case $d in -*) ex=',"SPAWN_Z_EXTRA":"0.005"';; esac
    env $F TAG=GTER_${n}_slope$d BASE=$C BIN=$BIN EXTRA="{\"TERRAIN_SLOPE\":\"$d,1.0,4.0\",\"STEP_DZ_MIN\":\"0.03\",\"TERRAIN_PITCH\":\"1\"$ex}" CONDS="T=40" python3 log/claude_g1rand_20260924/rand.py | sed "s/^/$n slope$d /"; done
  for d in 5 -5; do ex=""; case $d in -*) ex=',"SPAWN_Z_EXTRA":"0.005"';; esac
    env $F TAG=GTER_${n}_slope$d BASE=$C BIN=$BIN EXTRA="{\"TERRAIN_SLOPE\":\"$d,1.0,4.0\",\"STEP_DZ_MIN\":\"0.08\",\"STEP_DZ_SWING_MIN\":\"0.08\",\"TERRAIN_PITCH\":\"1\"$ex}" CONDS="T=40" python3 log/claude_g1rand_20260924/rand.py | sed "s/^/$n slope$d /"; done
  CONDS="SENSE_DELAY_TICKS=2,SENSE_DELAY_TICKS=4,SENSE_DELAY_TICKS=8,SENSE_DELAY_TICKS=12,SENSE_NOISE_QD=0.1;SENSE_NOISE_V=0.03;SENSE_NOISE_W=0.03,SENSE_NOISE_QD=0.3;SENSE_NOISE_V=0.1;SENSE_NOISE_W=0.1,SENSE_DELAY_TICKS=4;SENSE_NOISE_QD=0.1;SENSE_NOISE_V=0.03;SENSE_NOISE_W=0.03"
  env MODE=flat N=48 W=14 SEED=20260924 T=25 TAG=GTER_${n}_sense BASE=$C BIN=$BIN EXTRA='{}' CONDS="$CONDS" python3 log/claude_g1rand_20260924/rand.py | sed "s/^/$n sense /"
}
arm v610 log/claude_g1champion_20261016v610/controls.json > $L/gterr1_v610.out 2>&1 &
arm v614 log/claude_g1champion_20261022v614/controls.json > $L/gterr1_v614.out 2>&1 &
wait
for t in bump0.010 bump0.020 bump0.030 rough0.020 rough0.030 slope3 slope-3 slope5 slope-5 sense; do echo "=== $t"; python3 $L/gpair.py GTER_v610_$t GTER_v614_$t; done
echo "#### GTER1_DONE"
