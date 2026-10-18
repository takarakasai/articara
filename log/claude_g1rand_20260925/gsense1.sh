set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
L=log/claude_g1rand_20260925; BIN=$L/kyo46rs_walk_tu48
for v in 20261003v6 20261003v62 20261003v63 20261004v64 20261008v65 20261010v66 20261010v67 20261011v68 20261013v69; do
  C=log/claude_g1champion_$v/controls.json
  MODE=flat N=48 W=8 SEED=20260924 T=25 TAG=GSENSE_${v#*v} BASE=$C BIN=$BIN EXTRA='{}' CONDS="SENSE_NOISE_QD=0.1;SENSE_NOISE_V=0.03;SENSE_NOISE_W=0.03" python3 log/claude_g1rand_20260924/rand.py | sed "s/^/${v#*v} /"
done
echo "#### GSENSE1_DONE"
