set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
L=log/claude_g1rand_20260925; C=log/claude_g1champion_20261004v64/controls.json; BIN=$L/kyo46rs_walk_vz2
[ "$(grep -ac VX2_Z_DROP $BIN)" -ge 1 ] || { echo "binary lacks knob"; exit 1; }
for z in 0.024 0.035; do echo "##### VX2_Z_DROP $z"; TAG=VY${z}_speed BASE=$C BIN=$BIN MODE=flat N=48 W=12 SEED=20260924 T=40 FLAT_MIN_STEPS=24 FLAT_START_X=30 EXTRA="{\"VX2_STRIDE_MAX\":\"0.25\",\"VX2_Z_DROP\":\"$z\"}" CONDS="VX2=0.20;VX2_AT=8,VX2=0.30;VX2_AT=8,VX2=0.35;VX2_AT=8,VX2=0.40;VX2_AT=8,VX2=0.45;VX2_AT=8,VX2=0.50;VX2_AT=8,VX2=0.0;VX2_AT=8" python3 log/claude_g1rand_20260924/rand.py; done
echo "#### SPEED_VY_DONE"
