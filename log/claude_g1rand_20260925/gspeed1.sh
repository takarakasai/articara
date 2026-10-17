set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
L=log/claude_g1rand_20260925; BIN=$L/kyo46rs_walk_tu38
for arm in v611:log/claude_g1champion_20261020v611/controls.json v613:log/claude_g1champion_20261021v613/controls.json; do n=${arm%%:*}; C=${arm#*:}
  for v in 0.30 0.40 0.50; do
    ( export PUSH_STEP_OFFSET=6 PUSH_STAIR_X=30 T=30
      TAG=GSPD_${n}_${v#0.} BASE=$C BIN=$BIN MODE=push N=48 W=12 SEED=20260924 EXTRA="{\"VX2_SCHED\":\"1\",\"VX2\":\"$v\",\"VX2_AT\":\"8\"}" CONDS=11@0,11@180,7@90,5@270 python3 log/claude_g1rand_20260924/rand.py | sed "s/^/$n $v /" )
  done
done
TAG=GSPD_speed_v613 BASE=log/claude_g1champion_20261021v613/controls.json BIN=$BIN MODE=flat N=48 W=12 SEED=20260924 T=40 FLAT_MIN_STEPS=24 FLAT_START_X=30 EXTRA='{"VX2_SCHED":"1"}' CONDS="VX2=0.20;VX2_AT=8,VX2=0.30;VX2_AT=8,VX2=0.40;VX2_AT=8,VX2=0.50;VX2_AT=8,VX2=0.55;VX2_AT=8" python3 log/claude_g1rand_20260924/rand.py | sed 's/^/speed v613 /'
for v in 30 40 50; do echo "=== $v"; python3 $L/mcpair.py GSPD_v611_$v GSPD_v613_$v push | grep -v '^GSPD'; done
echo "#### GSPD_DONE"
