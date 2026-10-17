set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
L=log/claude_g1rand_20260925; C=log/claude_g1champion_20261020v612/controls.json; BIN=$L/kyo46rs_walk_tu45
X='"PUSH_REPLAN":"0.03","PUSH_REPLAN_BACK_ONLY":"1","PUSH_REPLAN_VMAX":"0.15"'
C8=0@0,8@0,11@0,3@270,5@270,7@90,11@180,14@180; HC=11@0,5@270,6@270,7@90,11@180,5@90
R4=0.020,0.040,0.060,0.080
TAG=GV614g_s BASE=$C BIN=$BIN MODE=stairs N=48 W=12 SEED=20260924 EXTRA="{$X}" RISERS=$R4 python3 log/claude_g1rand_20260924/rand.py | sed 's/^/s /'
TAG=GV614g_desc STAIR_N=1 BASE=$C BIN=$BIN MODE=stairs N=48 W=12 SEED=20260924 START_X=0.40 EXTRA="{\"SPAWN_Z_EXTRA\":\"0.005\",$X}" RISERS=-0.020,-0.040,-0.060,-0.080 python3 log/claude_g1rand_20260924/rand.py | sed 's/^/desc /'
TAG=GV614g_desc_n3 STAIR_N=3 BASE=$C BIN=$BIN MODE=stairs N=48 W=12 SEED=20260924 START_X=0.40 EXTRA="{\"SPAWN_Z_EXTRA\":\"0.005\",$X}" RISERS=-0.040,-0.060,-0.080 python3 log/claude_g1rand_20260924/rand.py | sed 's/^/desc_n3 /'
TAG=GV614g_desc_deep STAIR_N=1 BASE=$C BIN=$BIN MODE=stairs N=48 W=12 SEED=20260924 START_X=0.40 EXTRA="{\"SPAWN_Z_EXTRA\":\"0.005\",$X}" RISERS=-0.100,-0.120,-0.150 python3 log/claude_g1rand_20260924/rand.py | sed 's/^/desc_deep /'
TAG=GV614g_asc_n4 STAIR_N=4 BASE=$C BIN=$BIN MODE=stairs N=48 W=12 SEED=20260924 T=48 EXTRA="{\"STAIR_LANDING_LEN\":\"3.5\",\"N_STEPS\":\"32\",$X}" RISERS=0.040,0.060,0.080 python3 log/claude_g1rand_20260924/rand.py | sed 's/^/asc_n4 /'
TAG=GV614g_c3 BASE=$C STAIR_N=3 T=80 BIN=$BIN MODE=stairs N=48 W=12 SEED=20260924 EXTRA="{\"STAIRS_DOWN\":\"3\",\"STAIR_LANDING_LEN\":\"1.5\",\"N_STEPS\":\"48\",$X}" RISERS=$R4 python3 log/claude_g1rand_20260924/rand.py | sed 's/^/c3 /'
for b in s desc desc_n3 desc_deep asc_n4 c3; do m=push; case $b in w|hold) m=push;; *) m=stairs;; esac; echo "=== $b"; python3 $L/mcpair.py GCROSS_${b}_e03 GV614g_${b} $m | grep -v '^GV'; done
echo "#### GV614G_DONE"
