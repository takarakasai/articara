set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
L=log/claude_g1rand_20260925; C=log/claude_g1champion_20261020v611/controls.json; BIN=$L/kyo46rs_walk_tu34
X='"MPC_FOOTSTEP_CROSS_Y":"0.20","MPC_FOOTSTEP_CROSS_FLOOR":"0.5","MPC_FOOTSTEP_CROSS_HOLD_Z":"0.05","MPC_FOOTSTEP_CROSS_EX":"0.03"'
C8=0@0,8@0,11@0,3@270,5@270,7@90,11@180,14@180; HC=11@0,5@270,6@270,7@90,11@180,5@90
( export MU_LO=0.40 MU_HI=0.90 JITTER=0.03 DX=0.05; TAG=GCROSS_w_e03 BASE=$C BIN=$BIN MODE=push N=48 W=12 SEED=20260924 EXTRA="{$X}" CONDS=$C8 python3 log/claude_g1rand_20260924/rand.py | sed 's/^/wide /' )
TAG=GCROSS_hold_e03 BASE=$C BIN=$BIN MODE=push N=48 W=12 SEED=777 EXTRA="{$X}" CONDS=$HC python3 log/claude_g1rand_20260924/rand.py | sed 's/^/hold /'
R4=0.020,0.040,0.060,0.080
TAG=GCROSS_s_e03 BASE=$C BIN=$BIN MODE=stairs N=48 W=12 SEED=20260924 EXTRA="{$X}" RISERS=$R4 python3 log/claude_g1rand_20260924/rand.py | sed 's/^/s /'
TAG=GCROSS_desc_e03 STAIR_N=1 BASE=$C BIN=$BIN MODE=stairs N=48 W=12 SEED=20260924 START_X=0.40 EXTRA="{\"SPAWN_Z_EXTRA\":\"0.005\",$X}" RISERS=-0.020,-0.040,-0.060,-0.080 python3 log/claude_g1rand_20260924/rand.py | sed 's/^/desc /'
TAG=GCROSS_desc_n3_e03 STAIR_N=3 BASE=$C BIN=$BIN MODE=stairs N=48 W=12 SEED=20260924 START_X=0.40 EXTRA="{\"SPAWN_Z_EXTRA\":\"0.005\",$X}" RISERS=-0.040,-0.060,-0.080 python3 log/claude_g1rand_20260924/rand.py | sed 's/^/desc_n3 /'
TAG=GCROSS_desc_deep_e03 STAIR_N=1 BASE=$C BIN=$BIN MODE=stairs N=48 W=12 SEED=20260924 START_X=0.40 EXTRA="{\"SPAWN_Z_EXTRA\":\"0.005\",$X}" RISERS=-0.100,-0.120,-0.150 python3 log/claude_g1rand_20260924/rand.py | sed 's/^/desc_deep /'
TAG=GCROSS_asc_n4_e03 STAIR_N=4 BASE=$C BIN=$BIN MODE=stairs N=48 W=12 SEED=20260924 T=48 EXTRA="{\"STAIR_LANDING_LEN\":\"3.5\",\"N_STEPS\":\"32\",$X}" RISERS=0.040,0.060,0.080 python3 log/claude_g1rand_20260924/rand.py | sed 's/^/asc_n4 /'
TAG=GCROSS_c3_e03 BASE=$C STAIR_N=3 T=80 BIN=$BIN MODE=stairs N=48 W=12 SEED=20260924 EXTRA="{\"STAIRS_DOWN\":\"3\",\"STAIR_LANDING_LEN\":\"1.5\",\"N_STEPS\":\"48\",$X}" RISERS=$R4 python3 log/claude_g1rand_20260924/rand.py | sed 's/^/c3 /'
echo "#### GCROSS2_DONE"
