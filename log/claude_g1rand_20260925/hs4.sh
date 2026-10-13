set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
L=log/claude_g1rand_20260925; C=log/claude_g1champion_20261011v68/controls.json; BIN=$L/kyo46rs_walk_hs3
X='{"MOM_CMP":"1","MOM_CMP_SIGN":"-1","MOM_CMP_MAX":"20","MOM_CMP_HMAX":"1","MOM_CMP_KH":"2"}'
XS=',"MOM_CMP":"1","MOM_CMP_SIGN":"-1","MOM_CMP_MAX":"20","MOM_CMP_HMAX":"1","MOM_CMP_KH":"2"'
R4=0.020,0.040,0.060,0.080
TAG=HSY_desc STAIR_N=1 BASE=$C BIN=$BIN MODE=stairs N=48 W=12 SEED=20260924 START_X=0.40 EXTRA="{\"SPAWN_Z_EXTRA\":\"0.005\"$XS}" RISERS=-0.020,-0.040,-0.060,-0.080 python3 log/claude_g1rand_20260924/rand.py
TAG=HSY_desc_n3 STAIR_N=3 BASE=$C BIN=$BIN MODE=stairs N=48 W=12 SEED=20260924 START_X=0.40 EXTRA="{\"SPAWN_Z_EXTRA\":\"0.005\"$XS}" RISERS=-0.040,-0.060,-0.080 python3 log/claude_g1rand_20260924/rand.py
TAG=HSY_desc_deep STAIR_N=1 BASE=$C BIN=$BIN MODE=stairs N=48 W=12 SEED=20260924 START_X=0.40 EXTRA="{\"SPAWN_Z_EXTRA\":\"0.005\"$XS}" RISERS=-0.100,-0.120,-0.150 python3 log/claude_g1rand_20260924/rand.py
TAG=HSY_c3 BASE=$C STAIR_N=3 T=80 BIN=$BIN MODE=stairs N=48 W=12 SEED=20260924 EXTRA="{\"STAIRS_DOWN\":\"3\",\"STAIR_LANDING_LEN\":\"1.5\",\"N_STEPS\":\"48\"$XS}" RISERS=$R4 python3 log/claude_g1rand_20260924/rand.py
TAG=HSY_s BASE=$C BIN=$BIN MODE=stairs N=48 W=12 SEED=20260924 EXTRA="$X" RISERS=$R4 python3 log/claude_g1rand_20260924/rand.py
TAG=HSY_asc_n4 STAIR_N=4 BASE=$C BIN=$BIN MODE=stairs N=48 W=12 SEED=20260924 T=48 EXTRA="{\"STAIR_LANDING_LEN\":\"3.5\",\"N_STEPS\":\"32\"$XS}" RISERS=0.040,0.060,0.080 python3 log/claude_g1rand_20260924/rand.py
{ for p in "desc" "desc_n3" "desc_deep" "c3" "s" "asc_n4"; do echo "=== $p"; python3 $L/mcpair.py DCX_$p HSY_$p; done; echo "#### HS4_DONE"; } > $L/hs4_summary.out 2>&1
