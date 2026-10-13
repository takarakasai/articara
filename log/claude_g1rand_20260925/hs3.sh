set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
L=log/claude_g1rand_20260925; C=log/claude_g1champion_20261011v68/controls.json; BIN=$L/kyo46rs_walk_hs2
R1='ran'; R2='d.py'; while pgrep -f "${R1}${R2}" > /dev/null; do sleep 30; done
X='{"MOM_CMP":"1","MOM_CMP_SIGN":"-1","MOM_CMP_MAX":"20","MOM_CMP_HMAX":"1","MOM_CMP_KH":"2"}'; C8=0@0,8@0,11@0,3@270,5@270,7@90,11@180,14@180; HC=11@0,5@270,6@270,7@90,11@180,5@90
XS=',"MOM_CMP":"1","MOM_CMP_SIGN":"-1","MOM_CMP_MAX":"20","MOM_CMP_HMAX":"1","MOM_CMP_KH":"2"'
TAG=HSX_lat BASE=$C BIN=$BIN MODE=push N=24 W=12 SEED=20260924 EXTRA="$X" CONDS=3@270,4@270,5@270,6@270,3@90,5@90,7@90 python3 log/claude_g1rand_20260924/rand.py
( export MU_LO=0.40 MU_HI=0.90 JITTER=0.03 DX=0.05; TAG=HSXW_p BASE=$C BIN=$BIN MODE=push N=48 W=12 SEED=20260924 EXTRA="$X" CONDS=$C8 python3 log/claude_g1rand_20260924/rand.py )
TAG=HSX_hold_p BASE=$C BIN=$BIN MODE=push N=48 W=12 SEED=777 EXTRA="$X" CONDS=$HC python3 log/claude_g1rand_20260924/rand.py
R4=0.020,0.040,0.060,0.080
TAG=HSX_s BASE=$C BIN=$BIN MODE=stairs N=48 W=12 SEED=20260924 EXTRA="$X" RISERS=$R4 python3 log/claude_g1rand_20260924/rand.py
TAG=HSX_desc STAIR_N=1 BASE=$C BIN=$BIN MODE=stairs N=48 W=12 SEED=20260924 START_X=0.40 EXTRA="{\"SPAWN_Z_EXTRA\":\"0.005\"$XS}" RISERS=-0.020,-0.040,-0.060,-0.080 python3 log/claude_g1rand_20260924/rand.py
TAG=HSX_desc_n3 STAIR_N=3 BASE=$C BIN=$BIN MODE=stairs N=48 W=12 SEED=20260924 START_X=0.40 EXTRA="{\"SPAWN_Z_EXTRA\":\"0.005\"$XS}" RISERS=-0.040,-0.060,-0.080 python3 log/claude_g1rand_20260924/rand.py
TAG=HSX_desc_deep STAIR_N=1 BASE=$C BIN=$BIN MODE=stairs N=48 W=12 SEED=20260924 START_X=0.40 EXTRA="{\"SPAWN_Z_EXTRA\":\"0.005\"$XS}" RISERS=-0.100,-0.120,-0.150 python3 log/claude_g1rand_20260924/rand.py
TAG=HSX_asc_n4 STAIR_N=4 BASE=$C BIN=$BIN MODE=stairs N=48 W=12 SEED=20260924 T=48 EXTRA="{\"STAIR_LANDING_LEN\":\"3.5\",\"N_STEPS\":\"32\"$XS}" RISERS=0.040,0.060,0.080 python3 log/claude_g1rand_20260924/rand.py
TAG=HSX_c3 BASE=$C STAIR_N=3 T=80 BIN=$BIN MODE=stairs N=48 W=12 SEED=20260924 EXTRA="{\"STAIRS_DOWN\":\"3\",\"STAIR_LANDING_LEN\":\"1.5\",\"N_STEPS\":\"48\"$XS}" RISERS=$R4 python3 log/claude_g1rand_20260924/rand.py
{ echo "=== std push"; python3 $L/mcpair.py DCX_p HSX_p push; echo "=== lat"; python3 $L/mcpair.py DCX_lat HSX_lat push; echo "=== wide push"; python3 $L/mcpair.py DCXW_p HSXW_p push; echo "=== holdout 777"; python3 $L/mcpair.py DCX_hold_p HSX_hold_p push
  echo "=== stairs"; python3 $L/mcpair.py DCX_s HSX_s; echo "=== desc"; python3 $L/mcpair.py DCX_desc HSX_desc; echo "=== desc n3"; python3 $L/mcpair.py DCX_desc_n3 HSX_desc_n3; echo "=== desc deep"; python3 $L/mcpair.py DCX_desc_deep HSX_desc_deep; echo "=== asc n4"; python3 $L/mcpair.py DCX_asc_n4 HSX_asc_n4; echo "=== course 3"; python3 $L/mcpair.py DCX_c3 HSX_c3; echo "#### HS3_DONE"; } > $L/hs3_summary.out 2>&1
