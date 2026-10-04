set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
L=log/claude_g1rand_20260925; BIN=$L/kyo46rs_walk_ff1
[ "$(grep -ac LAND_CONFIRM_XY_RISE $BIN)" -ge 1 ] || { echo "binary lacks knob"; exit 1; }
F='"TRACKING_CUBIC_FF":"1","LAND_CONFIRM_XY_RISE":"0"'
st() { TAG=$1 BASE=$C BIN=$BIN MODE=stairs N=48 W=12 SEED=$2 EXTRA="{$F}" RISERS=$3 python3 log/claude_g1rand_20260924/rand.py; }
desc() { TAG=$1 STAIR_N=$2 BASE=$C BIN=$BIN MODE=stairs N=48 W=12 SEED=20260924 START_X=0.40 EXTRA="{\"SPAWN_Z_EXTRA\":\"0.005\",$F}" RISERS=$3 python3 log/claude_g1rand_20260924/rand.py; }
asc() { TAG=$1 STAIR_N=$2 T=48 BASE=$C BIN=$BIN MODE=stairs N=48 W=12 SEED=20260924 EXTRA="{\"N_STEPS\":\"32\",\"STAIR_LANDING_LEN\":\"3.5\",$F}" RISERS=0.040,0.060,0.080 python3 log/claude_g1rand_20260924/rand.py; }
course() { TAG=$1 BASE=$C STAIR_N=$2 T=$4 BIN=$BIN MODE=stairs N=48 W=12 SEED=20260924 EXTRA="{\"STAIRS_DOWN\":\"$2\",\"STAIR_LANDING_LEN\":\"1.5\",\"N_STEPS\":\"$3\",$F}" RISERS=0.020,0.040,0.060,0.080 python3 log/claude_g1rand_20260924/rand.py; }
for cfg in "GA log/claude_g1champion_20261003v62/controls.json"; do
 set -- $cfg; P=$1; C=$2; echo "########## $P ($C)"
 R=0.020,0.040,0.060,0.080
 echo "## std s"; st ${P}_s 20260924 $R; echo "## std h"; st ${P}_h 777 $R
 echo "## low"; st ${P}_low 20260924 0.000,0.010,0.020,0.030
 ( export MU_LO=0.40 MU_HI=0.90 JITTER=0.03 DX=0.05; echo "## wide s"; st ${P}W_s 20260924 $R; echo "## wide h"; st ${P}W_h 777 $R )
 echo "## desc"; desc ${P}_desc 1 -0.020,-0.040,-0.060,-0.080; desc ${P}_desc_deep 1 -0.100,-0.120,-0.150; desc ${P}_desc_n2 2 -0.040,-0.060,-0.080; desc ${P}_desc_n3 3 -0.040,-0.060,-0.080
 echo "## asc"; asc ${P}_asc_n3 3; asc ${P}_asc_n4 4
 echo "## course"; course ${P}_c1 1 44 70; course ${P}_c2 2 46 75; course ${P}_c3 3 48 80
 { echo "===== $P 対 v6.4"; for p in "WQF_s s" "WQF_h h" "WQF_low low" "WQFW_s W_s" "WQFW_h W_h" "X3Y_desc _desc" "X3Y_desc_deep _desc_deep" "X3Y_desc_n2 _desc_n2" "X3Y_desc_n3 _desc_n3" "AB0.02_asc_n3 _asc_n3" "AB0.02_asc_n4 _asc_n4" "X3Y_c1 _c1" "AB0.02_c2 _c2" "AB0.02_c3 _c3"; do set -- $p; case $2 in s|h|low) t=${P}_$2;; W_s|W_h) t=${P}$2;; *) t=${P}$2;; esac; python3 $L/mcpair.py $1 $t | grep -E '対|総合'; done; } > $L/gg_${P}_summary.out 2>&1
done
echo "#### GG_DONE" >> $L/gg_GA_summary.out
