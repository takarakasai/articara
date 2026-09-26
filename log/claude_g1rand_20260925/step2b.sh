set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
BIN=target/release/examples/kyo46rs_walk; V1=log/claude_g1bdbaseline_20260926/controls.json
run () { echo "##### $1"; TAG=$2 BASE=$V1 BIN=$BIN MODE=stairs N=48 W=12 SEED=20260924 EXTRA="$3" RISERS=0.020,0.040,0.060,0.080 python3 log/claude_g1rand_20260924/rand.py; }
run "v1 + DCM_W=5e4（対照: 参照は計画）"      X2_dcm     '{"MPC_DCM_W":"50000"}'
run "v1 + DCM_W=5e4 + XI_REF lead=0"       X2_xi0     '{"MPC_DCM_W":"50000","MPC_XI_REF":"1"}'
run "v1 + DCM_W=2e5 + XI_REF lead=0.10"    X2_xi10    '{"MPC_DCM_W":"200000","MPC_XI_REF":"1","MPC_XI_REF_LEAD":"0.10"}'
echo "#### STEP2B_DONE"
