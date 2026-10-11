set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
L=log/claude_g1rand_20260925; C=log/claude_g1champion_20261010v67/controls.json; BIN=$L/kyo46rs_walk_dc5
for mg in 0.0 0.005; do
TAG=DCM${mg}_p BASE=$C BIN=$BIN MODE=push N=48 W=12 SEED=20260924 EXTRA="{\"DS_CAPTURE\":\"1\",\"DS_CAPTURE_MARGIN\":\"$mg\"}" CONDS=11@0,7@90,5@90 python3 log/claude_g1rand_20260924/rand.py
done
{ for mg in 0.0 0.005; do echo "=== DS_CAPTURE margin $mg vs v6.7"; python3 $L/mcpair.py RC_p DCM${mg}_p push; python3 $L/mcpair.py RC_lat DCM${mg}_p push 2>/dev/null | grep -E 'i5_d90|対'; done; echo "#### DSC4_DONE"; } > $L/dsc4_summary.out 2>&1
