set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
L=log/claude_g1rand_20260925; C=log/claude_g1champion_20261010v67/controls.json; BIN=$L/kyo46rs_walk_dc3
for hp in 1.5 1.2; do
TAG=DC${hp}_p BASE=$C BIN=$BIN MODE=push N=48 W=12 SEED=20260924 EXTRA="{\"DS_CAPTURE\":\"1\",\"DS_CAPTURE_HOPELESS\":\"$hp\"}" CONDS=11@0,11@180,7@90,5@270 python3 log/claude_g1rand_20260924/rand.py
done
{ for hp in 1.5 1.2; do echo "=== DS_CAPTURE hopeless $hp vs v6.7"; python3 $L/mcpair.py RC_p DC${hp}_p push; done; echo "#### DSC1_DONE"; } > $L/dsc1_summary.out 2>&1
