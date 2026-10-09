set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
L=log/claude_g1rand_20260925; C=log/claude_g1champion_20261010v66/controls.json; BIN=$L/kyo46rs_walk_fw1
for v in "CD3 {\"REF_COOLDOWN\":\"3.0\"}" "CD3F85 {\"REF_COOLDOWN\":\"3.0\",\"REF_TRIG_MAX_FRAC\":\"0.85\"}" "F85 {\"REF_TRIG_MAX_FRAC\":\"0.85\"}"; do set -- $v; TAG=${1}_p BASE=$C BIN=$BIN MODE=push N=48 W=12 SEED=20260924 EXTRA="$2" CONDS=7@90,5@90,5@270 python3 log/claude_g1rand_20260924/rand.py; done
{ for v in CD3 CD3F85 F85; do echo "=== $v vs v6.6"; python3 $L/mcpair.py ZE0.035_p ${v}_p push; done; echo "#### COOL_DONE"; } > $L/cool_summary.out 2>&1
