set -u
# usage: restep1.sh <tag> <landed-ref.json> [extra-json-fields]
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
L=log/claude_g1rand_20260925; C=log/claude_g1champion_20261011v68/controls.json; BIN=$L/kyo46rs_walk_rs7; D=log/claude_g1champion_20261011v68
TAGX=$1; REF=$2; EXTRA_FIELDS=${3:-}
LIB="$D/ref_walk5_phase03.json,$D/ref_walk5_phase05.json,$D/ref_walk7_phase03.json,$REF"
X="{\"RESTEP\":\"1\",\"REF_LIBRARY\":\"$LIB\"$EXTRA_FIELDS}"
TAG=${TAGX}_p BASE=$C BIN=$BIN MODE=push N=48 W=12 SEED=20260924 EXTRA="$X" CONDS=7@90,5@90,6@270,5@270 python3 log/claude_g1rand_20260924/rand.py
{ echo "=== $TAGX ($REF) vs v6.8"; python3 $L/mcpair.py DCX_p ${TAGX}_p push; python3 $L/mcpair.py DCX_lat ${TAGX}_p push 2>/dev/null | grep -E 'i5_d90|i6_d270'; echo "#### ${TAGX}_DONE"; } > $L/${TAGX}_summary.out 2>&1
