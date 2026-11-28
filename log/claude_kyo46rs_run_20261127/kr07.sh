#!/bin/bash
# kr07.sh OUTDIR [K=V ...]: the kyo46rs running candidate KR07 (11-28) = KR06 (kr06.sh) + COM_BIAS_X=0.006 (the CoM the walker
# reads, 6 mm forward: the backward-only push re-plan cut the step at every steady lag -> marching in place with the payload
# and on bumps). Needs a binary with COM_BIAS_X (tu90, misa-locomotion a270d7b); older ones ignore it silently.
D=$(cd "$(dirname "$0")" && pwd); O=${1:?outdir}; shift
export WALK_BIN=${WALK_BIN:-log/claude_g1rand_20260925/kyo46rs_walk_tu90}
exec $D/kr06.sh $O COM_BIAS_X=0.006 "$@"
