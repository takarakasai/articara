#!/bin/bash
# kr06.sh OUTDIR [K=V ...]: the kyo46rs running candidate KR06 (11-28; KR05 is skipped, kr05.sh is the battery) = KR04 (kr04.sh)
# + COP_LEASH=0. kyo's walking base v4 carries COP_LEASH 0.005 (the QP's CoP box cut to +-5 mm around the measured CoP once the
# sole tilts past COP_LEASH_TILT 0.02 rad, made for the walking seesaw on bumps). Running, a foot that lands across a board's
# front edge tilts onto that edge, the box follows the measured CoP pinned there (+9..+26 mm), the QP cannot ask for the CoP
# behind it to set the heel down, and every such stance brakes (0.18 -> 0.00 m/s). Off: bumps 0..3 -> 48, kr05.sh 1008 / 1008.
D=$(cd "$(dirname "$0")" && pwd); O=${1:?outdir}; shift
exec $D/kr04.sh $O COP_LEASH=0 "$@"
