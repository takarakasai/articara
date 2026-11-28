#!/bin/bash
# kr03.sh OUTDIR [K=V ...]: the kyo46rs running candidate KR03 (11-28) = KR01 (kr01.sh) + RUN_FOOT_XI_GY=1.0 (the touchdown follows
# the DCM's LATERAL error against the plan: no response to the normal sway, unlike RUN_FOOT_K; crossing pushes 1..5 -> 48 of 48)
# + PUSH_REPLAN=0.03 (kyo's own walking trigger; kr.sh's 0.045 = G1's running 0.10 scaled by the leg is the worst of 0..0.08).
# Robustness: RUN=.../kr03.sh kr05.sh OUT. Strides and env as in kr.sh.
D=$(cd "$(dirname "$0")" && pwd); O=${1:?outdir}; shift
exec $D/kr01.sh $O RUN_FOOT_XI_GY=1.0 PUSH_REPLAN=0.03 "$@"
