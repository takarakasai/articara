#!/bin/bash
# kr01.sh OUTDIR [K=V ...]: the kyo46rs running candidate KR01 (11-28): kr.sh with the weighted QP, COM_PREPLAN=0,
# T_SS 0.25, T_FLIGHT 0.06, and the lateral Raibert term off (RUN_FOOT_K=0: it compares the CoM's lateral velocity
# against v_cmd = 0 and so moves every foot outward by the normal sway). Strides 0.032 / 0.043 / 0.054, env as in kr.sh.
D=$(cd "$(dirname "$0")" && pwd); O=${1:?outdir}; shift
exec $D/kr.sh $O WBC_STRATEGY=weighted WBCW_COM=100 WBCW_POSTURE=0.001 WBC_WEIGHTED_OFF_STAIRS=0 COM_PREPLAN=0 \
  T_SS=0.25 T_FLIGHT=0.06 RUN_FOOT_K=0 "$@"
