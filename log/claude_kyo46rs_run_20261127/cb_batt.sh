#!/bin/bash
# cb_batt.sh: the kr05.sh battery for COM_BIAS_X=0.006 and COM_BIAS_DCM_T=1 on KR06 (tu89), sets a and b.
cd /home/takara/work/dp/articara; L=log/claude_kyo46rs_run_20261127
export WALK_BIN=log/claude_g1rand_20260925/kyo46rs_walk_tu89
RUN=$L/kr06.sh EXTRA="COM_BIAS_X=0.006" $L/kr05.sh $L/kr05cx_a > $L/kr05cx_a.txt
SEED0=16 RUN=$L/kr06.sh EXTRA="COM_BIAS_X=0.006" $L/kr05.sh $L/kr05cx_b > $L/kr05cx_b.txt
RUN=$L/kr06.sh EXTRA="COM_BIAS_DCM_T=1" $L/kr05.sh $L/kr05ci_a > $L/kr05ci_a.txt
SEED0=16 RUN=$L/kr06.sh EXTRA="COM_BIAS_DCM_T=1" $L/kr05.sh $L/kr05ci_b > $L/kr05ci_b.txt
echo done > $L/cb_batt.done
