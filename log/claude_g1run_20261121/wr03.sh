#!/bin/bash
# wr03.sh OUTDIR [BIN]: the 11-21 §63 running settings (wr01.sh + T_FLIGHT 0.04 + RUN_Z_LEVEL 1,
# misa-locomotion g1-run d6d7ee4 = kyo46rs_walk_tu71). Env SEEDS / STRIDES as in wr02.sh.
D=$(cd "$(dirname "$0")" && pwd)
exec $D/wr02.sh ${1:?outdir} ${2:-log/claude_g1rand_20260925/kyo46rs_walk_tu71} T_FLIGHT=0.04 RUN_Z_LEVEL=1
