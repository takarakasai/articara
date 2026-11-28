#!/bin/bash
# kr04.sh OUTDIR [K=V ...]: the kyo46rs running candidate KR04 (11-28) = KR03 (kr03.sh) + the base-velocity fusion the G1
# champion has carried in its controls since v6.15 (BASE_VEL_FUSE_HZ=3 BASE_VEL_FUSE_BIAS_T=2, with the IMU model
# SENSE_NOISE_A=0.02 SENSE_ACC_BIAS=0.05,-0.05,0.05). kyo's walking base v4 has none, so the running KR03 read the raw body
# velocity and its noise row (24 of 48) was not the G1 one (48 with fusion). Fusion: noise 24 -> 48 on both seed sets.
D=$(cd "$(dirname "$0")" && pwd); O=${1:?outdir}; shift
exec $D/kr03.sh $O BASE_VEL_FUSE_HZ=3 BASE_VEL_FUSE_BIAS_T=2 SENSE_NOISE_A=0.02 SENSE_ACC_BIAS=0.05,-0.05,0.05 "$@"
