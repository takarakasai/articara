#!/bin/bash
# PUSH_TRIG_LPF_HZ（押しの引き金のずれの低域通過、新設）を kpc4s の上で 3／5／10 Hz（2026-11-01、バイナリ ml4）。
#   押しの小ベンチ（kpb.sh、0.9 N·s・3 方向、種 31415）と、雑音（大）・凹凸 5 mm を 3 つの種で（kfamily.sh と同じ指定）。
#   比べるのは lpf_cmp.py。
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
L=log/claude_g1rand_20260925; B=$L/kyo46rs_walk_ml4; C=$L/kyo_base_v3.json; P=log/claude_kyo46rs_push_20261031
R=log/claude_g1rand_20260924/rand.py
X0=$(cat $P/cand_kpc4s.extra)
flat() { # NAME SEED EXTRA
  env MODE=flat N=48 W=6 SEED=$2 T=28 DX=0.01 FLAT_START_X=30 TAG=KFAM_$1_kbump05 BASE=$C BIN=$B \
    EXTRA="{$3,\"TERRAIN_BUMPS\":\"4,0.005,0.08,0.15,0.60\"}" CONDS='T=28' python3 $R
  env MODE=flat N=24 W=6 SEED=$2 T=28 DX=0.01 FLAT_START_X=30 TAG=KFAM_$1_ksense BASE=$C BIN=$B EXTRA="{$3}" \
    CONDS='SENSE_NOISE_QD=0.1;SENSE_NOISE_V=0.03;SENSE_NOISE_W=0.03' python3 $R
}
for hz in 5 10 3; do
  X="$X0,\"PUSH_TRIG_LPF_HZ\":\"$hz\""; N=lpf$hz
  W=8 bash $P/kpb.sh $N "$X" $B > $P/kpb_$N.out 2>&1 &
  flat ${N}n 31415 "$X" > $P/lpfflat_${N}n.out 2>&1 & flat ${N}o 27182 "$X" > $P/lpfflat_${N}o.out 2>&1 & flat ${N}t 16180 "$X" > $P/lpfflat_${N}t.out 2>&1 & wait
  echo "#### LPF_DONE $hz"
done
echo "#### LPF_ALL_DONE"
