#!/bin/bash
# kyo46rs の昇格の家族（2026-10-31 版）。使い方: kfamily.sh NAME CONTROLS.json BIN 'EXTRA_JSON_BODY'
#   形は family.sh（G1）にそろえる。EXTRA_JSON_BODY は {} の中身（例: '"JOINT_Q_EST":"1"'、空なら ''）。
#   CONTROLS は平地の基準（kyo_base_v2.json／kyo_base_v3.json）。降段は同じ CONTROLS に段を足す。
# 結果は log/claude_g1rand_20260924/KFAM_<NAME>_<bench>/。判定は
#   python3 log/claude_g1rand_20260925/family_judge.py --kyo BASE1:CAND1 BASE2:CAND2 BASE3:CAND3
# 種: FAM_SEED（既定 31415）と FAM_SEED_HOLD（既定 27182）。G1 と同じ 3 組（31415／27182、27182／31415、
#   16180／14142）で回す。種を入れ替えた組では kp（種 S1）と khold（種 S2）の共通の条件が同じ個体になるが、
#   family_judge.py は controls の中身で見分けて重複を除く。
# 規模: 個体 1,552 本（下の表）、W=24 で約 30 分（家族 2 つを同時に回したとき）。
#   kp 13×48・khold 4×48・kw 8×32・kdesc 3×48・kflat 3×24・kbump05／kbump10 各 48・ksense 7×24。
# 打ち切りの罠（memory stair-landing-censoring）: kyo46rs は計画（N_STEPS=40、約 34 s）が尽きると最後の歩で
#   転倒扱いになるので T は計画より短く取る。降段の一歩は 15.5〜18.7 s、T=32 なら段の後に 13 s 以上の平地
#   （約 17 歩）を歩く。KV3_desc_v3 を T=28 と T=32 で回して 24／24／9 が同じ（kpilot2.sh、28→32 s の転倒なし）。
# 回らないもの（kpilot1.sh、基準 v2、N=12）: 既知の斜面 ±2〜3° は 0/12（斜面は段の計画が足場を踏面 0.106 m
#   に切り、立ち位置から 0.154 m の一歩を課す ── 降段 §9.4 の長い一歩と同じ機構で、7 s 前後に転倒）。
#   センサの遅れ 8 tick（40 ms）は 0/12。だからセンサは遅れ 2〜6 tick と雑音で見る。
set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
NAME=$1; C=$2; BIN=$3; X=${4:-}
J() { if [ -n "$X" ]; then echo "{$X${1:+,$1}}"; else echo "{${1:-}}"; fi; }
S1=${FAM_SEED:-31415}; S2=${FAM_SEED_HOLD:-27182}
R=log/claude_g1rand_20260924/rand.py; W=${W:-24}
t() { echo "KFAM_${NAME}_$1"; }
P13=0@0,0.3@0,0.3@90,0.3@180,0.3@270,0.6@0,0.6@90,0.6@180,0.6@270,0.9@0,0.9@90,0.9@180,0.9@270
# 1. 押し（kpush3.sh の 0.3／0.6／0.9 N·s × 前・遊脚側・後ろ・支持足側、押し無し 0@0 を足す。T=20、段は x=30）
( export PUSH_STAIR_X=30
  TAG=$(t kp)    BASE=$C BIN=$BIN MODE=push N=48 W=$W SEED=$S1 EXTRA="$(J)" CONDS=$P13 python3 $R
  # hold 型: 別の種で壁の近くの 4 条件
  TAG=$(t khold) BASE=$C BIN=$BIN MODE=push N=48 W=$W SEED=$S2 EXTRA="$(J)" CONDS=0.6@0,0.9@0,0.9@90,0.9@270 python3 $R
  # 個体の振れ幅を広げる（μ 0.40〜0.90、初期ジッタ 0.03 rad、立ち位置 ±5 cm）
  ( export MU_LO=0.40 MU_HI=0.90 JITTER=0.03 DX=0.05
    TAG=$(t kw) BASE=$C BIN=$BIN MODE=push N=32 W=$W SEED=$S1 EXTRA="$(J)" CONDS=0@0,0.6@0,0.6@90,0.6@180,0.6@270,0.9@0,0.9@90,0.9@270 python3 $R ) )
# 2. 降段 1 段（kv3c.sh の踏面 110 mm・段鼻 x=0.40 ＋ 0.000 は段の計画の一歩だけ）。T=32（上の打ち切りの注）
TAG=$(t kdesc) STAIR_N=1 STAIR_TREAD=0.110 BASE=$C BIN=$BIN MODE=stairs N=48 W=$W SEED=$S1 START_X=0.40 DX=0.01 T=32 \
  EXTRA="$(J '"SPAWN_Z_EXTRA":"0.005"')" RISERS=0.000,-0.010,-0.020 python3 $R
# 3. 平地の歩行（段は x=30、巡航 0.03 と速い 0.06／0.09 m/s）
env MODE=flat N=24 W=$W SEED=$S1 T=28 DX=0.01 FLAT_START_X=30 TAG=$(t kflat) BASE=$C BIN=$BIN EXTRA="$(J)" \
  CONDS='VX=0.03,VX=0.06,VX=0.09' python3 $R
# 4. 小さな段差（計画の地形に無い凹凸 4 個、高さ 0.3〜1 × h、長さ 4〜12 cm、x 0.15〜0.60）
for h in 05 10; do
  env MODE=flat N=48 W=$W SEED=$S1 T=28 DX=0.01 FLAT_START_X=30 TAG=$(t kbump$h) BASE=$C BIN=$BIN \
    EXTRA="$(J "\"TERRAIN_BUMPS\":\"4,0.0$h,0.08,0.15,0.60\"")" CONDS='T=28' python3 $R
done
# 5. センサ（遅れ 10／20／30 ms、速度・角速度の雑音 半分／G1 と同じ、関節角の雑音、指令の遅れ 2 tick）
env MODE=flat N=24 W=$W SEED=$S1 T=28 DX=0.01 FLAT_START_X=30 TAG=$(t ksense) BASE=$C BIN=$BIN EXTRA="$(J)" \
  CONDS='SENSE_DELAY_TICKS=2,SENSE_DELAY_TICKS=4,SENSE_DELAY_TICKS=6,SENSE_NOISE_QD=0.05;SENSE_NOISE_V=0.015;SENSE_NOISE_W=0.015,SENSE_NOISE_QD=0.1;SENSE_NOISE_V=0.03;SENSE_NOISE_W=0.03,SENSE_NOISE_Q=0.002,ACT_DELAY_TICKS=2' python3 $R
echo "#### KFAMILY_DONE $NAME"
