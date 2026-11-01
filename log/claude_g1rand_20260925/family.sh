#!/bin/bash
# 昇格の家族（2026-10-22 版）。使い方: family.sh NAME CONTROLS.json BIN 'EXTRA_JSON_BODY'
#   EXTRA_JSON_BODY は {} の中身（例: '"PUSH_REPLAN":"0.03"'、空なら ''）。
# 結果は log/claude_g1rand_20260924/FAM_<NAME>_<bench>/。比較は family_cmp.sh BASE CAND。
# 種: FAM_SEED（既定 20260924）と FAM_SEED_HOLD（既定 777）。v6.x は旧い種で採用してきたので
#   チャンピオンはその試行に過剰適合している（2026-10-18）。昇格の判定は新しい種（例 31415／27182）で、
#   基準と候補を同じ種で回して行う。
# 判定（2026-10-19〜）: 悪化 = 入れ替わり 4 本以上で z ≤ −1.5、または別の種でも z ≤ −1 に再現した条件。
#   狙ったベンチを除く家族全体の合計の z も見る。基準は v6.15 の新しい種 FAM_v615n_*。
set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
NAME=$1; C=$2; BIN=$3; X=${4:-}
J() { if [ -n "$X" ]; then echo "{$X${1:+,$1}}"; else echo "{${1:-}}"; fi; }
S1=${FAM_SEED:-20260924}; S2=${FAM_SEED_HOLD:-777}
R=log/claude_g1rand_20260924/rand.py; W=${W:-24}
C8=0@0,8@0,11@0,3@270,5@270,7@90,11@180,14@180; P4=11@0,11@180,7@90,5@270; HC=11@0,5@270,6@270,7@90,11@180,5@90; R4=0.020,0.040,0.060,0.080
t() { echo "FAM_${NAME}_$1"; }
# 1. 押し（公称速度）
TAG=$(t p)    BASE=$C BIN=$BIN MODE=push N=48 W=$W SEED=$S1 EXTRA="$(J)" CONDS=$C8 python3 $R
TAG=$(t lat)  BASE=$C BIN=$BIN MODE=push N=24 W=$W SEED=$S1 EXTRA="$(J)" CONDS=3@270,4@270,5@270,6@270,3@90,5@90,7@90 python3 $R
( export MU_LO=0.40 MU_HI=0.90 JITTER=0.03 DX=0.05; TAG=$(t w) BASE=$C BIN=$BIN MODE=push N=48 W=$W SEED=$S1 EXTRA="$(J)" CONDS=$C8 python3 $R )
TAG=$(t hold) BASE=$C BIN=$BIN MODE=push N=48 W=$W SEED=$S2 EXTRA="$(J)" CONDS=$HC python3 $R
# 2. 階段 6 種
TAG=$(t s) BASE=$C BIN=$BIN MODE=stairs N=48 W=$W SEED=$S1 EXTRA="$(J)" RISERS=$R4 python3 $R
TAG=$(t desc) STAIR_N=1 BASE=$C BIN=$BIN MODE=stairs N=48 W=$W SEED=$S1 START_X=0.40 EXTRA="$(J '"SPAWN_Z_EXTRA":"0.005"')" RISERS=-0.020,-0.040,-0.060,-0.080 python3 $R
TAG=$(t desc_n3) STAIR_N=3 BASE=$C BIN=$BIN MODE=stairs N=48 W=$W SEED=$S1 START_X=0.40 EXTRA="$(J '"SPAWN_Z_EXTRA":"0.005"')" RISERS=-0.040,-0.060,-0.080 python3 $R
TAG=$(t desc_deep) STAIR_N=1 BASE=$C BIN=$BIN MODE=stairs N=48 W=$W SEED=$S1 START_X=0.40 EXTRA="$(J '"SPAWN_Z_EXTRA":"0.005"')" RISERS=-0.100,-0.120,-0.150 python3 $R
TAG=$(t asc_n4) STAIR_N=4 BASE=$C BIN=$BIN MODE=stairs N=48 W=$W SEED=$S1 T=48 EXTRA="$(J '"STAIR_LANDING_LEN":"3.5","N_STEPS":"32"')" RISERS=0.040,0.060,0.080 python3 $R
TAG=$(t c3) BASE=$C STAIR_N=3 T=80 BIN=$BIN MODE=stairs N=48 W=$W SEED=$S1 EXTRA="$(J '"STAIRS_DOWN":"3","STAIR_LANDING_LEN":"1.5","N_STEPS":"48"')" RISERS=$R4 python3 $R
# 3. 速い歩行の押し
for v in 0.30 0.50; do ( export PUSH_STEP_OFFSET=6 PUSH_STAIR_X=30 T=30
  TAG=$(t spd_${v#0.}) BASE=$C BIN=$BIN MODE=push N=48 W=$W SEED=$S1 EXTRA="$(J "\"VX2_SCHED\":\"1\",\"VX2\":\"$v\",\"VX2_AT\":\"8\"")" CONDS=$P4 python3 $R ); done
# 4. 地形（知覚なし・既知）
F="MODE=flat N=48 W=$W SEED=$S1 T=40 FLAT_MIN_STEPS=24 FLAT_START_X=30"
env $F TAG=$(t bump10) BASE=$C BIN=$BIN EXTRA="$(J '"TERRAIN_BUMPS":"6,0.010,0.25,1.0,4.0"')" CONDS="T=40" python3 $R
env $F TAG=$(t rough30) BASE=$C BIN=$BIN EXTRA="$(J '"TERRAIN_BUMPS":"6,0.030,0.25,1.0,4.0","ROUGH_MODE":"1"')" CONDS="T=40" python3 $R
env $F TAG=$(t slope5) BASE=$C BIN=$BIN EXTRA="$(J '"TERRAIN_SLOPE":"5,1.0,4.0","STEP_DZ_MIN":"0.08","STEP_DZ_SWING_MIN":"0.08","TERRAIN_PITCH":"1"')" CONDS="T=40" python3 $R
env $F TAG=$(t slope-5) BASE=$C BIN=$BIN EXTRA="$(J '"TERRAIN_SLOPE":"-5,1.0,4.0","STEP_DZ_MIN":"0.08","STEP_DZ_SWING_MIN":"0.08","TERRAIN_PITCH":"1","SPAWN_Z_EXTRA":"0.005"')" CONDS="T=40" python3 $R
# 5. センサ（遅延 40 ms、弱い雑音）
env MODE=flat N=48 W=$W SEED=$S1 T=25 TAG=$(t sense) BASE=$C BIN=$BIN EXTRA="$(J)" CONDS="SENSE_DELAY_TICKS=8,SENSE_NOISE_QD=0.1;SENSE_NOISE_V=0.03;SENSE_NOISE_W=0.03" python3 $R
# 6. 模型のずれ・乱れの組み合わせ（押し 4 条件）
( export PUSH_STAIR_X=30
  TAG=$(t mis_p5) BASE=$C BIN=$BIN MODE=push N=48 W=$W SEED=$S1 EXTRA="$(J '"PLANT_PAYLOAD":"5,0,0,0.2"')" CONDS=$P4 python3 $R
  TAG=$(t mis_ad2) BASE=$C BIN=$BIN MODE=push N=48 W=$W SEED=$S1 EXTRA="$(J '"ACT_DELAY_TICKS":"2"')" CONDS=$P4 python3 $R
  TAG=$(t comb_slope3) BASE=$C BIN=$BIN MODE=push N=48 W=$W SEED=$S1 EXTRA="$(J '"TERRAIN_SLOPE":"3,1.0,4.0","STEP_DZ_MIN":"0.03","TERRAIN_PITCH":"1"')" CONDS=$P4 python3 $R
  TAG=$(t comb_bump10) BASE=$C BIN=$BIN MODE=push N=48 W=$W SEED=$S1 EXTRA="$(J '"TERRAIN_BUMPS":"6,0.010,0.25,0.8,1.8"')" CONDS=$P4 python3 $R )
echo "#### FAMILY_DONE $NAME"
