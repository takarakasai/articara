#!/bin/bash
# 家族の対応あり比較。使い方: family_cmp.sh BASE CAND（family.sh の NAME）
cd /home/takara/work/dp/articara
L=log/claude_g1rand_20260925
for b in p lat w hold s desc desc_n3 desc_deep asc_n4 c3 spd_30 spd_50 bump10 rough30 slope5 slope-5 sense mis_p5 mis_ad2 comb_slope3 comb_bump10; do
  echo "=== $b"; python3 $L/gpair.py FAM_$1_$b FAM_$2_$b | grep -E 'z=-|総合'
done
