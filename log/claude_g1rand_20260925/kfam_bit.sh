#!/bin/bash
# kfam_bit.sh A B : kfamily.sh の家族 KFAM_A と KFAM_B の run.log の一致（ベンチごと）
G=/home/takara/work/dp/articara/log/claude_g1rand_20260924
for b in kp khold kw kdesc kflat kbump05 kbump10 ksense; do n=0; d=0
  for f in $G/KFAM_$1_$b/*/m*/run.log; do g=${f/KFAM_$1_/KFAM_$2_}; n=$((n+1)); cmp -s $f $g || d=$((d+1)); done
  echo "$b: $n 本中 差 $d"
done
