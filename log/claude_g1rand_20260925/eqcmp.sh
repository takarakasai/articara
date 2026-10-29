#!/bin/bash
# eqcmp.sh REF NEW : TRAJ_CSV のバイト一致と、ログ（CSV の場所の行を除く）の一致を数える
R=$1; N=$2; n=0; dc=0; dl=0
for f in $R/*.csv; do b=$(basename $f .csv); n=$((n+1))
  cmp -s $f $N/$b.csv || { dc=$((dc+1)); echo "CSV差 $b"; }
  diff -q <(grep -v 'logging trajectory' $R/$b.log) <(grep -v 'logging trajectory' $N/$b.log) >/dev/null || { dl=$((dl+1)); echo "ログ差 $b"; }
done
echo "計 $n 本: CSV 差 $dc、ログ差 $dl"
