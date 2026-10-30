#!/bin/bash
# eqpresetcmp.sh NEW REF... : eqpreset.sh's runs against the full-controls references (TRAJ_CSV byte equality,
# and the log without the CSV's path line), each sample against the REF directory that holds it.
N=$1; shift; n=0; dc=0; dl=0; miss=0
for f in $N/*.csv; do b=$(basename $f .csv); r=""
  for R in "$@"; do [ -f $R/$b.csv ] && { r=$R; break; }; done
  [ -z "$r" ] && { miss=$((miss+1)); echo "基準なし $b"; continue; }
  n=$((n+1))
  cmp -s $r/$b.csv $f || { dc=$((dc+1)); echo "CSV差 $b"; }
  diff -q <(grep -v 'logging trajectory' $r/$b.log) <(grep -v 'logging trajectory' $N/$b.log) >/dev/null || { dl=$((dl+1)); echo "ログ差 $b"; }
done
echo "計 $n 本: CSV 差 $dc、ログ差 $dl（基準なし $miss、対象外 $(wc -l < $N/excluded.txt)）"
