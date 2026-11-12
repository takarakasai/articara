#!/bin/bash
# jb79scr.sh: 20 cm で縁（足首 −0.2〜+0.06）に着いた本（jb78 の HYa、jb79scr.txt）を j73 で単発に回し直す。jb79scr.sh 名前 VAR=値 ...
D=/home/takara/work/dp/articara/log/claude_g1jump_20261021; S=$D/scr79
list=$D/jb79scr.txt; name=$1; shift; vars="$*"
mkdir -p $S/scr/$name
cat $list | xargs -P 16 -I{} bash -c "cd $D && BIN=$D/g1_flight_probe_j73 ./one.sh $S/scr/$name/\$(echo {} | tr / _).log {} $vars"
n=0; ok=0; for f in $S/scr/$name/*.log; do n=$((n+1)); if ! grep -q FELL $f && grep -q 'on the box' $f; then ok=$((ok+1)); fi; done
echo "$name: stood $ok/$n"
