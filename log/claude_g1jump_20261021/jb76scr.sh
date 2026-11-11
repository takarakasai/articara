#!/bin/bash
# jb76scr.sh: 20 cm で縁 +0.06 以上に載って倒れた本（jb75 の T04、jb76scr.txt の 57 本）を base_ht で単発に回し直す。jb76scr.sh 名前 VAR=値 ...
D=/home/takara/work/dp/articara/log/claude_g1jump_20261021; S=$D/scr76
list=$D/jb76scr.txt; name=$1; shift; vars="$*"
mkdir -p $S/scr/$name
cat $list | xargs -P 16 -I{} bash -c "cd $D && BIN=$D/g1_flight_probe_base_ht ./one.sh $S/scr/$name/\$(echo {} | tr / _).log {} $vars"
n=0; ok=0; for f in $S/scr/$name/*.log; do n=$((n+1)); if ! grep -q FELL $f && grep -q 'on the box' $f; then ok=$((ok+1)); fi; done
echo "$name: stood $ok/$n"
