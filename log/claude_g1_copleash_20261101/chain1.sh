#!/bin/bash
# 選別 1（2026-11-01）: 縛り 5 mm／10 mm（門 0.02 rad、MPC の箱も）を地形・斜面 5 ベンチ、凹凸 2 ベンチを振り直しの地形で（基準も）。
cd /home/takara/work/dp/articara
D=log/claude_g1_copleash_20261101; export W=16
L5='"COP_LEASH":"0.005","COP_LEASH_TILT":"0.02","COP_LEASH_MPC":"1"'
L10='"COP_LEASH":"0.010","COP_LEASH_TILT":"0.02","COP_LEASH_MPC":"1"'
bash $D/g1leash.sh gL5 "$L5"
bash $D/g1leash.sh v621L '' 1
bash $D/g1leash.sh gL5 "$L5" 1
bash $D/g1leash.sh gL10 "$L10"
bash $D/g1leash.sh gL10 "$L10" 1
echo "#### CHAIN1_DONE"
