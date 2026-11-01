# (a) 転倒の判定を確かめる（2026-11-01）: TV3 に FALL_TILT=1.5（胴の傾きの判定を緩め、重心の高さ 0.30 m だけで転倒とする）。j21。
#   未設定の j21 は TV3（j19）とビット同一のはず → FT52（FALL_TILT 未設定）で確かめる。
cd /home/takara/work/dp/articara
source ./setup-mujoco.sh >/dev/null
D=log/claude_g1jump_20261021; B=$D/g1_flight_probe_j21
X='"JUMP_ABORT":"1","JUMP_PUSH_R":"0.3","QD_CLAMP":"40","DIAG_TD":"1","FLIGHT_FEET_FLAT":"1","DIAG_HX":"1","RELEASE_FIRST_OFF":"1","WBCW_CONTACT_REST":"0.1","WBCW_TRUNK_LAND_VZ":"0.7","WBCW_TRUNK_LAND":"0.03"'
run() { TAG=$1 SEED=$2 BIN=$B CONDS=$4 N=48 W=8 EXTRA="{$X$3}" python3 $D/jrand.py > /dev/null; }
run FT52 31415 '' 2.0 &
run FT15 31415 ',"FALL_TILT":"1.5"' 1.0,1.5,2.0 &
run FT15s2 27182 ',"FALL_TILT":"1.5"' 1.0,1.5,2.0 &
wait
echo "=== TV3 -> FT52 (31415, 2.0 only, should be identical)"; python3 $D/jrand.py cmp TV3 FT52
echo "=== TV3 -> FT15 (31415)"; python3 $D/jrand.py cmp TV3 FT15
echo "=== TV3s2 -> FT15s2 (27182)"; python3 $D/jrand.py cmp TV3s2 FT15s2
echo "#### JB22_DONE"
