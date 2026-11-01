# TV3 の再現（2026-11-01）: main（misa-locomotion def96db、articara 65d2b2d）で組んだ g1_flight_probe_m68 で、TV3 と同じ設定・種。
cd /home/takara/work/dp/articara
source ./setup-mujoco.sh >/dev/null
D=log/claude_g1jump_20261021; B=$D/g1_flight_probe_m68
X='"JUMP_ABORT":"1","JUMP_PUSH_R":"0.3","QD_CLAMP":"40","DIAG_TD":"1","FLIGHT_FEET_FLAT":"1","DIAG_HX":"1","RELEASE_FIRST_OFF":"1","WBCW_CONTACT_REST":"0.1","WBCW_TRUNK_LAND_VZ":"0.7","WBCW_TRUNK_LAND":"0.03"'
TAG=M68 SEED=31415 BIN=$B CONDS=1.0,1.5,2.0 N=48 W=8 EXTRA="{$X}" python3 $D/jrand.py > /dev/null
echo "=== TV3 -> M68 (31415)"; python3 $D/jrand.py cmp TV3 M68
echo "#### JB21_DONE"
