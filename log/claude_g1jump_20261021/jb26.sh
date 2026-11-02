#!/bin/bash
# h_y ≈ 0 の踏み切り H0（タグは HZ0: runs/H0 は昼の別の実験と重なる）（接触 100・胴 0.003・角運動量 10）＋足を止まり点へ＋JUMP_VX（2026-11-02）。j25、種 31415。jb25 と対で比べる。
cd /home/takara/work/dp/articara
source ./setup-mujoco.sh >/dev/null
D=log/claude_g1jump_20261021; B=$D/g1_flight_probe_j25
X='"JUMP_ABORT":"1","JUMP_PUSH_R":"0.3","QD_CLAMP":"40","DIAG_TD":"1","FLIGHT_FEET_FLAT":"1","DIAG_HX":"1","RELEASE_FIRST_OFF":"1","WBCW_CONTACT_REST":"0.1","WBCW_TRUNK_LAND_VZ":"0.7","WBCW_TRUNK_LAND":"0.03","FALL_TILT":"1.5","LAND_REF_FOLLOW":"2","AIR_ABORT_T":"0.08"'
C=',"FLIGHT_FEET_CAPTURE":"1","FLIGHT_FEET_CAPTURE_T":"0.06","KP_FLIGHT_FEET":"1000","KD_FLIGHT_FEET":"63","WBCW_TRUNK_PUSH":"0.003","WBCW_MOMENTUM":"10","WBCW_CONTACT":"100"'
run() { TAG=$1 SEED=31415 BIN=$B CONDS=1.5,2.0 N=48 W=4 EXTRA="{$X$2}" python3 $D/jrand.py; }
run HZ0 "$C" &
: run H4 ',"JUMP_VX":"0.4"'"$C" &
: run H8 ',"JUMP_VX":"0.8"'"$C" &
wait
python3 - <<'PY'
import re,glob,statistics as st
D='log/claude_g1jump_20261021/runs'
for tag in ['HZ0','H4','H8']:
  for c in ['1.5','2.0']:
    L=[];vx=[];dx=[];hy=[]
    for f in sorted(glob.glob(f'{D}/{tag}/v{c}/m*/run.log')):
      t=open(f).read(); v=re.search(r'verdict: (.*)',t).group(1); L.append(v)
      m=re.search(r'forward: lift-off vx (\S+), touchdown CoM dx (\S+), final CoM dx (\S+)',t)
      lo=re.search(r'\[lift-off\][^\n]*h_ang \(([^,]+),([^,]+),',t)
      if m and m.group(1)!='NaN': vx.append(float(m.group(1)))
      if m and v=='LANDED': dx.append(float(m.group(3)))
      if lo: hy.append(float(lo.group(2)))
    med=lambda a: st.median(a) if a else float('nan')
    print(f"{tag:5s} V={c}: 着地 {L.count('LANDED')}/48 転倒 {L.count('FELL')} 飛ばず {L.count('NO FLIGHT')} | 離地vx 中央 {med(vx):+.3f} | h_y 中央 {med(hy):+.2f} | 着地した本の最終dx 中央 {med(dx):+.3f} m")
PY
echo "#### JB26_DONE"
