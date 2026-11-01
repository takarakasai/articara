# kyo46rs 基準 v4（2026-11-01）── v3 ＋ 押しの回復 ＋ COP_LEASH（候補 kl9、ユーザ判断で例外として昇格）

`controls.json` = `log/claude_g1rand_20260925/kyo_base_v3.json` ＋ `log/claude_kyo46rs_push_20261031/cand_kl9.extra`。プリセットは `WALKER_PRESET=kyo46rs_v4`（misa-locomotion `Config::kyo46rs_v4()`、commit 5d978d5、`tests/presets.rs` が `fixtures/kyo46rs_v4.json` = この controls と照合）。バイナリは main の `log/claude_g1rand_20260925/kyo46rs_walk_tu65`（同じ挙動の worktree 製 ml9・家族を回した ml8）。URDF は v3 と同じ。

**main への取り込み（11-01）**: misa-locomotion の 3 コミット（b49d71d・a672047・5d978d5）は push 済み。articara main は rev を 5d978d5 に上げ、host の変更（`log/claude_kyo46rs_push_20261031/mpcloss/leash_host.patch`: `src/biped/log.rs` の `ContactReadout::sole_cop` と `examples/kyo46rs_walk/main.rs` の `Observation::sole_cop`）を入れた。main のバイナリ `log/claude_g1rand_20260925/kyo46rs_walk_tu65` は、未設定で eq 96・押し 14・eqdiag 4 が tu63 とバイト一致し、kl9 の家族の 11 本（8 ベンチ）を再現し、`WALKER_PRESET=kyo46rs_v4` が全 controls と一致する。

その後（同日）misa-locomotion を最新の 29c6064（走行の 14 コミット、すべて既定 off）に上げた。main のバイナリ `log/claude_g1rand_20260925/kyo46rs_walk_tu66` でも同じ 3 つの確認がすべて一致（`main_tu66_check.out`）。以後の v4 の作業は tu66 から。

中身（v3 に足したもの）:
- レンチ MPC の適用（G1 の 9 キー: `STAIR_WRENCH_MPC WRENCH_MPC_ALWAYS WRENCH_MPC_APPLY STAIR_WRENCH_COUPLED=0 MPC_NODE_CENTRE MPC_NET_COP_W=10 MPC_SEED_PREVIOUS=2 MPC_W_SMOOTH=10 MPC_SOLE_AUTHORITY`）
- MPC の着地（`MPC_FOOTSTEP` の寸法は G1 の半分、内側なし `Y_IN_FWD=0`、段の区間では切る `MPC_FOOTSTEP_STAIR_OFF=1`）
- `PUSH_REPLAN=0.03`（全方向、STAND=0・RAMP=0、HOLD=10）、`PUSH_TRIG_LPF_HZ=5`
- `COP_LEASH=0.005 COP_LEASH_TILT=0.02 COP_LEASH_MPC=1`（片脚支持の傾いた支持足の CoP を、足裏の測った CoP ± 5 mm に縛る。両脚支持・段の区間では掛けない）

## 成績（家族 `kfamily.sh` 8 ベンチ 1,552 本、3 つの種、対 v3）
- 改善 311・悪化 87（z +11.23）。押し 0.9 N·s 遊脚側 90° 69／0、支持足側 270° 57／1、前 64／13。
- 凹凸 10 mm: +51／−15（振り直しの地形 `SEED_MIX_TERRAIN=1` で +57／−11、z +5.58）。
- 凹凸 5 mm: +6／−9（振り直しで +4／−13、z −2.18）── 残る損。
- 降段 20 mm: 20／35 ── 規則 A の悪化（MPC の両脚支持の権限が出どころ、§11）。**この悪化を例外として昇格**。
- 候補 kpc4L（COP_LEASH なし）比では 81／31、規則上の悪化なし。
- 判定 `log/claude_kyo46rs_push_20261031/kfam_judge_kl9.out`・`kfam_judge_kl9_vs_kpc4L.out`、振り直し `mixcmp_k3_kl9.out`。
- 未設定の一致: ml8 は eq 96・押し 14・eqdiag 4 が tu63 とバイト一致（`eq_ml8_result.out`）。ml9 は kl9 の家族の run.log を再現し、`WALKER_PRESET=kyo46rs_v4` は全 controls と一致（8 ベンチ 11 本）。

基準は `KFAM_kl9n_*`（31415／27182）・`KFAM_kl9o_*`（27182／31415）・`KFAM_kl9t_*`（16180／14142）。凹凸・雑音の振り直しは `KFAM_kl9{n,o,t}_{kbump05m,kbump10m,ksensem}`（`mixchk.sh`）。
判定は `python3 log/claude_g1rand_20260925/family_judge.py --kyo kl9n:CANDn kl9o:CANDo kl9t:CANDt`。

弱点: 降段 20 mm（権限）、凹凸 5 mm（傾きの門の下の個体と見ている、未確認）、センサの遅れ 6 tick（全滅、v3 から）。経緯は `doc/from_claude/20261031_kyo46rs_push_recovery.md` §5〜§13。
