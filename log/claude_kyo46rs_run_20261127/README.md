# kyo46rs の走りの下見（2026-11-27、ユーザ選択「推奨の進め方で」= §76 の案 1）

G1 の走りの基準 WR10 を kyo46rs（6.9 kg、脚 0.30 m、基準 v4）に移す下見。経緯は `doc/from_claude/20261127_kyo46rs_running_probe.md`。

- `kr.sh OUT [K=V ...]`: v4 の controls（`log/claude_kyo46rs_base_20261101v4/controls.json`）＋ WR10 の走りのキーを脚の比で縮めたもの（距離 ×0.45、時間 ×0.67）。VX は STRIDE／T_SS（G1 の既定と同じ）、`INIT_JITTER=0.01`、`T=10 N_STEPS=28`、`FLAT_START_X=30`。既定のバイナリは tu88（kyo の v4 の平地と押しで tu68 とバイト一致）。
- 結果は `results.txt`（種 1〜8 または 1〜4、歩幅 0.043）。
