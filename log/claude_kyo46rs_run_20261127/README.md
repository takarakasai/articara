# kyo46rs の走りの下見（2026-11-27、ユーザ選択「推奨の進め方で」= §76 の案 1）

G1 の走りの基準 WR10 を kyo46rs（6.9 kg、脚 0.30 m、基準 v4）に移す下見。経緯は `doc/from_claude/20261127_kyo46rs_running_probe.md`。

- `kr.sh OUT [K=V ...]`: v4 の controls（`log/claude_kyo46rs_base_20261101v4/controls.json`）＋ WR10 の走りのキーを脚の比で縮めたもの（距離 ×0.45、時間 ×0.67）。VX は STRIDE／T_SS（G1 の既定と同じ）、`INIT_JITTER=0.01`、`T=10 N_STEPS=28`、`FLAT_START_X=30`。既定のバイナリは tu88（kyo の v4 の平地と押しで tu68 とバイト一致）。
- 結果は `results.txt`（種 1〜8 または 1〜4、歩幅 0.043）。
- 11-28: `lat.py CSV...`（一歩ごとの横の表）で横の崩れの根を特定。横の Raibert 項 RUN_FOOT_K が普段の揺れで毎歩足を外へ出していた。`kr01.sh OUT` が候補 KR01（重み付きの QP、COM_PREPLAN=0、T_SS 0.25、T_FLIGHT 0.06、RUN_FOOT_K=0）で、種 1〜16／17〜32 とも 48／48、空中の中央値 50 ms。数字は `results.txt` の 9〜13。
- 11-28（続き）: `kr05.sh OUT`（G1 の wr05.sh を kyo に縮めた 21 条件、`RUN=` で候補を選ぶ、既定は kr01.sh）。`kr03.sh` が基準 KR03（11-28 にユーザ確定、`BASELINE_RUN_KR03.md`。KR01 ＋ RUN_FOOT_XI_GY=1.0 ＋ PUSH_REPLAN=0.03）で、21 条件 875／876 of 1008（KR01 は 717／736）。数字は `results.txt` の 14〜15。
- 11-28（続き 3）: `kr04.sh` が基準 KR04（11-28 にユーザ確定、`BASELINE_RUN_KR04.md`。KR03 ＋ G1 のチャンピオンと同じ胴の速度の融合と IMU の模型）。雑音 24 → 48、21 条件 900／893 of 1008。`pair.py A B` は 2 つの kr05.sh の出力の本ごとの対（条件ごとの +／− と合計の z）。数字は `results.txt` の 16。
- 11-28（続き 4）: `kr06.sh` が基準 KR06（11-28 にユーザ確定、`BASELINE_RUN_KR06.md`。KR04 ＋ COP_LEASH=0、凹凸の根）。21 条件 1008／1008。`spd.py DIR ...` は電池の条件ごとに立った本の出た速さ（指令比の中央値と 40 % 未満の本数）。凹凸と荷物は立つが足踏み（18〜68 %、20 %）。数字は `results.txt` の 17。
