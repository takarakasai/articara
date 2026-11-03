# G1 前への跳躍の基準 TB2（2026-11-03、ユーザ確定）

前への跳躍の候補はこれと比べる（前の比べ先は HV12k7、`BASELINE_FWD_HV12k7.md`）。垂直跳びの基準 TV3R とは別。

- バイナリ: `g1_flight_probe_base_tb2`（= j31、articara be4fba2、misa-locomotion def96db）。
- 設定: TV3R の設定（`JUMP_ABORT=1 JUMP_PUSH_R=0.3 QD_CLAMP=40 DIAG_TD=1 FLIGHT_FEET_FLAT=1 DIAG_HX=1 RELEASE_FIRST_OFF=1 WBCW_CONTACT_REST=0.1 WBCW_TRUNK_LAND=0.03 FALL_TILT=1.5 LAND_REF_FOLLOW=2 AIR_ABORT_T=0.08`）に次を足す。
  - `"FLIGHT_FEET_CAPTURE":"0.55","FLIGHT_FEET_CAPTURE_T":"0.06","KP_FLIGHT_FEET":"1000","KD_FLIGHT_FEET":"63","WBCW_TRUNK_PUSH":"0.003","WBCW_MOMENTUM":"10","WBCW_CONTACT":"100","WBCW_TRUNK_LAND_VZ":"0","JUMP_H_NAT":"1","WBCW_CONTACT_REST_LAUNCH":"1","JUMP_VX_TAB":"1.0:1.0,1.5:1.2,2.0:1.6","JUMP_LEAN_TAB":"1.0:0,1.5:0.03,2.0:0.045"`
  - 実行例は `jb44.sh` の TB2（1.0／1.5 の結果は表が TB1 と同じ値なので TB1 の実行を共有）。

| 種 | 結果の TAG | JUMP_V 1.0 | 1.5 | 2.0 | 合計 |
|---|---|---|---|---|---|
| 31415 | `runs/TB2all`（1.0／1.5 → TB1f、2.0 → TB2f） | 45 | 47 | 43 | **135/144** |
| 27182 | `runs/TB2all2`（1.0／1.5 → TB1s2、2.0 → TB2s2） | 45 | 45 | 45 | **135/144** |

- HV12k7 との対（`jrand.py cmp runs/HVall TB2all` など）: 2 つの種で 250→270（改善 35・悪化 15、z +2.83）。2.0 だけ 93→88。
- 離地の vx の中央値: +0.50／+0.53〜0.54／+0.54〜0.55 m/s。
- 着地したものの最終の前進の中央値: +0.21／+0.26〜0.27／+0.31 m（HV12k7 +0.24／+0.22／+0.17）。
- 中間の高さ 1.75（種 31415、表の補間で前出し 0.0375・指令 1.4）は 46/48、前進 +0.29。
- 値は種 31415 の掃引で選んだ。選ぶのに使っていない種 27182 でも 121→135（z +2.56）。

経緯は `doc/from_claude/20261021_g1_jump.md` §14・§15。
