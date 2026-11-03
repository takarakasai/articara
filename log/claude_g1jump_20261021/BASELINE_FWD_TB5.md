# G1 前への跳躍の基準 TB5（2026-11-03、ユーザ確定）

前への跳躍の候補はこれと比べる（前の基準は TB2、`BASELINE_FWD_TB2.md`。間の TB3・TB4 は候補のまま昇格していない）。垂直跳びの基準 TV3R とは別。

- バイナリ: `g1_flight_probe_base_tb5`（= j34、articara 73730fc、misa-locomotion def96db）。
- 設定: TV3R の設定（`JUMP_ABORT=1 JUMP_PUSH_R=0.3 QD_CLAMP=40 DIAG_TD=1 FLIGHT_FEET_FLAT=1 DIAG_HX=1 RELEASE_FIRST_OFF=1 WBCW_CONTACT_REST=0.1 WBCW_TRUNK_LAND=0.03 FALL_TILT=1.5 LAND_REF_FOLLOW=2 AIR_ABORT_T=0.08`）に次を足す。
  - `"FLIGHT_FEET_CAPTURE":"0.55","FLIGHT_FEET_CAPTURE_T":"0.06","KP_FLIGHT_FEET":"1000","KD_FLIGHT_FEET":"63","WBCW_TRUNK_PUSH":"0.003","WBCW_MOMENTUM":"10","WBCW_CONTACT":"100","WBCW_TRUNK_LAND_VZ":"0","JUMP_H_NAT":"1","WBCW_CONTACT_REST_LAUNCH":"1","JUMP_VX_TAB":"1.0:1.0,1.5:1.2,2.0:1.6","FLIGHT_TRUNK":"0","FLIGHT_FEET_W":"10","WBCW_MOMENTUM_PUSH":"1","JUMP_LEAN_TAB":"1.0:0,1.5:0.045,2.0:0.06"`
  - TB2 からの差: 空中の胴のタスクを外し足の重み 10（§16・§17）、押しと離しの間だけ角運動量の重み 1（§20）、前出しを 1.5 で 3→4.5 cm・2.0 で 4.5→6 cm。
  - 実行例は `jb52.sh`・`jb53.sh` の TB5（1.0／1.5 は表の値が同じ TB4MP1、2.0 は L6MP1 の実行を共有）。

| 種 | 結果の TAG | JUMP_V 1.0 | 1.25 | 1.5 | 1.75 | 2.0 | 合計 |
|---|---|---|---|---|---|---|---|
| 31415 | `runs/TB5all5` | 44 | 45 | 48 | 46 | 47 | **230/240** |
| 27182 | `runs/TB5all5s2` | 48 | 48 | 47 | 48 | 46 | **237/240** |
| 16180 | `runs/TB5all5s3` | 44 | 48 | 47 | 46 | 47 | **232/240** |

- TB4（`runs/TB4all5{,s2,s3}`）との対: 698→699 of 720（改善 17・悪化 16）、種ごとに z −2.11／+1.00／+1.39。31415 の 1.0・1.25 の損は他の種で再現しない。
- 離地の vx の中央値: +0.50／+0.56／+0.63／+0.65／+0.64 m/s。離地 h_y は 2.0 で +0.52〜0.59（押しの重み 1 の代価）。
- 着地したものの最終の前進の中央値: +0.21／+0.26〜0.27／+0.33／+0.35／+0.35 m（TB2 は 1.0／1.5／2.0 で +0.21／+0.26〜0.27／+0.31）。
- 既知の限界: 2.0 の前出し 7.5 cm は押しが崩れる（離地 vx 負）、1.5 の前出し 6 cm は前へ倒れる（75/96）。
- 動画: `g1_forward_jump_tb4_vs_tb5.mp4`（`diagh/render5.sh`、git には入れない）。

経緯は `doc/from_claude/20261021_g1_jump.md` §16〜§20。
