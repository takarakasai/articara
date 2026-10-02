# G1 ノブの束と否定済みノブ（2026-10-02、計画 4 の整理）

チャンピオンは **v5.2**（`log/claude_g1champion_20260927v52/controls.json`）。以下はすべて既定 off で、v5.2 のバイナリとビット同一。

## BD 構成の束（「MPC が CoP を計画し WBC が従う」）── 性能は v5.2 と同等、後ろ押しのみ劣る
| ノブ | 値 | 役割 |
|---|---|---|
| `MPC_PCMD` | 1（混合なら 0.5） | WBC の CoP 指令を MPC の第 1 節点の正味 CoP に置換 |
| `MPC_PCMD_FB` | 1 | 解いた時点の ξ 予測からのずれを毎 tick 補正（零次ホールドの損失を消す） |
| `MPC_CAPTURE_LEX` | 1 | 正味 CoP を到達多角形内で LIPM 捕捉最適に計画し、等式で固定（Clarabel） |
| `MPC_CAPTURE_LEX_WP/WS/MARGIN/FMARGIN` | 0.1／0.1／0.005／0.1 | 計画 CoP からの逸脱・変化の重み、箱の余裕、頂点回避の力の余裕 |
| `MPC_KKT_COMP_TOL` | 1e-2 | 内点法の相補性残差の閾値（KKT 門） |
測定: 前押し 11 N·s 36→46（+2.67）、後ろ押し 21→16（−2.24）、段同点（§15）。後ろ押しは WBC が腰で加速度を作る折衷（§17）。

## 否定済み（既定 off のまま残す。再調整に時間を使わない）
| ノブ | 否定の根拠 |
|---|---|
| `MPC_CAPTURE_COP_W`（罰則の捕捉行） | WBC が従わないと不活性、従うと重みの刃（§12、§14） |
| `MPC_FZ_SPLIT_EQ`（足ごとの配分固定） | 両脚支持の荷重移動を禁じて全転倒（§14） |
| `MPC_PCMD_FROM_ACC`、`MPC_F_MAX_SCALE`、`MPC_W_ACC` 増 | 後ろ押しを戻さない（§14） |
| `MPC_LANDING_TIME`（着地時刻を捕捉残差で） | 前押し 1〜7/48 対 36（§16） |
| `MPC_LANDING_CAPTURE_N`（捕捉地平延長） | 後ろ押し 2/48（§18） |
| `MPC_FOOTSTEP_CAPTURE_FLOOR`、箱 0.20〜0.35、`CP_T_MIN` 0.3、延長 `CP_T_MAX` 1.5 | 後ろ押し 21 不変（§18、§23） |
| `MPC_FOOTSTEP_MAX_Y_IN` 0.05〜0.35（交差ステップ） | 横押し 13 不変、遊脚速度の壁（§18、§23） |
| `CP_LAT_EMERGENCY`（緊急着地） | 足は下りるが荷重が入らず無効（§18、§23） |
| `MPC_FOOTSTEP_TERRAIN_W`（地形コスト） | 段で着地変数が発火せず効果なし（§19） |
| `MOM_REF_MPC` | v5.2 では不活性（§21） |
| `WBCW_REGULARISE` 1e-2〜、`STAIR_WRENCH_COUPLED=1` | MPC レンチへの追従強化で後ろ押し 0/48（§17） |
| `MPC_FOOTSTEP_MOMENT=0`、`MPC_FOOTSTEP_FREEZE_FRAC*` | 第 1 段で否定（09-27） |

## ベンチマーク拡張（計画 3）の口
`VX2`／`VX2_AT`／`VX2_RAMP`（速度変更）、`SENSE_DELAY_TICKS`／`SENSE_NOISE_Q/QD/V/W`／`SENSE_SEED`（センサ）、
`rand.py MODE=flat`（CONDS=`KEY=VAL;…`）、`START_X`（降段は 0.40）。基準バッテリ `log/claude_g1rand_20260925/bench_ext.sh`。

## 実時間性（計画 4）
制御 5 ms。MPC は余裕（p99 2.5 ms／20 ms）。WBC の裾は段ゾーンの厳密カスケードのみ（p99 11 ms、超過 13〜28%）。
`WBC_WEIGHTED_OFF_STAIRS=0`（重み付き QP 全域）で単発は超過 0.8%・完走。N=48 で確認中（§26）。
