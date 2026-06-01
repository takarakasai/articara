# 実機 Go2 で歩容を動かす計画 — quadruped-gait + unitree-sdk-rs

articara の歩容生成（`quadruped-gait`）で確認した歩容を、**articara 本体（GUI/エディタ）を使わず**
`quadruped-gait` + `misarta` + `unitree-sdk-rs`（先行構築した Go2 低レベル SDK）で実機 Go2 上で動かす。

- 作成日: 2026-06-02
- 関連: [unitree-sdk-rs の Go2 実機ブリングアップ手順](../../unitree-sdk-rs/doc/go2-bringup.md)
- 決定事項:
  - 脱 articara 方式 = **B（auto-detect を gait 側へ移植）**
  - 最初の歩容 = **LinearCrawl**（静的安定・最安全）
  - ランナー配置 = **articara ワークスペース内の新クレート**

## 0. 背景と依存関係の地図（調査結果）

```
articara (GUI/エディタ)              ← 使わない
  └ src/gait.rs   …… quadruped-gait のラッパ＋モデルから設定を自動検出（articara::RobotModel 依存）
quadruped-gait (歩容生成, CHAMP相当)  ← 使う
  └ コンストラクタは new(GaitConfig, KinematicsConfig) のみ。misarta モデルも articara も不要
  └ tick(dt) → 12関節目標 (関節名, q[rad])、順序 FL/FR/RL/RR×(hip/thigh/calf)、IK符号系
misarta (運動学/動力学プリミティブ)    ← 使う（.misa ロード、FK、軸/offset 取得）
unitree-sdk-rs (Go2 低レベル)         ← 使う。LowCmd を rt/lowcmd で 500Hz 送信
```

### 重要な事実
- **quadruped-gait は実行時 articara 非依存。** 単純な Trot/Crawl IK パイプラインは misarta を実行コード上呼ばない（nalgebra 幾何のみ）。misarta を呼ぶのは未使用の `wbc` モジュールだけ。
- **articara 依存は初期セットアップのみ**：モデル→`KinematicsConfig`、関節名→index 解決、軸符号（IK→URDF/misa 符号補正）、MPC auto-config（MPC モード時のみ／Crawl には不要）。
- **モデルは `.misa` から articara 不要でロード可能**：`misarta::native::load("go2.misa")` → `misarta::model::Model<f64>`。`go2.misa` には足リンク（FL_foot 等）と `bent_home` 姿勢が焼き込み済み。
- `KinematicsConfig` は serde 直列化可能。
- misarta `Model<f64>` は Pinocchio 流：`joints`（1-based, 各 `JointModel{name, joint_type(軸), parent, placement(SE3)}`）、`link_names` 並列。FK は `misarta::fk`。→ auto-detect 移植に必要な情報は全て取得可能。

### ご質問への回答（articara 依存機能は gait_controller に持たせるべきか）
| 機能 | 現状 | 方針 |
|---|---|---|
| `tick` 実行時の IK→符号/index 補正・各 setter | articara `src/gait.rs` | quadruped-gait 側で完結可能（モデル非依存。本計画では新ランナーが担う） |
| auto-detect（モデル→KinematicsConfig/符号）| articara `RobotModel` 依存 | **quadruped-gait へ移植**（misarta::model::Model 入力）= 本計画 M1 |
| モデルロード（mjcf/robot）| articara 内 | 不要。`misarta::native` で `.misa` 直接ロード |

## 1. マイルストーン

### M1. auto-detect を gait_controller へ移植（★核心・ロボット不要）
- **新モジュール `quadruped_gait::autodetect`**（misarta feature 付き）。
  入力 `&misarta::model::Model` + 足リンク名 `[(LegId, &str); 4]` → 出力
  `KinematicsConfig` ＋ 関節符号テーブル `[[f64;3];4]` ＋ `KneePattern`。
- 移植元: articara `src/gait.rs` の `auto_detect_leg_kinematics`（脚チェーンを climb して
  `hip_offset` / `hip_to_thigh_y` / `upper_leg_m` / `lower_leg_m` / `nominal_foot_body` を導出）と
  `GaitController::build` の軸符号導出（`joint.joint_type` の軸向きから IK↔URDF 符号を決定）。
  articara `RobotModel` ベースのツリー走査・FK を misarta `Model` の `joints[].parent` / `placement` /
  `misarta::fk` に置き換える。
- パリティ確認: 同じ go2.misa に対し articara `auto_detect_kinematics_config` と数値一致を確認。

### M2. ランナークレート scaffold（ロボット不要）
- 新クレート `articara/go2-gait-runner`（workspace member に追加）。
- 依存: `quadruped-gait`・`misarta`（path）・`unitree-go2`（path = `../../unitree-sdk-rs/crates/unitree-go2`）。
  **articara 本体に依存しない。**
- 注意: `unitree-go2` は `cyclonedds-sys` 経由で libddsc をビルド。直接実行時は
  `LD_LIBRARY_PATH=/home/takara/cyclonedds-install/lib`（`cargo run` は自動設定）。

### M3. オフライン検証（ロボット不要）
- `go2.misa` ロード → `autodetect` → `AnyGaitController::new(GaitMode::LinearCrawl, GaitConfig::crawl(), kin)`。
- vx=0 と微速で N tick 実行し 12 関節軌道を dump。
  - vx=0 で nominal stance 近傍（位相凍結）であることを確認。
  - 全関節が Go2 関節リミット内であることを assert。
- articara `examples/go2_linear_crawl` の出力と数値パリティ確認。

### M4. 関節マッピング & 送信パス（ロボット不要部分）
- 関節名 → Go2 LowCmd モータ index（FR=0-2 / FL=3-5 / RR=6-8 / RL=9-11、各 hip/thigh/calf）テーブル。
  ※ quadruped-gait は FL/FR/RL/RR 順、Go2 は FR/FL/RR/RL 順なので **脚の入れ替えに注意**。
- 符号補正（IK→misa 軸符号 = 実機 Go2 符号）を適用して LowCmd を生成（kp/kd PD + CRC、`unitree-go2::set_crc`）。
- `unitree-go2` の joint 定義（index/順序）と突き合わせて検証。

### M5. 実機 安全ラダー（ロボット使用・段階ごとに確認）
前提: 周囲クリア、手元コントローラ待機。各段階の前に確認する。
1. `go2_motion_ctrl release eth0`（sport_mode OFF）→ 現在姿勢から **歩容 nominal stance へ ramp**
   （go2_stand 流の線形補間、kp 漸増）。
2. **在地 vx=0**：歩容を起動。LinearCrawl は速度0で stance 保持（位相凍結）。
   500Hz 送信パス全体を立位ほぼ静止で検証。
3. **微速前進** vx=0.05 m/s を数秒 → 様子を見て増速。
4. 終了: vx→0 → 伏せ姿勢へ ramp で安全終了（地面で折り畳み）。または sport_mode へハンドオフ。

## 2. 技術メモ・既知のリスク

- **関節順序の不一致**: quadruped-gait FL/FR/RL/RR ↔ Go2 ハード FR/FL/RR/RL。関節名でマップして取り違えを防ぐ。
- **符号系**: quadruped-gait の出力は IK 符号系。`go2.misa`（Menagerie 由来＝実機準拠）の軸符号へ補正してから送る。
- **nominal stance**: LinearCrawl の nominal_foot_body は既定で脚直下。`bent_home`（hip=0, thigh=0.9, calf=-1.8）相当の
  立位クラウチ高さになるよう auto-detect 時に設定（articara go2_gait と同様）。
- **stance への ramp**: 開始姿勢（伏せ/任意）から歩容 nominal stance へ滑らかに補間してから歩容クロックを起動（ジャンプ防止）。
- **制御周期**: 500Hz（dt=0.002）。go2_stand と同じ Instant ベースの定周期ループ。
- **ゲイン**: 最初は position PD のみ（feedforward torque は使わない）。kp/kd は go2_stand の MOVE 既定（60/5）付近から保守的に。
- **ビルド依存**: quadruped-gait のビルドには misarta（`/misarta`、gitignore・別 clone 済み）が必要。確認済みでビルド成功（misarta + quadruped-gait, exit 0）。

## 3. 進め方
M1〜M4（オフライン）を先に通し、M5（実機）の前に一度立ち止まって確認する。
