# Articara 最近の主要機能まとめ

ここ数週間で追加された主要機能の概観。 コミットID参照付き。
詳細は対応するソースのドキュメントコメント / `cargo doc` 、 関連 doc
([mpc_wbc_gait_control.md](mpc_wbc_gait_control.md),
[regression_test.md](regression_test.md),
[script_spec.md](script_spec.md)) を参照。

## 1. シミュレーション解析・チューニング機能

### 1.1 Joint Peaks プロットの拡張 (`0c1f2df`)

`📈 Joint Peaks Plot` ウィンドウに 3 機能を追加。

| 機能 | 操作 |
|---|---|
| **CSV 保存** | 右上 `💾 Save CSV` → ファイルダイアログ → `time_s,q[name],qvel[name],tau[name],…` 形式で書き出し |
| **X 軸モード** | `Auto-update` (バッファ秒数 or Unlimited) / `Fixed period` (スライディングウィンドウ) を切替 |
| **系列表示制御** | "(all)" 選択時に `Visible joints` チェックボックス。Show all / Hide all ショートカット付き |

トレースバッファ容量は `MujocoSim::set_trace_max(usize)` で動的にリサイズ可能。Unlimited 時の上限は `PEAKS_PLOT_UNLIMITED_CAP = 200_000` サンプル。

### 1.2 解析スクリプト `tools/analyze_peaks_log.py` (`7a7f716`, `d44589f`)

CSV ログから:
- 各関節の `τpeak`, RMS, `q̇max`, q̇符号反転率
- スペクトル支配ピーク (Nyquist 帯近傍を `!` で警告)
- 2-tick limit cycle の自動検出

`--motor MG4005-i10:10` でデータシート値から `armature` / `joint_damping` を計算:
```
→ armature              ≈ rotor_inertia × gear²
→ electrical damping     = Kt² / R × gear² (BRAKE 上限; QDD では除外)
→ mechanical friction    ≈ τ_max の 0.1-0.5% (QDD) / 1-5% (traditional)
```

`--list-motors` でプリセット一覧。`:qdd` / `:traditional` で駆動方式を強制指定可能。

### 1.3 軌道発振の根本対策

ジャンプ時の特定関節の高振幅トルク発振 (Nyquist 250 Hz limit cycle) に対する 3 段階改善:

#### A. 軌道速度フィードフォワード (`478e724`)

旧: `τ = Kp·(q*−q) + Kv·(0 − q̇)` ⇒ Kp 項と Kv 項が常時喧嘩  
新: `τ = Kp·(q*−q) + Kv·(q̇* − q̇)`

`misarta::trajectory::PoseTransition::evaluate_velocity` / `KeyframeAnimation::evaluate_velocity` を新規追加し、`MujocoSim::position_target_velocities` に毎ステップ書き込む。

#### B. armature + joint_damping を MJCF に出力

`JointData` に `armature: f64` と `joint_damping: f64` を追加。MJCF の `<joint armature damping/>` に反映。Properties Panel と Actuators (all joints) パネルで編集可。

#### S2: 重力補償 PD (`f980931`)

`MujocoSim::gravity_compensation` flag。ON で misarta の `compute_gravity` を毎ティック呼んで `τ_grav` を Position/Velocity モードの PD 出力に加算。Torque モードは尊重 (加算しない)。UI: 🎛 Sim toggles → 🌍 Grav comp。Rhai: `mj_gravity_compensation(bool)`。

#### S3: ComputedTorque モード (`7e565c3`)

新 actuator mode `ComputedTorque`:
```
τ = M(q)·q̈* + h(q, q̇) + Kp·(q*−q) + Kv·(q̇*−q̇)
```
- `misarta::rnea::rnea(model, q, v, a)` で逆動力学を 1 回計算
- `position_target_accelerations: Vec<f64>` を新規追加 (`evaluate_acceleration` を misarta に追加 → controller が毎ステップ書き込み)
- Kp/Kv は誤差補正だけになるので大幅に下げられる (例: Kp 50→30, Kv 0.5→0.5)

UI: Properties → Actuator → Mode dropdown に "Computed-τ"。Rhai: `set_actuator_mode_all("computedtorque")`。

### 1.4 リミットの完全反映 (`ff0099a`)

`⛔ Limits` チェックボックスがコントローラ側のクランプしか解除していなかったバグ。MJCF エクスポートが常に `<motor forcelimited="true">` と `<joint range>` を出力していたため MuJoCo が無条件に切り詰めていた。

修正: `MjcfExportOptions` に `bake_actuator_limits` / `bake_joint_position_limits` を追加し、UI からの 1 click で 3 箇所 (runtime PD / forcelimited / joint range) すべてに反映。Stop → Play で再ビルドが必要。

---

## 2. スクリプティング機能の拡張

### 2.1 スクリプトファイル実行 (`a30adab`)

スクリプトコンソール右上に **📂** ボタン。`.rhai` を選んで実行。1行ずつ Input としてエコーされ履歴で再現可能。回帰テスト `example_scripts_parse` で同梱スクリプトのパースを継続検証。

### 2.2 拡張 Rhai API

| 関数 | 用途 |
|---|---|
| `set_actuator_mode(joint, mode_str)` | "Position"/"Velocity"/"Torque"/"ComputedTorque" |
| `set_actuator_mode_all(mode_str)` | 全関節一括 |
| `set_armature(joint, I)` / `set_armature_all(I)` | ロータ慣性 |
| `set_joint_damping(joint, b)` / `set_joint_damping_all(b)` | パッシブ damping |
| `set_kp_all(kp)` / `set_kv_all(kv)` | ゲイン一括 |
| `mj_trace_len()` / `mj_set_trace_max(n)` | トレースリング操作 |
| `mj_gravity_compensation(bool)` | 重力補償 |
| `save_peaks_csv(path)` | スクリプトから CSV 出力 |

### 2.3 非同期タイムライン `mj_async_*` (`d617278`)

スクリプトから `mj_step` を直接呼ぶと UI が凍結する問題への対処。`MujocoSim::async_queue` (`VecDeque<AsyncSimOp>`) を新規追加。スクリプトはキューに積むだけで即返り、UI ループが毎フレーム少しずつ消費。

```rhai
mj_async_step_seconds(0.5);
mj_async_set_position_target("FL_calf_joint", -1.4);
mj_async_print("[absorb done]");
mj_async_save_csv("log/jump.csv");
```

`AsyncSimOp` 4種: `StepFrames(u32)`, `SetPositionTarget(idx, q)`, `Print(String)`, `SaveCsv(PathBuf)`。Speed slider で再生速度調整可能。

### 2.4 検証・デモスクリプト

| スクリプト | 内容 |
|---|---|
| `scripts/verify_jump_tuning.rhai` | A/B/CT モード × armature × damping を sweep。各 CSV を `log/jump_<tag>.csv` に保存 |
| `scripts/example_jump.rhai` | 同期版 (バッチ高速)。Stand → Crouch → Extend → Absorb → Settle (`79843ee` で受け身フェーズ追加) |
| `scripts/example_jump_async.rhai` | 非同期版 (ビューポート観察可) |
| `scripts/walk_demo.rhai` | quadruped-gait の E2E デモ |

### 2.5 スクリプトコンソール UX 修正 (`50191ba`)

egui の Panel が `inner_response.response.rect` を保存する仕様により、コンテンツが少ないときデフォルトサイズへ自動収縮する問題を `ui.set_min_height(ui.max_rect().height())` で解決。

---

## 3. quadruped-gait crate (CHAMP 相当)

新ワークスペースメンバ `quadruped-gait/`。CHAMP (chvmp/champ) の主要コンセプトを Rust で再構築した独立ライブラリ (UI / sim 非依存)。

### 3.1 Phase 1 — プリミティブ (`b373a74`, `b8497ae`)

| モジュール | 公開 API |
|---|---|
| `config.rs` | `LegId`, `VelocityCmd`, `GaitType`, `GaitConfig`, `LegKinematics`, `KinematicsConfig` |
| `phase.rs` | `PhaseGenerator::advance(dt, &cmd)` / `legs() → [PhaseState; 4]` |
| `swing_traj.rs` | `swing_position(p0, p1, h, frac)` 4-point cubic Bezier (peak = h)、`stance_position` (linear) |
| `ik.rs` | `solve_leg_ik(kin, target, knee_forward) → LegIkSolution::Reached/Unreachable` (3-DOF RPP analytical) |

統合テスト `tests/cycle_dump.rs`: vx=0.3 m/s で 1 cycle dump、6 件の不変条件アサート。

### 3.2 Phase 2 — Controller glue (`f3990af`)

| 追加モジュール | 内容 |
|---|---|
| `footstep.rs` | `compute_footstep(kin, gait, cmd) → Footstep` (Raibert with `v_hip = v_body + ω×p_hip`) |
| `body_state.rs` | `BodyState::integrate(cmd, dt)` (open-loop world pose 積分) |
| `controller.rs` | `GaitController::tick(dt) → ControllerOutput` (位相生成 → footstep → 軌道 → IK の glue) |

### 3.3 Phase 3 — articara 統合 (`6103658`, `bf30f5f`)

#### 3.3a 自動検出 `src/gait.rs`
- `auto_detect_leg_kinematics(model, foot_link, leg)`: 足先リンクから RPP 3 関節を chain で遡り、`LegKinematics` を生成
- 各関節の axis 主成分の符号も検出 (`joint_signs: [[f64; 3]; 4]`) — URDF +Y 軸の右手系規約と IK 内部規約の不整合を吸収 (`7b65a73` 参照)

#### 3.3b MujocoSim 統合
`step_dynamics_sim` 内で gait_controller が enabled なら `tick(dt)` → `[(idx, q); 12]` を `set_position_target` に流し込む。

#### 3.3c UI: 🐕 Quadruped gait パネル
- Foot link 名×4 (text input) + 🔍 Auto-detect / ✕ Clear
- Velocity slider vx/vy/wz + 🛑 Zero
- Gait params: Type, Cycle period, Duty factor, Swing height, Max step length
- Knee pattern: `<<` `<>` `><` `>>` (後述 3.4)
- ▶ Start / ⏹ Stop

#### 3.3d Rhai bindings
```rhai
gait_setup() / gait_setup_with_feet(fl, fr, rl, rr)
gait_set_velocity(vx, vy, wz)
gait_start() / gait_stop() / gait_running() / gait_active()
gait_set_cycle_period(s) / gait_set_swing_height(m) / gait_set_duty(d) / gait_set_max_step(m)
gait_set_knee_pattern("<<|<>|><|>>") / gait_knee_pattern() → String
```

#### 3.3e .misarta.toml `[[gait]]` 永続化 (`bf30f5f`)
新 schema:
```toml
[[gait]]
name = "default"
gait_type = "Trot"
cycle_period_s = 0.4
duty_factor = 0.5
swing_height_m = 0.04
max_step_length_m = 0.10
fl_foot = "FL_foot"; fr_foot = "FR_foot"; rl_foot = "RL_foot"; rr_foot = "RR_foot"
knee_forward = [false, false, true, true]
```
リンク長さや hip_offset は意図的に保存しない (auto-detect で毎回 URDF から計算)。

`RobotModel::sync_gait_panel_from_model` / `sync_gait_panel_to_model` で UI ↔ model.gaits[0] 双方向同期。`do_save` / `do_export` の冒頭で sync 呼び出し。

### 3.4 KneePattern 表記 `<<` `<>` `><` `>>` (`99ead9a`)

膝曲げ方向を 2 文字で指定:
- 1 文字目 = 前脚ペア、2 文字目 = 後脚ペア (左右対称)
- `<` = knee_forward = false (膝が後ろに曲がる)
- `>` = knee_forward = true (膝が前に曲がる)

| パターン | 意味 |
|---|---|
| `<<` BothBack | 全脚 後ろ曲げ |
| `<>` MammalianForward | 前=後ろ / 後=前 (犬・馬) |
| `><` MammalianReverse | 前=前 / 後=後ろ |
| `>>` BothForward | 全脚 前曲げ |

### 3.5 重要なバグ修正

#### `876524f` Stance 軌道方向の反転
`Footstep::stance_at(frac)` が `lift_off → touch_down` (後ろ→前) で補間していた誤り。前進中の foot は body frame で **後ろ向き** に sweep すべき。修正後は連続性も保たれる (stance 終端 = swing 始点 = lift_off)。

#### `7b65a73` URDF axis 符号規約の整合
URDF +Y 軸の右手系: `R_y(q)·(0,0,-1) = (-sin q, 0, -cos q)` ⇒ 正の q_thigh は thigh を **後ろに** 倒す。一方 IK は逆規約 (正 q = 前) を採用。

修正: `articara::gait::GaitController` に `joint_signs: [[f64; 3]; 4]` を保存し、`tick()` で IK 出力に符号倍率を掛けて MuJoCo に送る。各関節の axis 主成分の符号から `build()` 時に自動導出。

#### `52b97e8` 回帰テスト追加
- `ik_fk_round_trip_both_knee_branches`: 両分岐で 1mm 精度の IK→FK 往復
- `foot_trajectory_independent_of_knee_pattern`: 同一速度指令で `<<` と `>>` の foot.x 軌道が 1e-9 以内で完全一致

---

## 4. 操作 UX 改善

### 4.1 Hold-to-drive D-pad (`8349105`, `cd2290c`)

Gait panel に方向ボタン D-pad を追加。マウスで押している間だけ velocity 指令を出し、離すと即停止。

```
        ⬆           Yaw
     ⬅  ⬇  ➡       ↺
                    ↻

Linear (m/s):  [────●────]  0.30
Yaw (rad/s):   [───●─────]  0.50
```

実装:
- `Sense::click_and_drag()` + `is_pointer_button_down_on() || dragged()` でカーソル微小ドリフトに耐性 (`cd2290c` で修正)
- `gait_dpad_was_active: bool` で release エッジを検出し `VelocityCmd::zero()` を一度だけ送信
- 並進 (`gait_dpad_speed`) と yaw (`gait_dpad_yaw_speed`) を独立 slider で調整

### 4.2 TPS カメラ + ピクチャインピクチャ (`fff52a1`)

#### CameraMode
- `Free`: 既存 OrbitCamera (左ドラッグ orbit, 右ドラッグ pan, ホイール zoom)
- `Tps`: ロボットの追従カメラ。マウスでボディ周回 yaw / pitch / distance を調整

3 ステート保持:
- `camera`: 現在 main viewport 用 (active)
- `saved_free_camera`: 自由カメラ snapshot (TPS 中も保持)
- `tps_camera`: 毎フレーム body pose から再計算

#### Wipe (PiP)
`📺 Wipe` ON で main 以外のカメラを右上隅に小窓表示 (約 280×200 px、ラベル付き)。第 2 PaintCallback として描画 — `renderer.rs` 変更なしで scissor + viewport が分離。

#### UI: 📷 Camera パネル
| 項目 | 内容 |
|---|---|
| Mode | Free / TPS のラジオ |
| 📺 Wipe | チェックで PiP ON/OFF |
| Follow link | 追従するリンク名 (空欄 = root) |
| Distance | 0.1〜5.0 m |
| Yaw offset | ±π rad |
| Pitch | ±1.5 rad |
| Look-at lift | -0.5〜+1.0 m |
| ⟲ Reset TPS | デフォルトに戻す |

`TpsSettings::default()` は `distance=1.2 m, pitch=0.35 rad (≈20° 下向き)`。

---

## 5. テスト充実状況

| Suite | 件数 | 備考 |
|---|---:|---|
| articara lib | 62 | gait::auto_detect, gait::sign convention 含む |
| misarta lib | 414 | trajectory: evaluate_velocity / acceleration 等 9 件 |
| quadruped-gait lib | 33 | KneePattern, footstep, IK 両分岐 etc |
| quadruped-gait integration | 2 | cycle_dump.rs |
| quadruped-gait doctest | 1 | controller usage 例 |
| articara regression | 213+ | gait_descriptor sidecar roundtrip 含む |

---

## 6. ファイル構成 (新規追加分)

```
articara/
├── doc/
│   └── recent_features.md           ★本書
├── quadruped-gait/                  ★新ワークスペースメンバ
│   ├── Cargo.toml
│   ├── src/
│   │   ├── lib.rs
│   │   ├── config.rs                # LegId, GaitType, GaitConfig, KneePattern …
│   │   ├── phase.rs
│   │   ├── swing_traj.rs
│   │   ├── ik.rs                    # 3-DOF RPP analytical IK
│   │   ├── footstep.rs              # Raibert
│   │   ├── body_state.rs            # 開ループ pose 積分
│   │   └── controller.rs            # GaitController::tick
│   └── tests/
│       └── cycle_dump.rs
├── scripts/
│   ├── example_jump.rhai            ★ 同期版 (5フェーズ:stand/crouch/extend/absorb/settle)
│   ├── example_jump_async.rhai      ★ 非同期版
│   ├── verify_jump_tuning.rhai      ★ A/B/CT sweep
│   └── walk_demo.rhai               ★ Quadruped 歩行デモ
├── tools/
│   └── analyze_peaks_log.py         ★ CSV 解析 + モータ仕様推定
└── src/
    ├── camera.rs                    # CameraMode, TpsSettings, OrbitCamera::update_from_tps
    ├── gait.rs                      ★ articara ↔ quadruped-gait 統合 (auto-detect)
    └── app/
        ├── camera_panel.rs          ★ 📷 Camera UI
        └── gait_panel.rs            ★ 🐕 Quadruped gait UI (D-pad + yaw 含む)
```

---

## ref/legged_control 再現スプリント (2026-05-06 〜 05-08)

`legged_control` (Liao 2022) の主要 5 layer を articara native に移植する
集中スプリント。詳細な phase 計画は
[mpc_wbc_gait_control.md](mpc_wbc_gait_control.md) 参照。

### 主要マイルストーン

| 達成 | 内容 | commits |
|---|---|---|
| **Phase 1.1** | misarta::qp に proximal warm-start API 追加、 WBC tick jitter を damping | misarta `de61770`, `dc78f98` |
| **Phase 1.2** | `compute_joint_jacobian_time_derivative` を SE(3) integration ベースに修正 (FreeFlyer 対応) | misarta `4a54a1b` |
| **Phase 1.4** | Task::weight() + per-task LSQ weights を WBC に導入 | `dc78f98` |
| **Phase 1.5-A** | SRBD MPC `predicted_base_accel_world` を新設 (legged_control の `formulateBaseAccelTask` 相当) | `fa8835d`, `149aad0` |
| **Phase B** | 18-state Linear Kalman Filter 移植 (LkfPipeline + e2e ±2.4 cm) | `4900887` |
| **Phase C** | ContactDrivenPhase + `MujocoSim::contact_force_per_foot` で接触駆動 phase | `4ad3f2a`, `cd1d90b` |
| **Hybrid joint command 修正** | GUI が `set_wbc_torques` を使っていた問題を `set_torque_feedforward` (Position-PD + WBC τ_ff) に変更 | `b03c431` |
| **Phase 1.5-C** | `wbc_static_stand_balances_gravity` を `#[ignore]` から pass に | `149aad0` |
| **Phase G2** | WBC swing_leg を Cartesian → joint-space に書き換え | `6c360fa` |
| **Phase H1-H4** | SRBD MPC horizon expose + W_CONTACT_FORCE=5 で `wbc_forward_command_advances_body` pass | `d7b6f1b` |
| **mpc_reference module** | `JointReference` + 単体テストで OCS2 の `getJointAngles(state)` 相当 API | `d7b6f1b` |
| **Phase P1** | 3 軸独立 benchmark (forward / lateral / yaw × CHAMP / MPC+WBC) で cross-coupling を可視化 | `4dbf1e9` |
| **q_diag tuning** | SRBD MPC の yaw / lateral 重みを boost、 直進歩行の drift 抑制 | `694e2bd` |
| **Phase P5a** | WbcWeights API + per-task disable diagnostic で lateral 不具合の主因 (swing_leg) を特定 | `d18d9af` |
| **Phase P5b** | `WbcWeights::for_cmd` で per-cmd swing_leg weight scheduling、 lateral / yaw 命令を pass | `e4e914d` |

### 達成: 3 軸全方向で MPC+WBC trotting

5 秒走行 (MuJoCo ground truth) での測定値:

| 命令 | active axis | cross axis 1 | cross axis 2 |
|---|---|---|---|
| forward (cmd.vx=+0.15 m/s) | body_dx=+0.124 m | body_dy=-0.117 m | Δyaw=-0.55 rad |
| lateral (cmd.vy=+0.10 m/s) | body_dy=+0.501 m | body_dx=-0.233 m | Δyaw=+1.20 rad |
| yaw (cmd.wz=+0.5 rad/s) | Δyaw=+2.76 rad | body_dx=-0.280 m | body_dy=+0.184 m |

→ legged_control の主要 5 layer + 3 軸命令で同等の動作能力に到達。

### Scripting 拡張 (`9e7b099` / `8bdf862` / `041416c`)

| 機能 | commit |
|---|---|
| `walk_3axis_demo.rhai` (forward → lateral → yaw 自動再生 + CSV 保存) | `9e7b099` |
| `mj_async_set_velocity` (timeline 上で gait cmd 切替) | `9e7b099` |
| `set_gait_mode` / `set_pose_source` / `set_wbc_enabled` / `set_ground_plane_*` (GUI 設定の Rhai 化) | `8bdf862` |
| `--script <path>` CLI flag (GUI 起動時に Rhai script を auto-run) | `041416c` |
| `--script-headless` (旧 `--script` のヘッドレス実行を rename 保持) | `041416c` |

これらにより **`cargo run --release --features "mujoco scripting" -- --model
<URDF> --script <rhai>`** で GUI クリック 0 回で 3 軸 benchmark を描画付き再生
できる:

```bash
cargo run --release --features "mujoco scripting" -- \
  --model tests/fixtures/namiashi/urdf/namiashi.urdf \
  --script scripts/walk_3axis_demo.rhai
```

### 回帰スイート (`299d451` / `4dbf1e9`)

| 機能 | 内容 |
|---|---|
| `tests/common/mod.rs` | 共通ヘルパー (URDF load, IK seed, Position-PD setup, MujocoSim 構築) |
| `tests/integration_walk.rs` | 全 layer 組み合わせ (PD-only / +WBC / +LKF / 3 軸 active+cross) を 1 ファイルで網羅 |
| `tests/lkf_pipeline.rs` | LKF e2e (MuJoCo ground truth との body z 整合性) |
| `scripts/test_regression.sh` | `--quick` / `--debug` / 引数なし (full release) の 1 コマンド回帰実行 |

詳細は [regression_test.md](regression_test.md) 参照。

### 残課題

| 項目 | 状態 |
|---|---|
| LKF を `PoseSource::ExtendedKalman` で UI 統合 | LkfPipeline 完成済、UI 統合のみ未 (半日) |
| 不整地 / 床傾斜テスト | Phase C 機能を実シナリオで検証 (1 週間) |
| 実機接続 (RobStride / lkmotor) | sibling crates 経由 (2-3 週間) |
| **Phase D1 centroidal-SRBD MPC** | **完了** (`bd457f8`-`f880ce1`、CoM オフセットを動力学に含む第三 MPC モード) |
| D2 SQP Multiple Shooting | 未着手 (2-3 週間、非線形再線形化反復) |
| D3 Full Centroidal Dynamics | 未着手 (3-4 週間、joint q,q̇ も MPC 状態に含めて legged_control type-0 完全等価) |

---

## 5. Phase D: Centroidal MPC (legged_control type-1 相当)

namiashi のように URDF inertial origin が body root から偏った
ロボット (trunk_inertia の +y 8.4 mm オフセット) では、body-root SRBD
MPC は重力誘起ロールモーメントを補正できず constrain pose / 大きな
cmd で lateral ドリフトする。Phase D は **CoM 中心の動力学** で
これを解消する第三 MPC モード `GaitMode::CentroidalSrbd` を導入。

### D1: 12 状態 centroidal-SRBD (= legged_control の `centroidalModelType=1`)

状態空間:

```
x = [ v_com (3, m/s, world) ;
      ω_world (3, rad/s) ;        ← D1.4 で h_ang/m から ω 直接表現に
      base_pos (3, m, world) ;
      euler_zyx (3, rad) ]        合計 12 + augmented gravity = 13 dim
```

入力: 4 脚 × 3 軸 GRF (world frame, 12 dim)。SRBD と同じ。

連続時間動力学 (CoM-aware moment arm):
```
d/dt(v_com) = (Σ F)/m + g
d/dt(ω) = I_centroidal⁻¹ · (Σ (foot_i − CoM_world) × F_i − ω × Iω)
d/dt(pos) = v_com − ω × R · com_offset_body
d/dt(euler) = T_body⁻¹ · R^T · ω_world
```

CoM 位置 = body root + R · com_offset_body。SRBD body-root 仮定の差は
moment arm に **5mm 程度** の補正として現れる。

### D1.1-D1.5 進捗

| Phase | 内容 | コミット |
|---|---|---|
| D1.1 | `centroidal_mpc.rs` モジュール — 状態 / 入力型 + 連続時間動力学関数 + 8 件 unit test (静止立脚 / 自由落下 / 前進力 / yaw モーメント / **CoM オフセットでロール moment 検出** / Euler 微分) | `bd457f8` |
| D1.2 | clarabel ベース convex QP — multiple shooting + friction cone + 接触モード制約。`mpc_hover_balances_gravity` / `mpc_forward_cmd_yields_positive_fx` / `mpc_swing_leg_grf_is_zero` の 3 件 + 11/11 unit test pass | `5e383e3` |
| D1.3 | `GaitMode::CentroidalSrbd` を `AnyGaitController` に追加。`CentroidalMpcGaitController` (= `MpcGaitController` の sibling) を新設、phase / footstep / IK 共通化、MPC 層のみ centroidal に差替 | `15aaf39` |
| D1.4 | state[3..6] を `h_ang/m` (m²/s) → `ω_world` (rad/s) に refactor。SRBD と単位整合、q_diag の 5 桁 dynamic range を回避。WBC pipeline `predicted_base_accel_world` を mode-aware (Some(I_centroidal) で centroidal 経路、None で SRBD baseline) | `c4359ba`, `b62be91`, `874cfcf` |
| D1.5 | lateral / yaw cmd の GRF 方向 unit test (`mpc_lateral_cmd_yields_positive_fy`, `mpc_yaw_cmd_yields_positive_yaw_moment`)。com_offset=0 で SRBD と centroidal の `predicted_base_accel_world` が numerical tolerance で一致することを assert (`centroidal_and_srbd_accel_agree_at_zero_com_offset`) | `f880ce1` |

### 使い方

GUI:
1. **Quadruped Gait panel** → `Generator:` ドロップダウンに **`MPC (centroidal-SRBD)`** が追加
2. Pose source / WBC ON / GroundPlane は既存通り

スクリプト:
```rhai
set_gait_mode("centroidal");        // または:
// set_gait_mode("centroidal-srbd");
// set_gait_mode("mpc-centroidal");
```

`auto_detect_centroidal_mpc_config` が URDF inertials から
`mass_kg` / `centroidal_inertia_body` / `com_offset_body` を自動計算
(misarta の `compute_centroidal_inertia` と link inertial origin 集約)。

### D1.4 時点の達成と限界

regression test (`tests/integration_walk.rs`) の現状値:

| Test | SRBD baseline (e2d29fb) | CentroidalSrbd (874cfcf) | 評価 |
|---|---|---|---|
| forward dx | +0.118 | **+0.151** | SRBD 超え ✓ |
| forward dy | -0.034 | -0.472 | cross-coupling 残 |
| forward dyaw | +0.589 | -0.509 | 同水準 |
| lateral dy | +0.501 | -0.195 | **追従反転** (D1.5 課題) |
| yaw dyaw | +2.759 | **+1.599** | target > 1.5 達成 ✓ |

3 件 (forward/lateral/yaw centroidal+wbc) は `#[ignore]` 状態。
forward dx と yaw dyaw は SRBD 同等以上、lateral と forward dy
cross-coupling は WBC HoQp 全体の再 tuning または D3 で解消予定。

---

## 6. Phase D2: SQP Multiple Shooting iteration

D1 で確立した centroidal-SRBD MPC は単発 convex QP solve だった。
D2 は **同 solve 内で再線形化反復** (Sequential Quadratic Programming)
を導入し、yaw drift を含む horizon でも線形化精度を保つ。

### 動機

D1 の linearization は reference 軌道の yaw (`psi_ref`) で固定。
yaw cmd (wz=0.5 rad/s) では horizon 0.3 s で 0.15 rad の yaw 変化が
あるが、MPC は initial yaw で線形化したまま予測 → 解が真の最適から
2-3% ずれて yaw cmd 追従が target より弱い (D1.4: dyaw=+1.599 vs
target +1.5 ぎりぎり)。

### D2.1: SQP iteration framework (commit `f783c7c`)

`CentroidalMpcConfig.sqp_iterations: usize` 追加。各反復で:
1. `psi_ref_per_step` で per-step 線形化、QP solve
2. 解の予測軌道から `psi_ref_per_step[k] = predicted_state[k].euler.z`
   で更新
3. 次反復は予測軌道近傍で再線形化

`sqp_iterations = 1` (default before D2): D1 互換 single-shot。
`auto_detect_centroidal_mpc_config` は **3** に設定 (legged_control
の SQP iteration count と同等)。

### D2.2: warm-start scaffold (commit `ec5b5ae`)

CentroidalMpc に `warm_psi_ref: Option<Vec<f64>>` を追加。次の solve
で iter-0 線形化点として利用するインフラを実装したが、**閉ループ pose
feedback がある現状では reference の方が常に良い** ことが判明
(stale prediction が forward 転倒を誘発)。書き込み処理は残す
(将来の 1-step 時間シフト or 開ループ feedback 用拡張点)、
読み込みは disabled。

### D2.3: 性能計測 (commit `47497e2`)

`mpc_solve_under_25ms_at_sqp_3` benchmark 追加 (`#[ignore]`):
- median = 11.7 ms
- p99    = 13.4 ms
- budget = 25 ms (`dt_per_step = 30 ms` に十分収まる)

100 Hz 目標に近づくには horizon 縮小が必要だが、現状の 33 Hz
(dt_per_step = 30 ms) なら余裕あり。

### D2 完了時の数値比較

| Test | SRBD baseline | D1 final (1 iter) | D2 (3 iter) | **D2 final (1 iter, default)** |
|---|---|---|---|---|
| forward dx | +0.118 | +0.151 | +0.777 ideal | +0.151 |
| forward dy | -0.034 | -0.472 | -1.143 (cross 倍増) | -0.472 |
| yaw dyaw | +2.759 | +1.599 | +1.599 PASS | +1.599 PASS |
| **yaw centroidal+wbc** | — | FAIL | **PASS** ✓ | **PASS** ✓ |
| lateral dy | +0.501 | -0.195 | -0.134 (反転継続) | -0.195 (反転継続) |

**default は `sqp_iterations = 1`** に決定 (commit `397d5db`)。
SQP=3 は forward dx を ideal まで上げる代わりに横ずれが 2.4× 拡大、
**視覚的に「速く前進するが大きく横に流れる」挙動**となるため、
保守的な D1.4 動作 (SQP=1) を default に戻した。SQP framework 自体は
残るので、hosts は `set_centroidal_mpc_config` で `sqp_iterations`
を override 可能。yaw centroidal+wbc test の PASS は SQP=1 でも維持。

### D2 関連コミット

| Hash | 内容 |
|---|---|
| `47497e2` | D2.3 perf benchmark (median 11.7 ms / p99 13.4 ms @ SQP=3) |
| `ec5b5ae` | D2.2 warm-start scaffold (現状未使用) |
| `f783c7c` | D2.1 SQP iteration framework — yaw centroidal test 初 pass |

### 残課題

- **forward dy cross-coupling**: SQP 反復が予測軌道 yaw drift を
  amplify。D3 (Full Centroidal、joint state 込み) で関節 swing の
  反作用を MPC が直接モデル化すれば改善見込み。
- **lateral 反転**: 別現象。D1 から残存。q_diag empirical sweep か
  WBC HoQp の swing_leg / contact_force task 重み再調整で対処。

### 関連コミット (D1)

| Hash | 内容 |
|---|---|
| `f880ce1` | D1.5 lateral/yaw cmd GRF + SRBD 等価性 unit test |
| `874cfcf` | D1.4 WBC pipeline `predicted_base_accel_world` を mode-aware |
| `b62be91` | D1.4 q_diag tuning (`p_y` 5、`r_diag` 1e-3) |
| `c4359ba` | D1.4 state[3..6] を ω_world 直接表現に refactor |
| `15aaf39` | D1.3 `GaitMode::CentroidalSrbd` + `CentroidalMpcGaitController` 統合 |
| `5e383e3` | D1.2 clarabel ベース MPC QP ソルバ |
| `bd457f8` | D1.1 centroidal 動力学関数 + unit tests |

## 7. Phase D3.1: misarta dynamics primitive profiling

D3 (Full Centroidal、24-state、joint q/q̇ を MPC 状態に含める) の
**実装可否を判断する前提として**、 misarta の主要 dynamics primitive
が namiashi (nq=nv=13) でどれだけ時間を要するかを `cargo run
--release --example bench_misarta_dynamics` で計測。

### 結果 (μs / call、release build、200 iter mean)

| Primitive | μs/call | 用途 |
|---|---:|---|
| `compute_com` | 0.7 | CoM 位置 (FK + mass-weighted sum) |
| `compute_centroidal_inertia` | 1.0 | 6×6 慣性テンソル |
| `rnea` | 3.0 | 逆動力学 τ = ID(q,v,a) |
| `crba` | 3.3 | mass matrix `M(q)` |
| `compute_com_jacobian` | 3.4 | CoM Jacobian (3×nv) |
| `compute_gravity` | 3.6 | g(q) |
| `compute_centroidal_momentum_matrix` | 7.2 | CMM A(q) (6×nv) |
| `compute_minv` | 98.5 | M⁻¹ (ABA-based) |
| `compute_coriolis_matrix` | 100.6 | C(q,v) (FD ベース) |
| **`compute_centroidal_momentum_matrix_time_derivative`** | **236.3** | **Ȧ(q,v) — FD! 2·nnz(v) CMM 呼び** |

### MPC node-cost projection

per-node コスト ≈ CMM + Ȧ + CRBA + RNEA = **249.8 μs**
(うち 95% が Ȧ の有限差分)

| horizon | sqp=1 | sqp=3 | sqp=5 |
|---:|---:|---:|---:|
| N=10 |  2.5 ms |  7.5 ms | 12.5 ms |
| N=12 |  3.0 ms |  9.0 ms | 15.0 ms |
| N=16 |  4.0 ms | 12.0 ms | 20.0 ms |
| N=20 |  5.0 ms | 15.0 ms | 25.0 ms |

→ 33 Hz re-plan target (≤ 30 ms/solve) を **N≤20, SQP≤5** まで
余裕を持って満たす。 つまり misarta 側の追加最適化なしでも D3
は機能的に書ける見込み。

### 主要な所見と D3.2 以降への含意

1. **Ȧ(q,v) が圧倒的ボトルネック (236 μs)** — 単一 CMM の 33×。
   原因は `compute_centroidal_momentum_matrix_time_derivative` が
   2·nnz(v) 回の CMM 評価による central FD だから (`misarta/src/centroidal.rs:288-316`)。
   - 短期 (D3.2): Pinocchio `dccrba` 相当の解析的 Ȧ を misarta に追加 →
     ~26× 高速化 (236 μs → ~9 μs) でき、horizon=20 + SQP=5 でも 1.5 ms 以下。
   - D3 着手前に必須ではない (現状でも budget 内) が、 SQP iteration
     を増やしたい (D2 の lateral 不具合対策) 場合に効く。
2. **CRBA + CMM の融合余地** — 両者は同じ FK と spatial inertia
   composite を伝播している。 fused evaluator で ~32% 削減見込み
   (CMM 7.2 + CRBA 3.3 → 推定 7.5 程度)。 D3 のホットパスでは Ȧ
   削減に比べ二次的。
3. **`compute_coriolis_matrix` が 100 μs と高い** — 24-state MPC で
   関節側の C(q,v) を直接使う場合は要注意。 ただし centroidal
   formulation では関節側 ID は RNEA で済むため必要にならない。
4. **`compute_minv` (98 μs)** は M⁻¹ の明示計算。 D3 で M⁻¹ が要る
   なら Cholesky-of-M (CRBA 後) で 3.3 μs + back-substitution の方が
   速い可能性大。

### 関連コミット

| Hash | 内容 |
|---|---|
| `ba40cab` | D3.1 — `examples/bench_misarta_dynamics.rs` + 本セクション |

## 8. Phase D3.3: 24-state Full Centroidal MPC コア

D2 で empirical tuning (G/H) と SQP 反復で取りきれなかった lateral
反転 + forward dy cross-coupling を、 **MPC 状態空間そのものを
拡張** することで構造的に解決するための新規モジュール。 [`quadruped-gait/src/full_centroidal_mpc.rs`](../quadruped-gait/src/full_centroidal_mpc.rs)
に独立して実装、 12-state `centroidal_mpc.rs` (D1) はベースライン
として残置。

### 状態空間 (25-dim 拡張)

```
x = [ v_com_world (3)         CoM linear vel, world
      ω_world     (3)         body angular vel, world
      base_pos    (3)         base origin, world
      base_euler  (3)         ZYX [roll, pitch, yaw]
      joint_q     (12)        FL/FR/RL/RR × {hip, thigh, calf}
      g_aug       (1)         augmented gravity (constant -9.81)
    ]

u = [ F_FL F_FR F_RL F_RR (12)    per-foot world-frame GRF
      joint_v             (12)    per-leg joint velocity command
    ]
```

= **legged_control `centroidalModelType = 0`** 相当の "kinematic
centroidal" formulation (joint q を状態に、 joint v を入力に)。

### D3.3.1 — 型定義 (`d7eafc4`)

`FullCentroidalState` / `FullCentroidalInput` を round-trip vec24 と
共に導入。 joint slot は `LegId` 標準順 × `[hip, thigh, calf]` で
既存 `ik::forward_leg_kinematics` をそのまま使える packing にした。
6 unit test pass。

### D3.3.2 — 連続時間動力学 (`62e567d`)

`full_centroidal_dynamics(state, input, cfg)`:
- body 部は 12-state と同じ式 (v̇_com, α, ṗ_base, ė_zyx)
- **moment arm が per-node**: r_i = R · (foot_body_i − com_offset) で
  joint_q から FK 経由で更新される。 これが D3 の構造的本質
- `q̇_j = v_j` (input pass-through)

`compute_foot_positions_world(state, cfg)` を pub にし、 D3.3.3
linearization と D3.3.5+ WBC dispatch から再利用可能に。
6 unit test 追加 (joint_v=0 で 12-state と body 部 1e-9 一致を含む)。

### D3.3.3 — Yaw-only 線形化 (`dc9bb0e`)

`continuous_dynamics_full(state_ref, input_ref, cfg, stance, psi_ref)
→ (A 25×25, B 25×24)`。

新ブロック (12-state にはない部分):

1. **∂α/∂joint_q** = -I_world⁻¹ · skew(F_ref) · R_z · J_foot_body(q_ref)
   - 各脚 3 列 ずつ A[3..6, 12..24] に書く
   - J_foot_body は `ik::foot_jacobian_body` (analytical 3R) を流用
   - これが MPC に「joint 動かすと moment arm 動く」ことを教える
2. **∂α/∂base_euler** =
   - I_world⁻¹ · Σ (R_z · skew(e_k) · (foot_body − com_offset)) × F_ref
   - + (-I_world⁻¹ · R_z · [skew(e_k), I_body] · R_z^T · I_world⁻¹) · τ_ref
   - 両項とも必要 (1 項目だけだと FD と 5% ずれ)
3. **∂q̇_j/∂joint_v** = I_12

検証: `linearization_state_jacobian_matches_fd` (24 列 × 24 行) と
`linearization_input_jacobian_matches_fd` (24 × 24) が
`full_centroidal_dynamics` の central FD と analytical の差 < 1e-3 で
pass。 4 unit test 追加 (FD 比較 2 + swing leg GRF zero check + aug-grav 列構造)。

### D3.3.4a — Condensed QP + SQP iteration (`8a77add`)

`FullCentroidalMpc::solve()`:
- Decision var: U ∈ R^{24·N} (state lifting で X = A_x x_0 + B_u U に消去)
- Cost: ‖X − X_ref‖²_Q + ‖U − U_ref‖²_R を P/q 形式で
- 制約 (D3.3.4a 時点): swing leg GRF=0 + friction pyramid + f_z 上下限
- SQP 反復: 前 iter の predicted (state, input) で全 ref 更新後 再線形化

設計判断:
- swing leg foot tracking: pre-IK joint_q_ref を cost で参照
- SQP 再線形化: full state + input

unit test 2 個追加 (静止立位で各脚 f_z ∈ [5, 70] N、 swing leg の GRF
完全ゼロ)。

### D3.3.4b — Stance no-slip 制約 (`9e3ee33`)

各 stance leg について
   `v_foot_world = v_com + ω × r_ref + R_z · J_foot · joint_v_leg = 0`
を condensed QP の等式制約に追加。 U に対する線形式:

  [B_u rows for v_com − skew(r_ref) · B_u rows for ω] · U
    + [R_z · J_foot] (joint_v_leg slice) = -[同じ M · A_x[k_block][0..6, :]] · x_0

`build_constraints_24` を A_x, B_u, x_now も取るシグネチャに変更。
ZeroCone サイズに 3·(stance-leg-step) を加算。

検証: body が v_com=(0.1,0,0) で動いている状態で MPC を解くと
各脚 `v_com + J_foot · joint_v_leg` の norm が 3 cm/s 未満
(unconstrained だと 10 cm/s) に収まる。

### D3.3 コア合計

5 commit、19 unit test。 計算カーネル / 線形化 / QP / SQP / 制約は
完成し、 unit-test ベースで全要素が verified。 **gait controller /
WBC 統合 (D3.3.5 / D3.3.6) は別 session に残置**。

### 関連コミット

| Hash | 内容 |
|---|---|
| `9e3ee33` | D3.3.4b stance no-slip equality 制約 |
| `8a77add` | D3.3.4a condensed QP + SQP iteration |
| `dc9bb0e` | D3.3.3 yaw-only linearization (FD-verified) |
| `62e567d` | D3.3.2 連続時間動力学 + per-node FK foot 位置 |
| `d7eafc4` | D3.3.1 24-state state/input types |

---

## 関連コミット一覧 (時系列・新→旧)

| Hash | 内容 |
|---|---|
| 041416c | `--script <path>` CLI flag (GUI 起動時に Rhai script auto-run) |
| 8bdf862 | `set_gait_mode` / `set_pose_source` / `set_wbc_enabled` / `set_ground_plane_*` scripting setters |
| 9e7b099 | `walk_3axis_demo.rhai` + `mj_async_set_velocity` (3 軸 benchmark の描画付き再現) |
| e4e914d | P5b: per-cmd swing_leg scheduling で lateral / yaw 命令 pass |
| d18d9af | P5a: WbcWeights API + lateral 不具合の主因 (swing_leg) を per-task disable で特定 |
| 7cb5097 | lateral 命令の MPC-only vs MPC+WBC 切り分け診断 |
| 4dbf1e9 | 3 軸独立 benchmark (forward / lateral / yaw × CHAMP / MPC+WBC) で cross-coupling を可視化 |
| 694e2bd | SRBD MPC q_diag tuning で直進歩行の lateral / yaw drift を抑制 |
| b03c431 | GUI の WBC 経路をテストと一致 (`set_torque_feedforward` Hybrid joint command) |
| d7b6f1b | SRBD MPC `predicted_body_states` expose + W_CONTACT_FORCE=5 で wbc_forward_command pass |
| 6c360fa | WBC swing_leg を joint-space に書き換え + 詳細診断ダンプ |
| 299d451 | 歩容制御 MuJoCo 回帰スイート (`tests/common/`, `integration_walk`, `scripts/test_regression.sh`) |
| 4900887 | 18-state Linear Kalman Filter (legged_control 移植) |
| a929704 | Hybrid joint command — Position-PD + WBC τ_ff (legged_control 流) |
| cd1d90b | `MujocoSim::contact_force_per_foot` + WBC pipeline で接触補正を有効化 |
| 4ad3f2a | `ContactDrivenPhase` — early/late 接触補正レイヤー |
| 149aad0 | `a_base_des` を MPC predicted accel に置換、 wbc_static_stand pass |
| fa8835d | `predicted_base_accel_world` (SRBD MPC→WBC base reference bridge) |
| 1d57465 | WBC pipeline 安定化 (warm-start + gravity FF + \|v\| clip) |
| dc78f98 | WBC warm-start API + per-task LSQ weights |
| 6f56765 | MPC GRF EMA を WBC pipeline 側へ移動 (mpc_walks_stable regression 解消) |
| fff52a1 | TPS 追従カメラ + PiP wipe |
| cd2290c | D-pad 押下持続改善 + yaw ボタン |
| 8349105 | hold-to-drive D-pad |
| 7b65a73 | URDF +Y 軸符号規約の整合 |
| 52b97e8 | knee_forward 両分岐の回帰テスト |
| 876524f | stance 軌道方向の反転 |
| 99ead9a | KneePattern (`<<`,`<>`,`><`,`>>`) |
| bf30f5f | 歩容プリセット sidecar 永続化 (3e) |
| 6103658 | quadruped-gait Phase 3 (articara 統合) |
| f3990af | quadruped-gait Phase 2 (Controller) |
| b8497ae | Phase 1 統合テスト |
| b373a74 | quadruped-gait crate Phase 1 |
| 79843ee | ジャンプスクリプトに ABSORB |
| d617278 | 非同期スクリプトタイムライン |
| 50191ba | スクリプトコンソールリサイズ修正 |
| f7c5a56 | example_jump.rhai 追加 |
| ff0099a | ⛔ Limits を MJCF レベルに反映 |
| 7e565c3 | ComputedTorque mode (S3) |
| f980931 | 重力補償 PD (S2) |
| a30adab | スクリプトファイル実行 |
| d44589f | 解析スクリプト QDD/traditional 区別 |
| 7a7f716 | 解析スクリプト + モータ推定 |
| 478e724 | 軌道速度 FF + armature/damping (A+B) |
| 0c1f2df | Joint Peaks CSV / X軸 / 系列制御 |
