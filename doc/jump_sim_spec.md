# Jump Simulation — 制御系仕様書

## 1. 概要

ジャンプシミュレーションは、4脚ロボット (namiashi) の伸展→飛行→着地を
Featherstone 空間代数ベースの順動力学 (Forward Dynamics) で再現する。
関節制御には **Computed-Torque + PD + 足先 Null-Space 拘束** を採用し、
Extension / Flight の両フェーズで同一の `fd.step()` を継続呼出しする。

```
fd.step(model, target_angles, dt)
  ├── trajectory 非空 → step_pd()       [Computed-Torque + Foot-X 拘束]
  └── trajectory 空   → step_nullspace() [Null-Space 速度制御 (フォールバック)]
```

---

## 2. 軌道生成 (Feedforward)

### 2.1 JointTrajectoryPoint

各関節に対してコサイン族のプロファイルを事前計算する。
`evaluate(t)` は $(q_{des},\, \dot{q}_{des},\, \ddot{q}_{des})$ を返す。
$t \ge T$ では $(q_{end}, 0, 0)$ を返し、PD コントローラは位置保持に移行する。

### 2.2 TrajectoryProfile

| プロファイル | 数式 | 特徴 |
|---|---|---|
| **Launch** | $q(t) = q_0 + \Delta q\,(1 - \cos\frac{\pi t}{2T})$ | 開始: $\dot{q}=0$、終了: $\dot{q}=\frac{\pi\Delta q}{2T}$ (最大) |
| **Symmetric** | $q(t) = q_0 + \frac{\Delta q}{2}(1 - \cos\frac{\pi t}{T})$ | 開始: $\dot{q}=0$、終了: $\dot{q}=0$ |

- **Extension** フェーズ: Launch（ジャンプ打ち上げ速度最大化）
- **Flight retract**: Symmetric（滑らかに元の姿勢へ）

---

## 3. メインコントローラ: `step_pd()`

### 3.1 関節空間 PD 加速度指令

$$a_{pd,i} = \ddot{q}_{des,i} + K_p\,(q_{des,i} - q_i) + K_d\,(\dot{q}_{des,i} - \dot{q}_i)$$

理想的な閉ループ誤差ダイナミクス:

$$\ddot{e} + K_d\,\dot{e} + K_p\,e = 0$$

### 3.2 足先 X 方向 Null-Space 射影 (接地時のみ)

足先が X 方向に滑らないよう、PD 加速度を null-space に射影し、
足先位置フィードバックを加える。

$$a_{cmd} = N \cdot a_{pd} + J_x^+\,\bigl(-K_{fb}\,\Delta x - K_{dfb}\,\dot{x}\bigr)$$

| 記号 | 定義 |
|---|---|
| $J_x$ | 足先 X 方向 Jacobian ($n_{feet} \times n$) |
| $J_x^+$ | $J_x$ の正則化擬似逆行列 $J_x^T(J_x J_x^T + \varepsilon I)^{-1}$ |
| $N$ | Null-Space 射影行列 $I - J_x^+ J_x$ |
| $\Delta x$ | 足先 X 位置ドリフト (初期位置からの偏差) |
| $\dot{x}$ | 足先 X 方向速度 ($J_x \dot{q}$) |

**接地なし (Flight)**: $a_{cmd} = a_{pd}$ (pure PD, 無拘束)

### 3.3 Computed Torque

$$\tau = M(q) \cdot a_{cmd} + h(q, \dot{q})$$

- $M(q)$: CRBA (Composite Rigid Body Algorithm)
- $h(q, \dot{q})$: RNEA (Recursive Newton-Euler Algorithm) — Coriolis + 重力
- URDF effort limit でクランプ

### 3.4 順動力学 → 積分

$$\ddot{q} = M^{-1}(\tau - h)$$

Semi-implicit Euler:

$$\dot{q}_{k+1} = \dot{q}_k + \ddot{q}\,\Delta t$$
$$q_{k+1} = q_k + \dot{q}_{k+1}\,\Delta t$$

URDF の速度リミット・関節リミットでクランプ。

---

## 4. フォールバック: `step_nullspace()`

軌道未設定時 (`trajectory` 空) のレガシーパス。

$$\dot{q}_{des} = N \cdot K_{ext}(q_{target} - q) + J_x^+(-K_{fb}\,\Delta x)$$

$$\tau = M \cdot \frac{\dot{q}_{des} - \dot{q}}{\Delta t} + h$$

- $K_{ext} = 30$ [1/s]
- 1 ステップで目標速度に到達するインパルス型トルク

---

## 5. シミュレーションフェーズ

### 5.1 フェーズ一覧

| フェーズ | 接地 | 軌道種別 | コントローラ | ベース Z |
|---|---|---|---|---|
| **Extension** | あり | Launch | `step_pd()` + foot-X | FK 足拘束 |
| **Flight** | なし | Symmetric (retract) or なし | `step_pd()` (無拘束) | 弾道 |
| **Landed** | — | — | FD 停止 | 静止 |

### 5.2 Extension フェーズ

1. 事前計算した Launch 軌道に沿って各関節を伸展
2. `step_pd()` + 足先 X null-space 拘束で足滑りを防止
3. ベース Z は FK 足拘束で算出:
   - ベースを $z=0$ に置いて FK → 足高さ算出
   - $z_{base} = z_{foot,init} - z_{foot@base0}$
4. 速度・GRF は有限差分で算出

### 5.3 Extension → Flight 遷移条件

以下のいずれかで発射:

- **(a)** 平滑化速度がピークから 50% 以下に低下
- **(b)** 伸展時間経過 **かつ** 全関節が目標の 95% 以上に到達
- **(c)** 安全タイムアウト: $3 \times$ extension\_duration

### 5.4 遷移時の処理

1. `contact_feet`, `foot_chains`, `initial_foot_x` をクリア
   → 以降 `step_pd()` は foot-X 拘束なし (pure PD)
2. 全関節速度を **0 にリセット**
   → 打ち上げ速度はベース弾道軌道に反映済み
3. Extension 軌道をクリア
4. Retract 有効時: Symmetric 軌道 (150 ms) を設定
   → PD computed-torque で伸展姿勢から保存姿勢へ滑らかに戻す

### 5.5 Flight フェーズ

- ベース: 弾道軌道 $z(t) = z_0 + v_0 t - \frac{1}{2}g t^2$
- 関節: 同一の `fd.step()` を継続 (無拘束 FD)
  - Retract 軌道あり → PD で姿勢復帰
  - 軌道なし → 重力のみの自然運動
- 着地判定: 足先 Z ≤ 初期足先 Z

### 5.6 Landed フェーズ

- FD 停止 (`fd_state = None`)
- 着地保持後、保存姿勢を復元してシミュレーション終了

---

## 6. 物理パラメータ

| パラメータ | デフォルト値 | 備考 |
|---|---|---|
| 物理タイムステップ | 0.5 ms | フレーム dt をサブステップ分割 |
| グラフ記録間隔 | 1 ms | |
| $K_p$ | 500 N·m/rad | UI から調整可 |
| $K_d$ | 20 N·m·s/rad | UI から調整可 |
| $K_{fb}$ (足先位置) | 200 1/s² | |
| $K_{dfb}$ (足先速度) | 30 1/s | |
| $K_{ext}$ (null-space) | 30 1/s | |
| $\varepsilon$ (正則化) | $10^{-6}$ | $J_x J_x^T$ の正則化 |
| Retract duration | 150 ms | Symmetric プロファイル |
| Effort limit | URDF 値 | トルククランプ |
| Velocity limit | URDF 値 | 速度クランプ |
| Joint limit | URDF 上下限 | 位置クランプ |

---

## 7. UI 操作

| 操作 | 説明 |
|---|---|
| 🦘 Jump | シミュレーション準備（一時停止状態で開始） |
| ▶ Play / ⏸ Pause | 実時間再生 / 一時停止 |
| ⏭ 1ms / 10ms / 100ms | 指定時間分だけステップ実行 |
| Ext dur | 伸展時間 (Auto or 手動指定) |
| Kp / Kd | PD ゲイン調整 |
| Enforce torque limits | URDF effort limit の適用 |
| Retract after extend | Flight 中の姿勢復帰 |
| Launch axes | 飛行中にベースが動ける軸 |
| Locked joints | 駆動しない関節の指定 |
| Graph link | 位置/速度/加速度グラフの対象リンク |

---

## 8. WASM プラグイン I/F 仕様

汎用コマンドディスパッチ + **View プリミティブ** によるホスト描画方式。
新しいコマンドを追加してもホスト側の修正は不要。

### 8.1 アーキテクチャ

```
┌──────────────────┐                 ┌──────────────────────────┐
│   Host (native)  │                 │  WASM Module (sandbox)   │
│  jump-sim-runner │                 │  jump_sim_wasm.wasm      │
│                  │                 │                          │
│  Request {       │                 │  execute(ptr, len)       │
│    version: 1,   │── JSON ──────> │    ├─ dispatch(command)   │
│    command: "…",  │                │    │   ├─ jump_sim        │
│    params: {…}   │                │    │   ├─ static_analysis │
│  }               │                │    │   ├─ gravity_torques │
│                  │                 │    │   ├─ payload_capacity│
│  Response {      │                 │    │   ├─ jump_height     │
│    ok, views,    │<── JSON ──────│    │   ├─ payload_sim     │
│    data          │                │    │   └─ list_commands   │
│  }               │                │    └─ → Response          │
│                  │                 │                          │
│  render_views()  │                 │  (GUIなし / serde only)   │
│  ├─ Heading      │                 └──────────────────────────┘
│  ├─ Scalars      │
│  ├─ Table        │
│  ├─ LinePlot     │
│  ├─ BarChart     │
│  ├─ Progress     │
│  └─ Log          │
└──────────────────┘
```

- WASM ターゲット: `wasm32-unknown-unknown`
- ランタイム: wasmtime 33 (ホスト側)
- シリアライズ: serde_json (JSON)
- WASM モジュールは GUI 依存なし (`default-features = false`)
- 共有型定義: `plugin-api` クレート (ホスト・WASM 双方が依存)

### 8.2 Export 関数一覧

| 関数 | シグネチャ | 用途 |
|---|---|---|
| `alloc` | `(size: u32) → u32` | WASM 線形メモリにバッファを確保しポインタを返す |
| `dealloc` | `(ptr: u32, size: u32) → ()` | `alloc` で確保したバッファを解放 |
| `execute` | `(ptr: u32, len: u32) → u32` | **汎用エントリポイント**: JSON Request → dispatch → Response。戻り値: 0=成功, 1=エラー |
| `last_output_ptr` | `() → u32` | 直前の出力 JSON バイト列のポインタ |
| `last_output_len` | `() → u32` | 直前の出力 JSON バイト列の長さ |
| `run_jump_sim` | `(ptr: u32, len: u32) → u32` | *後方互換*: 旧ホスト向け。内部で `execute` に委譲 |
| `memory` | *(Memory export)* | WASM 線形メモリ（読み書き用） |

### 8.3 呼び出しプロトコル

```
Host                                WASM
 │                                   │
 │  1. alloc(json_len) ────────────> │  → input_ptr
 │  2. memory.write(input_ptr, json) ─> │
 │  3. execute(input_ptr, len) ─────> │  → 0 (成功) or 1 (エラー)
 │  4. last_output_len() ────────────> │  → out_len
 │  5. last_output_ptr() ────────────> │  → out_ptr
 │  6. memory.read(out_ptr, out_len) ─> │  → Response JSON bytes
 │  7. dealloc(input_ptr, json_len) ──> │
 │                                   │
```

**注意事項**:
- 出力バッファは WASM 側の `static` 領域が所有。次回 `execute` 呼び出しで上書きされる。
- `dealloc` は入力バッファのみ呼び出す。出力バッファの解放はモジュール側が管理。

### 8.4 Request / Response エンベロープ

#### Request

```json
{
  "version": 1,
  "command": "jump_sim",
  "params": { … }
}
```

| フィールド | 型 | 説明 |
|---|---|---|
| `version` | `u32` | プロトコルバージョン (現在 `1`) |
| `command` | `String` | コマンド名 |
| `params` | `Value` | コマンド固有のパラメータ (JSON object) |

#### Response (成功時)

```json
{
  "version": 1,
  "ok": true,
  "command": "jump_sim",
  "views": [ … ],
  "data": { … }
}
```

#### Response (エラー時)

```json
{
  "version": 1,
  "ok": false,
  "command": "jump_sim",
  "error": "JSON parse error: …"
}
```

| フィールド | 型 | 説明 |
|---|---|---|
| `version` | `u32` | プロトコルバージョン |
| `ok` | `bool` | 成功/失敗 |
| `command` | `String` | コマンド名のエコー |
| `error` | `String?` | エラーメッセージ (`ok=false` 時) |
| `views` | `[View]?` | ホストが描画する **View プリミティブ** の順序付きリスト |
| `data` | `Value?` | 機械可読な生データ (プログラム向け) |

### 8.5 View プリミティブ

ホスト側は `views` 配列を上から順にレンダリングする。
新しい View 型の追加にはホスト更新が必要だが、
**新しいコマンドの追加はホスト変更不要** （既存の View 型を組み合わせるだけ）。

| View 型 | 用途 | 主なフィールド |
|---|---|---|
| `Heading` | セクション見出し | `text`, `level` (1=大, 2=中…) |
| `Scalars` | キー・値のリスト | `title?`, `items: [{label, value, numeric?, emphasis?}]` |
| `Table` | 列定義+行データ | `title?`, `columns: [{name, align?}]`, `rows: [[Cell]]` |
| `LinePlot` | 時系列折れ線グラフ | `title`, `x_label`, `y_label`, `series: [{name, x, y, color?}]` |
| `BarChart` | 棒グラフ | `title`, `bars: [{label, value, color?, tag?}]`, `max_value?` |
| `Progress` | プログレスバー | `label`, `value` (0–1), `text?` |
| `Log` | メッセージブロック | `messages: [{level, text}]` |

#### Cell 型 (Table 内)

| Cell 型 | 説明 |
|---|---|
| `Text` | プレーンテキスト (`value: String`) |
| `Number` | 数値 (`value: f64`, `format?: String` — printf 形式) |
| `Tag` | 色付きバッジ (`value: String`, `color?: "green"\|"yellow"\|"red"\|"gray"`) |

### 8.6 コマンド一覧

| コマンド | カテゴリ | 説明 | 必須 params |
|---|---|---|---|
| `list_commands` | meta | 利用可能コマンドの列挙 | なし |
| `jump_sim` | simulation | 完全ジャンプシミュレーション | `model`, `ground_links`, … (§8.7 参照) |
| `static_analysis` | analysis | 重力トルク + 可搬質量 + 推定跳躍高 | `model` |
| `gravity_torques` | analysis | 関節ごとの静的重力トルク | `model` |
| `payload_capacity` | analysis | エンドエフェクタでの最大可搬質量 | `model`, `ee_link` |
| `jump_height` | analysis | エネルギー法に基づく跳躍高推定 | `model` |
| `payload_sim` | simulation | ペイロード漸増シミュレーション | `model`, `ee_link` |

### 8.7 コマンド固有 params

#### `jump_sim`

```json
{
  "model": { … },
  "ground_links": ["RL_foot", "FL_foot", "RR_foot", "FR_foot"],
  "body_link": "trunk",
  "speed": 1.0,
  "locked_joints": [],
  "launch_axes": [false, false, true],
  "extension_duration": null,
  "enforce_torque_limits": false,
  "enable_retract": true,
  "graph_link": "trunk",
  "pd_kp": 500.0,
  "pd_kd": 20.0
}
```

| フィールド | 型 | 必須 | 説明 |
|---|---|---|---|
| `model` | `RobotModel` | ✓ | URDF から解析済みのロボットモデル |
| `ground_links` | `[String]` | ✓ | 接地リンク名リスト |
| `body_link` | `String?` | | 胴体リンク名。`null` → ルートリンク |
| `speed` | `f32` | ✓ | シミュレーション速度倍率 |
| `locked_joints` | `[String]` | ✓ | ジャンプ中にロックする関節名セット |
| `launch_axes` | `[bool; 3]` | ✓ | 飛行中のベース並進軸 `[x, y, z]` |
| `extension_duration` | `f32?` | | 伸展時間 (秒)。`null` → 自動計算 |
| `enforce_torque_limits` | `bool` | ✓ | URDF effort limit の適用 |
| `enable_retract` | `bool` | ✓ | 飛行中の脚引き戻し |
| `graph_link` | `String?` | | グラフ記録対象リンク。`null` → 記録なし |
| `pd_kp` | `f64` | ✓ | PD 位置ゲイン (N·m/rad) |
| `pd_kd` | `f64` | ✓ | PD 速度ゲイン (N·m·s/rad) |

#### `gravity_torques` / `static_analysis` / `jump_height`

```json
{ "model": { … } }
```

#### `payload_capacity` / `payload_sim`

```json
{ "model": { … }, "ee_link": "FL_foot" }
```

### 8.8 Cargo Feature フラグ

| Feature | 対象 | 内容 |
|---|---|---|
| `gui` (default) | articara | eframe, egui_plot, glow, env_logger |
| `serde` | articara | serde derives + nalgebra/serde-serialize |

WASM クレートは `default-features = false, features = ["serde"]` で GUI 非依存にする。

### 8.9 ビルドコマンド

```bash
# WASM モジュール (最適化ビルド: LTO + opt-level=z)
cargo build -p jump-sim-wasm --target wasm32-unknown-unknown --profile wasm-release

# WASM モジュール (通常リリース → ~569 KB)
cargo build -p jump-sim-wasm --target wasm32-unknown-unknown --release

# ホストランナー
cargo run -p jump-sim-runner --release -- <command> <urdf_path> [--wasm <wasm_path>] [--ee-link <link>]

# 実行例
target/release/jump-sim-runner list_commands sample/namiashi_description/urdf/namiashi.urdf
target/release/jump-sim-runner gravity_torques sample/namiashi_description/urdf/namiashi.urdf
target/release/jump-sim-runner payload_capacity sample/namiashi_description/urdf/namiashi.urdf --ee-link FL_foot

# テスト (serde ラウンドトリップ含む)
cargo test -p articara --features serde
```

### 8.10 ファイル構成

| パス | 内容 |
|---|---|
| `plugin-api/Cargo.toml` | 共有プラグイン API クレート設定 |
| `plugin-api/src/lib.rs` | Request/Response エンベロープ、View プリミティブ (7型)、CommandInfo |
| `jump-sim-wasm/Cargo.toml` | WASM cdylib クレート設定 |
| `jump-sim-wasm/src/lib.rs` | `execute` エントリポイント、コマンドディスパッチ、7 コマンドのハンドラ |
| `jump-sim-runner/Cargo.toml` | wasmtime ホストランナー設定 |
| `jump-sim-runner/src/main.rs` | 汎用 CLI ホスト: WASM ロード → Request 構築 → View レンダリング |

---

## 9. 主要ソースファイル

| ファイル | 内容 |
|---|---|
| `src/rbd/dynamics.rs` | CRBA, RNEA, FD, `JointTrajectoryPoint`, `ForwardDynamicsState`, `step_pd()`, `step_nullspace()` |
| `src/dynamics.rs` | `JumpSim`, `start_jump_sim()`, `step_jump_sim()`, `step_jump_sub()`, グラフ記録 |
| `src/app/dynamics_panel.rs` | UI パネル、結果表示、グラフ描画、SimConfig 保存/読込 |
| `src/app/mod.rs` | アプリ状態 (`dynamics_pd_kp`, `dynamics_pd_kd` 等) |
| `plugin-api/src/lib.rs` | 共有プロトコル型: View enum, Request/Response, Cell, Series, Bar 等 |
| `jump-sim-wasm/src/lib.rs` | WASM プラグイン: `execute` + command dispatch + 7 ハンドラ |
| `jump-sim-runner/src/main.rs` | wasmtime ホスト: 汎用 CLI, `call_plugin()`, `render_views()` |
