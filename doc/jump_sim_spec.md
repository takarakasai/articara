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

## 8. 主要ソースファイル

| ファイル | 内容 |
|---|---|
| `src/rbd/dynamics.rs` | CRBA, RNEA, FD, `JointTrajectoryPoint`, `ForwardDynamicsState`, `step_pd()`, `step_nullspace()` |
| `src/dynamics.rs` | `JumpSim`, `start_jump_sim()`, `step_jump_sim()`, `step_jump_sub()`, グラフ記録 |
| `src/app/dynamics_panel.rs` | UI パネル、結果表示、グラフ描画、SimConfig 保存/読込 |
| `src/app/mod.rs` | アプリ状態 (`dynamics_pd_kp`, `dynamics_pd_kd` 等) |
