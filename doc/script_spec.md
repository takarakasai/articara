# Articara Script Specification

Articara のスクリプトコンソールは **Rhai** スクリプト言語を採用し、
ロボットモデルの操作・解析・エクスポートをインタラクティブまたはバッチで実行できる。

---

## 1. 現行 API（実装済み）

### 1.1 モデル読み込み

| 関数 | 戻り値 | 概要 |
|---|---|---|
| `load(path)` | `bool` | URDF / SDF / MJCF / USD を読み込む |
| `model_name()` | `String` | モデル名 |
| `has_model()` | `bool` | モデルがロードされているか |

### 1.2 リンク情報

| 関数 | 戻り値 | 概要 |
|---|---|---|
| `link_names()` | `Array<String>` | 全リンク名 |
| `num_links()` | `i64` | リンク数 |
| `link_pos(name)` | `[x, y, z]` | ワールド座標位置 |
| `link_rpy(name)` | `[r, p, y]` | オイラー角姿勢 |

### 1.3 ジョイント操作

| 関数 | 戻り値 | 概要 |
|---|---|---|
| `joint_names()` | `Array<String>` | 全ジョイント名 |
| `num_joints()` | `i64` | ジョイント数 |
| `joint_pos(name)` | `f64` | 名前で位置取得 |
| `joint_pos_idx(i)` | `f64` | インデックスで位置取得 |
| `joint_positions()` | `Array<f64>` | 全ジョイント位置 |
| `set_joint(name, val)` | − | 名前で位置設定 |
| `set_joint_idx(i, val)` | − | インデックスで位置設定 |
| `set_joints(array)` | − | 全ジョイントを一括設定 |
| `joint_limits(name)` | `[lower, upper]` | リミット |
| `joint_type(name)` | `String` | タイプ文字列 |

### 1.4 運動学

| 関数 | 戻り値 | 概要 |
|---|---|---|
| `fk()` | `Map{name: [x,y,z]}` | 全リンクFK結果 |
| `ik(link, x, y, z)` | `bool` | 10ステップIK |
| `ik_steps(link, x, y, z, n)` | `f64` | nステップIK → 残差 |

### 1.5 拘束条件

| 関数 | 戻り値 | 概要 |
|---|---|---|
| `add_loop_closure(name, linkA, ox,oy,oz, linkB, ox,oy,oz)` | − | 閉ループ拘束追加 |
| `loop_closure_error()` | `f64` | 拘束誤差 |
| `num_loop_closures()` | `i64` | 拘束数 |

### 1.6 エクスポート

| 関数 | 戻り値 | 概要 |
|---|---|---|
| `export_urdf(path)` | `bool` | URDF出力 |
| `export_sdf(path)` | `bool` | SDF出力 |
| `export_mjcf(path)` | `bool` | MJCF出力 |

### 1.7 メッシュ削減

`method` は `"qem"` | `"edge"` | `"cluster"` のいずれか。

| 関数 | 戻り値 | 概要 |
|---|---|---|
| `reduce_mesh(link, vi, ratio)` | `i64` | Visual メッシュを QEM で削減 |
| `reduce_mesh(link, vi, ratio, method)` | `i64` | アルゴリズム指定 |
| `reduce_collision_mesh(link, ci, ratio)` | `i64` | Collision メッシュ削減 |
| `reduce_collision_mesh(link, ci, ratio, method)` | `i64` | アルゴリズム指定 |
| `reduce_all_meshes(ratio)` | `i64` | 全メッシュ一括削減 |
| `reduce_all_meshes(ratio, method)` | `i64` | アルゴリズム指定 |

### 1.8 メッシュ分解

| 関数 | 戻り値 | 概要 |
|---|---|---|
| `decompose_vhacd(link, ci)` | `i64` | V-HACD 凸分解（デフォルト設定） |
| `decompose_vhacd(link, ci, max_hulls)` | `i64` | ハル数上限指定 |
| `decompose_spheres(link, ci)` | `i64` | Sphere Tree 分解 |
| `decompose_spheres(link, ci, max_n)` | `i64` | 球数上限指定 |

### 1.9 数学関数

| 関数 | 概要 |
|---|---|
| `sin(x)` `cos(x)` `sqrt(x)` `abs(x)` | 基本関数 |
| `atan2(y, x)` | 逆正接 |
| `min_f(a, b)` `max_f(a, b)` | 最小・最大 |
| `clamp(x, lo, hi)` | クランプ |
| `to_deg(x)` `to_rad(x)` | 角度変換 |
| `PI()` | 円周率 |
| `dist(ax,ay,az, bx,by,bz)` | 3D距離 |

### 1.10 コンソール

| コマンド | 概要 |
|---|---|
| `clear` | 出力クリア |
| `help` / `help()` | ヘルプ表示 |
| `↑` / `↓` | コマンド履歴 |
| `Tab` | 補完 |

### 1.11 MuJoCo シミュレーション (mujoco feature)

| 関数 | 戻り値 | 概要 |
|---|---|---|
| `mj_start()` / `mj_stop()` | `bool` | sim build / teardown |
| `mj_active()` | `bool` | sim 起動中か |
| `mj_step(n)` / `mj_step_seconds(s)` | `i64` | 同期ステップ |
| `mj_set_trace_max(n)` | `bool` | sample buffer 上限 |
| `mj_gravity_compensation(on)` | `i64` | gravity comp トグル |
| `set_actuator_mode_all(s)` | — | "position" / "velocity" / "torque" / "computedtorque" |
| `set_kp_all(v)` / `set_kv_all(v)` | — | Position-PD ゲイン |
| `set_armature_all(v)` / `set_joint_damping_all(v)` | — | actuator dynamics |

#### 非同期タイムライン (UI viewport で再生)

| 関数 | 概要 |
|---|---|
| `mj_async_step_seconds(s)` | 指定秒数だけ非同期に進める |
| `mj_async_step_frames(n)` | 指定フレーム数だけ非同期に進める |
| `mj_async_set_position_target(joint, q)` | timeline 上で position target を変更 |
| `mj_async_set_velocity(vx, vy, wz)` | timeline 上で gait controller の cmd を変更 |
| `mj_async_print(msg)` / `mj_async_save_csv(path)` | timeline 上の echo / CSV save |
| `mj_async_pending()` | キューに残っている op 数 |
| `mj_async_clear()` | 未消化 op を破棄 |

### 1.12 Quadruped gait controller

| 関数 | 戻り値 | 概要 |
|---|---|---|
| `gait_setup()` / `gait_setup_with_feet(fl, fr, rl, rr)` | `bool` | URDF から自動検出して controller 構築 |
| `gait_start()` / `gait_stop()` | `bool` | 制御開始 / 停止 |
| `gait_set_velocity(vx, vy, wz)` | `bool` | 同期 cmd 設定 (body frame) |
| `gait_set_cycle_period(s)` / `gait_set_swing_height(m)` / `gait_set_duty(d)` | `bool` | 歩容パラメータ |
| `gait_set_max_step(m)` / `gait_set_knee_pattern(s)` | `bool` | 歩幅 / 膝パターン |
| `gait_active()` / `gait_running()` | `bool` | 状態クエリ |

### 1.13 GUI 設定 (ScriptOverrides)

スクリプトから ArticaraApp の主要 GUI 設定を変更できる setter。
スクリプト終了後の最初のフレームで host が apply する。

| 関数 | 引数 | 効果 |
|---|---|---|
| `set_gait_mode(s)` | `"mpc"` / `"champ"` (case-insensitive) | Generator (gait controller mode) を切替 |
| `set_pose_source(s)` | `"groundtruth"` / `"imu_madgwick"` / `"leg_odometry"` | Pose source を切替 |
| `set_wbc_enabled(on)` | `bool` | Hierarchical WBC ON/OFF |
| `set_ground_plane_enabled(on)` | `bool` | Ground plane の include/exclude (次回 mj_start で有効) |
| `set_ground_plane_z(z)` | f64 (m) | Ground plane height |
| `set_ground_plane_pitch(p)` | f64 (rad) | Ground plane pitch |
| `set_ground_plane_roll(r)` | f64 (rad) | Ground plane roll |

これらは `ScriptOverrides` 構造体に Option<> field として queue され、
スクリプト終了後に `ArticaraApp::apply_script_overrides` で反映される。
将来の他の設定 (camera angle, viewport overlays 等) も同じパターンで追加
可能。

---

## 2. 拡張ロードマップ（未実装）

### Phase 1: 動力学クエリ

読み取り専用で副作用なし。既存 `rbd::dynamics` を薄くラップ。

| 関数 | 戻り値 | 概要 |
|---|---|---|
| `total_mass()` | `f64` | モデル総質量 |
| `com()` | `[x, y, z]` | 重心位置 |
| `gravity_torques()` | `Map{joint: f64}` | 各ジョイントの重力トルク |
| `mass_matrix()` | `Array<Array<f64>>` | 関節空間慣性行列 M(q) |
| `inverse_dynamics(qdd)` | `Array<f64>` | RNEA 逆動力学 |
| `payload_capacity(link)` | `f64` | 指定リンク先端のペイロード上限 |
| `jump_height()` | `f64` | 推定ジャンプ高さ |

**優先度**: ★★★（最優先）  
**理由**: GUI の Dynamics Panel 機能が完全に未公開。計算コストが低く安全。パラメータスタディの自動化に直結。

---

### Phase 2: モデル構造編集

スクリプトからロボットをゼロ構築・バッチ編集するための操作群。

| 関数 | 戻り値 | 概要 |
|---|---|---|
| `new_model(name)` | `bool` | 空モデル作成 |
| `add_link(name)` | `bool` | リンク追加 |
| `add_joint(name, type, parent, child)` | `bool` | ジョイント追加 |
| `remove_link(name)` | `bool` | リンク再帰削除 |
| `rename_link(old, new)` | `bool` | リンク名変更 |
| `set_inertial(link, mass, ixx, ixy, ixz, iyy, iyz, izz)` | − | 慣性パラメータ設定 |
| `auto_inertia(link, density)` | `bool` | ジオメトリから慣性自動推定 |
| `add_visual(link, type, ...)` | `i64` | Visual 追加 (box/sphere/cylinder/mesh) |
| `add_collision(link, type, ...)` | `i64` | Collision 追加 |
| `remove_visual(link, vi)` | `bool` | Visual 削除 |
| `remove_collision(link, ci)` | `bool` | Collision 削除 |
| `set_visual_color(link, vi, r, g, b, a)` | − | 色変更 |
| `set_visual_origin(link, vi, x,y,z, r,p,y)` | − | Visual 原点設定 |
| `set_collision_origin(link, ci, x,y,z, r,p,y)` | − | Collision 原点設定 |

**優先度**: ★★★  
**理由**: ロボットのプログラマティック構築が可能に。全リンク慣性一括計算など、バッチ処理の需要が高い。

---

### Phase 3: 衝突検出 & 高度な IK

干渉チェックとIKソルバ拡張。

| 関数 | 戻り値 | 概要 |
|---|---|---|
| `self_collision()` | `bool` | 自己干渉チェック |
| `collision_pairs()` | `Array<[String, String]>` | 干渉リンクペア |
| `min_distance()` | `f64` | 最小離間距離 |
| `ik_solver(method)` | − | IKソルバ切替 (`"dls"` / `"sr"` / `"jt"`) |
| `ik_root(link, root, x, y, z)` | `bool` | 部分チェーン IK |
| `ik_6dof(link, x,y,z, r,p,y)` | `bool` | 6自由度 IK（姿勢込み） |
| `multi_ik(pins)` | `f64` | 複数ピン同時 IK |
| `jacobian(link)` | `Array<Array<f64>>` | 位置ヤコビアン (3×N) |
| `jacobian_full(link)` | `Array<Array<f64>>` | フル Jacobian (6×N) |

**優先度**: ★★☆  
**理由**: 衝突回避付きモーション生成に必須。姿勢最適化ループで高度な IK が必要。

---

### Phase 4: カメラ・表示制御 & ポスチャ管理

GUI 状態のスクリプト制御。スクリーンショット自動化やデモ用途に有用。

| 関数 | 戻り値 | 概要 |
|---|---|---|
| `camera_set(yaw, pitch, dist)` | − | カメラ位置設定 |
| `camera_target(x, y, z)` | − | 注視点設定 |
| `camera_reset()` | − | カメラリセット |
| `display_mode(mode)` | − | 表示モード (`"solid"` / `"wire"` / `"transparent"` / `"off"`) |
| `show_collisions(flag)` | − | コリジョン表示切替 |
| `show_com(flag)` | − | CoM マーカー表示 |
| `show_axes(flag)` | − | ジョイント軸表示 |
| `save_posture(path)` | `bool` | ポスチャ TOML 保存 |
| `load_posture(path)` | `bool` | ポスチャ TOML 読込 |
| `set_base_transform(x,y,z, r,p,y)` | − | ベース座標変換設定 |

**優先度**: ★★☆  
**理由**: スクリプトからの画角制御はスクリーンショット自動化・教育デモに有用。ポスチャ保存/復元はワークフロー効率化に寄与。

---

### Phase 5: エクスポート拡充 & バッチワークフロー

CI/CD 品質チェックパイプライン構築を可能にする仕上げフェーズ。

| 関数 | 戻り値 | 概要 |
|---|---|---|
| `export_usda(path)` | `bool` | Isaac USD 形式エクスポート |
| `export_stl(link, vi, path)` | `bool` | 個別メッシュ STL 出力 |
| `validate_inertia()` | `Array` | 全リンクの慣性バリデーション |
| `decompose_visual_vhacd(link, vi)` | `i64` | Visual 側 V-HACD 分解 |
| `decompose_visual_spheres(link, vi)` | `i64` | Visual 側 Sphere 分解 |
| `save_config(path)` | `bool` | `.misarta.toml` 保存 |
| `sleep(ms)` | − | スクリプト内待機 |
| `timestamp()` | `f64` | 現在時刻（秒） |
| `screenshot(path)` | `bool` | スクリーンショット保存 |

**優先度**: ★☆☆  
**理由**: USD 出力は Isaac Sim 連携で需要大。バリデーション＋スクリーンショットで自動品質チェックが実現可能。

---

## 3. スクリプト例

### 3.1 現行 API での使用例

```rhai
// ロボットを読み込んで歩行ポーズを設定
load("namiashi.urdf");

// 全ジョイントをゼロにリセット
let n = num_joints();
let zeros = [];
for i in 0..n { zeros.push(0.0); }
set_joints(zeros);

// 右脚のIKで目標位置に到達
let err = ik_steps("right_foot", 0.05, -0.1, -0.3, 50);
print(`IK error: ${err}`);

// 各ジョイント位置を表示
let names = joint_names();
let pos = joint_positions();
for i in 0..names.len() {
    print(`${names[i]}: ${to_deg(pos[i])} deg`);
}

// URDF にエクスポート
export_urdf("/tmp/namiashi_modified.urdf");
```

### 3.2 Phase 1 拡張後の使用例

```rhai
// 動力学解析パイプライン
load("namiashi.urdf");

let m = total_mass();
print(`Total mass: ${m} kg`);

let c = com();
print(`CoM: [${c[0]}, ${c[1]}, ${c[2]}]`);

// 現在姿勢での重力トルク
let tau = gravity_torques();
for name in joint_names() {
    print(`${name}: ${tau[name]} Nm`);
}

// ペイロード解析
let payload = payload_capacity("right_hand");
print(`Max payload at right_hand: ${payload} kg`);
```

### 3.3 Phase 2 拡張後の使用例

```rhai
// スクリプトでロボットをゼロから構築
new_model("simple_arm");

add_link("base");
add_link("upper_arm");
add_link("forearm");
add_link("hand");

add_joint("shoulder", "revolute", "base", "upper_arm");
add_joint("elbow", "revolute", "upper_arm", "forearm");
add_joint("wrist", "revolute", "forearm", "hand");

// ジオメトリ追加と慣性自動計算
add_visual("upper_arm", "cylinder", 0.02, 0.15);
add_collision("upper_arm", "cylinder", 0.02, 0.15);
auto_inertia("upper_arm", 2700.0); // アルミ密度

export_urdf("/tmp/simple_arm.urdf");
```

### 3.4 Phase 3 拡張後の使用例

```rhai
// 干渉チェック付きモーション探索
load("namiashi.urdf");

let target_x = 0.1;
for step in 0..100 {
    ik_steps("right_hand", target_x, 0.0, 0.5, 5);
    if self_collision() {
        let pairs = collision_pairs();
        print(`Collision at x=${target_x}: ${pairs}`);
        break;
    }
    target_x += 0.005;
}

// Jacobian によるマニピュラビリティ計算
let J = jacobian("right_hand");
print(`Jacobian rows: ${J.len()}, cols: ${J[0].len()}`);
```

---

## 4. CLI 実行

| 起動コマンド | 動作 |
|---|---|
| `articara` | 空 GUI |
| `articara <model>` | GUI でモデルロード |
| `articara --model <path>` | 同上 (明示 flag) |
| `articara --script <s>` | GUI 起動 + スクリプト auto-run |
| `articara --model <m> --script <s>` | **すべて自動 — クリック 0 回** |
| `articara <m> --script <s>` | 同上 (positional model) |
| `articara --script-headless <s> [m]` | ヘッドレス実行、 終了 (CI 用) |

### 例

```bash
# 全自動: モデルロード → GUI 起動 → スクリプト auto-run
articara --model tests/fixtures/namiashi/urdf/namiashi.urdf \
         --script scripts/walk_3axis_demo.rhai

# ヘッドレスでスクリプトを走らせて終了 (旧 --script 相当)
articara --script-headless scripts/analyze.rhai namiashi.urdf
```

`--script` は GUI 起動時の自動実行で、スクリプト末尾までいったら viewport
が動き続ける (sim が走っていれば)。 `--script-headless` は GUI を立ち上げず
スクリプトの `print()` を stdout に流して終了。

CI / ベンチマーク向けには `--script-headless` を、 デモ / 目視確認には
`--script` を使う。

---

## 5. 設計方針

1. **読み取り → 書き込み の順で段階実装** — 副作用のない関数を先に安定させる
2. **既存 Rust 関数の薄いラップ** — `rbd::dynamics`, `rbd::model`, `collision` 等を直接活用
3. **エラーは Rhai 例外に変換** — 不正な引数は `EvalAltResult::ErrorRuntime` で報告
4. **共有状態は `Arc<RwLock<RobotModel>>`** — スクリプトエンジンと UI 間で安全に共有
5. **全公開関数にタブ補完対応** — `completion_candidates()` に自動登録
6. **ヘルプテキスト同期** — `emit_help()` に全関数のリファレンスを維持
