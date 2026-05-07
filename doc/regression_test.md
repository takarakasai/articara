# Articara 回帰テスト仕様書

## 概要

Articara の回帰テストは `tests/regression.rs` に実装されています。  
Rust 標準の `#[test]` フレームワークを使用し、`cargo test` で全テストを実行できます。

- **テスト総数**: 118
- **テストファイル**: `tests/regression.rs`
- **フィクスチャ**: `tests/fixtures/`

## 実行方法

```bash
# 全テスト実行
cargo test

# 特定モジュールのテストのみ実行
cargo test test_robot
cargo test test_sdf

# テスト名でフィルタ実行
cargo test ray_sphere

# 出力キャプチャを無効にして実行（デバッグ用）
cargo test -- --nocapture
```

## テストフィクスチャ

| ファイル | 形式 | 内容 |
|---|---|---|
| `tests/fixtures/urdf/test_robot.urdf` | URDF | 4リンク・3ジョイント (box/cylinder/sphere/fixed) |
| `tests/fixtures/sdf/test_robot.sdf` | SDF | 3リンク・2ジョイント（URDF相当のSDF版） |
| `tests/fixtures/mjcf/test_robot.xml` | MJCF | 3リンク・2ジョイント（URDF相当のMJCF版） |
| `tests/fixtures/meshes/test_box.stl` | STL | 最小バイナリSTL（2三角形） |

### テスト用ロボット構造

```
base_link (box 0.2×0.2×0.1, mass=1.0)
├── [joint1: revolute, Y軸, ±1.57rad] → link1 (cylinder r=0.02 l=0.2, mass=0.5)
│   └── [joint2: revolute, Y軸, ±2.0rad] → link2 (sphere r=0.03, mass=0.3)
└── [fixed_joint: fixed] → fixed_part (box 0.05×0.05×0.02, mass=0.1)
```

また、実環境の `namiashi_description/urdf/namiashi.urdf`（四足歩行ロボット、18リンク・17ジョイント・STLメッシュ付き）を統合テストで使用します（ファイルが存在しない場合はスキップ）。

---

## テストモジュール一覧

### 1. test_robot（30テスト）— robot.rs

URDF読み込み、データ構造、トランスフォーム計算、レイキャスト、パス解決、エクスポートの回帰テスト。

#### URDF読み込み・データ構造

| テスト名 | 検証内容 |
|---|---|
| `load_fixture_urdf` | URDFのパース成功、名前・リンク数・ジョイント数・ルートリンクの確認 |
| `link_map_contains_all_links` | 全リンクが `link_map` に登録されている |
| `joint_types_correct` | ジョイントタイプ文字列が正しい（`"revolute"`, `"fixed"`） |
| `joint_limits_parsed` | ジョイントリミット（lower/upper/effort/velocity）が正しく解析される |
| `joint_axis_parsed` | ジョイント軸ベクトルの値が正しい |
| `inertial_parsed` | 慣性パラメータ（mass, ixx等）が正しく読み込まれる |
| `visual_geometry_types` | 各リンクのビジュアルジオメトリ型（Box/Cylinder/Sphere）が正しい |
| `materials_and_colors` | マテリアル名参照による色の解決が正しい |
| `joint_positions_initialized_to_zero` | 初期ジョイント位置が全てゼロ |
| `source_path_stored` | `source_path` にロード元パスが保存される |
| `children_joints_structure` | 親リンクごとの子ジョイントマップが正しい |
| `parent_joint_of_link` | リンク名から親ジョイントの逆引きが正しい |

#### トランスフォーム計算

| テスト名 | 検証内容 |
|---|---|
| `compute_transforms_at_zero` | ジョイント角度ゼロ時のリンク位置が正しい（ルート=原点、link1=z:0.05、link2=z:0.25） |
| `compute_transforms_with_joint_rotation` | joint1 を90°回転させた場合の link2 の位置がX方向にシフトする |
| `fixed_joint_child_transform` | fixedジョイントの子リンクが正しい位置に配置される |

#### バウンディング球

| テスト名 | 検証内容 |
|---|---|
| `bounding_sphere_base_link` | Boxジオメトリのバウンディング球の中心と半径が妥当な範囲 |
| `bounding_sphere_empty_visuals` | ビジュアルが空のリンクは半径ゼロ |

#### レイキャスト

| テスト名 | 検証内容 |
|---|---|
| `ray_sphere_hit` | レイが球に命中する場合の距離が正しい |
| `ray_sphere_miss` | レイが球を外す場合に `None` を返す |
| `ray_box_hit` | レイがAABBに命中する場合の距離が正しい |
| `ray_box_miss` | レイがAABBを外す場合に `None` を返す |
| `ray_cylinder_hit_side` | レイがシリンダー側面に命中する |
| `ray_cylinder_hit_cap` | レイがシリンダーキャップに命中する |
| `ray_triangle_hit` | Möller–Trumbore法による三角形命中テスト |
| `ray_triangle_miss` | 三角形外のレイが `None` を返す |
| `ray_mesh_intersect_with_flat_vertices` | flat vertex 配列（18float/三角形）でのメッシュ交差テスト |

#### パス解決

| テスト名 | 検証内容 |
|---|---|
| `resolve_package_path_basic` | `package://` URI が正しく絶対パスに変換される |
| `resolve_package_path_file_uri` | `file://` URI が正しく処理される |
| `resolve_package_path_relative` | 相対パスがそのまま返される |

#### エクスポート・フォーマットディスパッチ

| テスト名 | 検証内容 |
|---|---|
| `export_urdf_roundtrip` | `export_urdf()` が有効なURDF XMLを生成する |
| `from_file_urdf` | `from_file()` でURDFを自動検出して読み込める |
| `from_file_sdf` | `from_file()` でSDFを自動検出して読み込める |
| `from_file_mjcf` | `from_file()` でMJCFを自動検出して読み込める |
| `from_file_unknown_extension` | 未知の拡張子でエラーを返す |

#### namiashi 統合テスト

| テスト名 | 検証内容 |
|---|---|
| `load_namiashi_urdf` | 実ロボットURDFのロード（リンク/ジョイント数、STLメッシュ読み込み確認） |
| `namiashi_transforms_reasonable` | 全リンクのトランスフォームが存在し、位置が妥当な範囲内 |
| `namiashi_pick_link` | 上方からのレイキャストでロボット本体にヒットする |

---

### 2. test_format（14テスト）— format.rs

フォーマット検出ロジックの回帰テスト。

| テスト名 | 検証内容 |
|---|---|
| `detect_urdf_extension` | `.urdf` → `Urdf` |
| `detect_xacro_extension` | `.xacro` → `Urdf` |
| `detect_sdf_extension` | `.sdf` → `Sdf` |
| `detect_world_extension` | `.world` → `Sdf` |
| `detect_xml_extension` | `.xml` → `Mjcf`（デフォルト） |
| `detect_unknown_extension` | `.png` → `None` |
| `detect_no_extension` | 拡張子なし → `None` |
| `supports_import` | URDF/SDF/MJCF は `true`、IsaacUsd は `false` |
| `supports_export` | 全フォーマットが `true` |
| `all_contains_four` | `RobotFormat::ALL` が4要素 |
| `labels_non_empty` | 全フォーマットのラベルが空でない |
| `extensions_non_empty` | 全フォーマットの拡張子が空でない |
| `display_trait` | `Display` トレイトの出力文字列が正しい |
| `detect_from_fixture_sdf` | SDFフィクスチャファイルの内容ベース検出 |
| `detect_from_fixture_mjcf` | MJCFフィクスチャファイルの `<mujoco` タグ検出 |

---

### 3. test_sdf（11テスト）— sdf.rs

SDFインポート・エクスポートの回帰テスト。

| テスト名 | 検証内容 |
|---|---|
| `import_sdf_basic` | SDFパース成功、名前・リンク数・ジョイント数・ルートリンク |
| `sdf_link_inertial` | 慣性パラメータ（mass, ixx）が正しく読み込まれる |
| `sdf_visual_geometry` | Box ジオメトリの half-extent が正しい |
| `sdf_cylinder_geometry` | Cylinder の radius/half_length が正しい |
| `sdf_sphere_geometry` | Sphere の radius が正しい |
| `sdf_joint_properties` | ジョイントタイプ・親子リンク・リミットが正しい |
| `sdf_visual_color` | `<ambient>` カラーが正しく解析される |
| `sdf_collision_parsed` | コリジョンジオメトリが解析される |
| `sdf_source_path` | `source_path` が保存される |
| `export_sdf_contains_model` | エクスポートXMLに必要な要素が含まれる |
| `sdf_roundtrip_data_preserved` | SDF → エクスポート → 再インポートでデータが保持される |

---

### 4. test_mjcf（7テスト）— mjcf.rs

MJCFインポート・エクスポートの回帰テスト。

| テスト名 | 検証内容 |
|---|---|
| `import_mjcf_basic` | MJCFパース成功、リンク数≧3、ジョイント数≧2 |
| `mjcf_link_names` | `base_link`, `link1`, `link2` が全て存在する |
| `mjcf_joint_properties` | ジョイントタイプ・リミットが正しい |
| `mjcf_inertial` | 慣性パラメータが正しい |
| `mjcf_visual_geometry` | ビジュアルジオメトリ型（Box）が正しい |
| `export_mjcf_contains_mujoco` | エクスポートXMLに `<mujoco>` タグが含まれる |
| `mjcf_roundtrip_data_preserved` | MJCF → エクスポート → 再インポートでデータが保持される |

---

### 5. test_isaac（4テスト）— isaac.rs

Isaac Sim エクスポートの回帰テスト。

| テスト名 | 検証内容 |
|---|---|
| `export_isaac_python_script` | Pythonスクリプトに必要な要素（import, robot名, URDF_PATH, DriveAPI）が含まれる |
| `isaac_script_has_joint_config` | revolute ジョイントの angular drive 設定が含まれ、fixed ジョイントはスキップされる |
| `isaac_script_has_physics_scene` | 物理シーン・重力設定が含まれる |
| `export_isaac_to_dir_creates_files` | 出力ディレクトリに URDF ファイルと Python スクリプトが生成される |

---

### 6. test_camera（8テスト）— camera.rs

OrbitCamera の数学的振る舞いの回帰テスト。

| テスト名 | 検証内容 |
|---|---|
| `default_camera` | デフォルト値（distance > 0, fov_y > 0）が妥当 |
| `eye_position_changes_with_distance` | distance を2倍にすると eye-target 距離も2倍になる |
| `eye_at_target_distance` | eye() と target の距離が distance と一致する |
| `view_matrix_is_invertible` | ビュー行列が逆行列を持つ |
| `projection_matrix_is_invertible` | プロジェクション行列が逆行列を持つ |
| `project_target_near_center` | ターゲット点がスクリーン中心付近に投影される |
| `screen_ray_center_points_at_target` | スクリーン中心からのレイがターゲット方向を向く |
| `screen_ray_origin_near_eye` | レイの原点がカメラの eye 位置に近い |

---

### 7. test_ik（7テスト）— ik.rs

逆運動学（DLS法）の回帰テスト。

| テスト名 | 検証内容 |
|---|---|
| `build_chain_two_joints` | link2 までのチェーンが joint1→joint2 の2要素 |
| `build_chain_one_joint` | link1 までのチェーンが joint1 の1要素 |
| `build_chain_root_is_empty` | ルートリンクのチェーンは空 |
| `build_chain_fixed_joint_skipped` | fixed ジョイントはチェーンに含まれない |
| `jacobian_dimensions` | ヤコビアンのサイズが 3×（チェーン長） |
| `ik_step_reduces_error` | 1ステップのIK計算でエンドエフェクタの位置誤差が減少する |
| `apply_ik_deltas_respects_limits` | 大きなデルタを適用してもジョイントリミットを超えない |

---

### 8. test_primitives（8テスト）— primitives.rs

プリミティブジオメトリ生成の回帰テスト。

| テスト名 | 検証内容 |
|---|---|
| `generate_box_vertex_count` | Box の頂点数が 216（6面×2三角形×3頂点×6float）|
| `generate_box_coords_within_bounds` | 全頂点座標が half-extent 以内 |
| `generate_cylinder_non_empty` | Cylinder の頂点配列が空でない |
| `generate_cylinder_radius_bounded` | 全頂点が半径・高さの範囲内 |
| `generate_sphere_non_empty` | Sphere の頂点配列が空でない |
| `generate_sphere_radius_bounded` | 全頂点が半径以内 |
| `generate_grid_non_empty` | Grid の頂点配列が空でない |
| `generate_axes_six_endpoints` | 軸線が 36 float（3軸×2端点×6float） |

---

### 9. test_cross_format（3テスト）— クロスフォーマット統合テスト

異なるフォーマット間の変換・互換性テスト。

| テスト名 | 検証内容 |
|---|---|
| `urdf_to_sdf_roundtrip` | URDF → SDF エクスポート → SDF 再インポートでリンク数・ジョイント数・質量が保持される |
| `urdf_to_mjcf_roundtrip` | URDF → MJCF エクスポート → MJCF 再インポートでリンク名・質量が保持される |
| `all_formats_produce_valid_transforms` | URDF/SDF/MJCF 全フォーマットでロードしたモデルのトランスフォームが有効で妥当な範囲内 |

---

### 10. test_model_editing（18テスト）— モデル編集（リンク/ジョイント追加・削除）

プログラムによるモデル構築・編集機能のテスト。

| テスト名 | 検証内容 |
|---|---|
| `new_empty_has_root_link` | `new_empty()` で base_link 1つのみ、ジョイントなし |
| `add_link_updates_maps` | `add_link()` で links/link_map が正しく更新される |
| `add_joint_updates_maps_and_children` | `add_joint()` で joints/joint_map/children_joints/joint_positions が更新される |
| `add_joint_invalid_parent_fails` | 存在しない親リンクでエラーが返る |
| `add_child_creates_link_and_joint` | `add_child()` でリンク＋ジョイントがワンステップで追加される |
| `add_child_transforms_valid` | 追加したリンクの `compute_transforms()` がオリジンに一致 |
| `generate_link_name_unique` | 既存名と衝突しないユニーク名が生成される |
| `generate_joint_name_unique` | 既存名と衝突しないユニーク名が生成される |
| `remove_link_basic` | リンク削除でリンク・ジョイント・マップが正しく更新される |
| `remove_link_recursive` | 親リンク削除で子孫も再帰的に削除される |
| `remove_root_link_fails` | ルートリンク削除でエラーが返る |
| `remove_nonexistent_link_fails` | 存在しないリンク削除でエラーが返る |
| `rebuild_indices_consistency` | `rebuild_indices()` 後の全マップがベクタと一致 |
| `link_names_returns_all` | 全リンク名が返される |
| `added_model_exports_valid_urdf` | 新規構築モデルの `export_urdf()` が有効なXML |
| `added_model_exports_valid_sdf` | 新規構築モデルの `export_sdf()` が有効なXML |
| `added_model_exports_valid_mjcf` | 新規構築モデルの `export_mjcf()` が有効なXML |
| `multiple_children_from_same_parent` | 同じ親から複数子リンクを追加し、トランスフォームが正しい |

---

## テスト結果サマリ（最終実行）

```
test result: ok. 118 passed; 0 failed; 0 ignored; 0 measured
```

| モジュール | テスト数 | カバー対象 |
|---|---|---|
| test_robot | 30 | URDFロード, トランスフォーム, レイキャスト, パス解決, エクスポート, namiashi統合 |
| test_format | 14 | フォーマット検出（拡張子・内容ベース）, メタデータ |
| test_sdf | 11 | SDFインポート/エクスポート, ラウンドトリップ |
| test_mjcf | 7 | MJCFインポート/エクスポート, ラウンドトリップ |
| test_isaac | 4 | Isaac Sim Pythonスクリプト生成, ファイル出力 |
| test_camera | 8 | カメラ数学（投影, レイキャスト, 距離計算） |
| test_ik | 7 | IKチェーン構築, ヤコビアン, DLS求解, リミット遵守 |
| test_primitives | 8 | Box/Cylinder/Sphere/Grid/Axes 頂点生成 |
| test_cross_format | 3 | URDF↔SDF/MJCF ラウンドトリップ, 全フォーマット整合性 |
| test_model_editing | 18 | 新規モデル作成, リンク/ジョイント追加, 削除, URDF/SDF/MJCF生成, インデックス整合性 |
| **合計** | **118** | |

## テスト対象外

以下のモジュールはGUI/OpenGLコンテキスト依存のため、自動テストの対象外です：

- `app.rs` — eframe/eguiのUIロジック（手動テストで確認）
- `renderer.rs` — glow OpenGLレンダリング（GPUコンテキストが必要）

これらは `src/lib.rs` からエクスポートされておらず、統合テストからはアクセスできません。

---

## 歩容制御 MuJoCo 回帰スイート

`tests/regression.rs` (フォーマット / IK / カメラ等の純粋ロジック) とは別に、
**MuJoCo を実際に走らせる歩容制御の e2e 回帰**を独立した integration test
として用意しています。MPC・WBC・LKF を実装した各 layer に対して、組合せ
ごとに「身体が落ちないか」「前進するか」「推定が収束するか」を検証します。

### テスト一覧

| ファイル | テスト | 構成 | 検証指標 |
|---|---|---|---|
| `tests/gait_walk_stability.rs` | `champ_walks_stable` | CHAMP + Position-PD | 1 cycle 安定 + 4 cm 以上前進 |
| `tests/gait_walk_stability.rs` | `mpc_walks_stable` | SRBD MPC + Position-PD + τ_ff | 同上 |
| `tests/wbc_walk.rs` | `wbc_static_stand_balances_gravity` | Hybrid joint (PD + WBC τ_ff) | min_z > 0.18 m, Σf_z ≈ m·g (±60%) |
| `tests/wbc_walk.rs` | `wbc_forward_command_advances_body` | Hybrid joint + 前進指令 | min_z > 0.18 m, Δx > 4 cm (P5b 後 pass) |
| `tests/lkf_pipeline.rs` | `lkf_static_stand_tracks_ground_truth_body_z` | LKF 単独 + ground truth | body z 推定誤差 < 5 cm |
| `tests/integration_walk.rs` | `integration_position_pd_with_mpc_torque_ff` | PD + MPC τ_ff | min_z > 0.18 m, Δx > 2 cm |
| `tests/integration_walk.rs` | `integration_position_pd_plus_wbc` | PD + WBC + ContactDrivenPhase | min_z > 0.18 m, Δx > -10 cm |
| `tests/integration_walk.rs` | `integration_position_pd_plus_lkf` | PD + MPC + LKF (parallel) | min_z > 0.18 m, KF z-err < 5 cm |
| `tests/integration_walk.rs` | `integration_walk_straight_champ` | CHAMP open-loop forward | min_z > 0.18 m (drift は記録のみ) |
| `tests/integration_walk.rs` | `integration_walk_straight_mpc_wbc` | MPC+WBC + Hybrid + 5 s forward | body_dx > +0.10 m, \|body_dy\| < 0.20 m, \|Δyaw\| < 1.0 rad |
| `tests/integration_walk.rs` | `integration_walk_lateral_champ` | CHAMP open-loop lateral | min_z > 0.18 m (記録のみ) |
| `tests/integration_walk.rs` | `integration_walk_lateral_mpc_wbc` | MPC+WBC + 5 s lateral | body_dy > +0.20 m, \|body_dx\| < 0.30 m, \|Δyaw\| < 1.5 rad |
| `tests/integration_walk.rs` | `integration_walk_yaw_champ` | CHAMP open-loop yaw | min_z > 0.18 m (記録のみ) |
| `tests/integration_walk.rs` | `integration_walk_yaw_mpc_wbc` | MPC+WBC + 5 s yaw | \|Δyaw\| > 1.5 rad, \|body_dx\| / \|body_dy\| < 0.35 m |
| `tests/integration_walk.rs` | `integration_walk_lateral_mpc_no_wbc` | MPC + Position-PD のみ (WBC 無効) | 切り分け診断 (assertion = fall guard) |
| `tests/integration_walk.rs` | `diag_lateral_no_*` × 4 | per-task 無効化 (WBC weights override) | **#[ignore]** P5a 診断のみ |
| `tests/integration_walk.rs` | `diag_swing_leg_sweep_*` × 2 | swing_leg weight sweep | **#[ignore]** P5b sweep |
| `tests/integration_walk.rs` | `diag_forward_no_swing_leg` | swing_leg=0 単独 | **#[ignore]** 候補 fix 検証 |
| `tests/misarta_mujoco_gravity_consistency.rs` | `*` | misarta vs MuJoCo 動力学一致 | 重力反作用 τ が一致 |

### 3 軸独立 benchmark の特徴 (P1 + P5b)

`integration_walk_*` は **active axis (cmd 通りの進行) と cross axes (ずれてい
ないか)** を独立に評価します:

| 命令 | active axis (進む) | cross axis 1 | cross axis 2 |
|---|---|---|---|
| forward (cmd.vx=+0.15) | body_dx | body_dy (横ずれ) | Δyaw (回転) |
| lateral (cmd.vy=+0.10) | body_dy | body_dx (前後ずれ) | Δyaw (旋回) |
| yaw (cmd.wz=+0.5) | Δyaw | body_dx (前後) | body_dy (左右) |

`body_dx` / `body_dy` は **初期 yaw で逆回転**して body 初期姿勢の前進 / 横移動
成分を取り出すアクセサ (`WalkBenchmark::body_dx`, `body_dy`)。yaw 命令で
body が回っても cross-axis 評価が一貫します。

### 共通ヘルパー (`tests/common/mod.rs`)

URDF ロード、IK 経由の関節 seed、Position-PD アクチュエータ設定、`MujocoSim`
構築をまとめた helper を提供。各テストは `mod common;` で取り込み、
`build_namiashi_stand_fixture()` で `(robot, kin, sim)` を一発で得られる。

### 1 コマンド回帰実行

```bash
# 全 MuJoCo 回帰を release で走らせる (約 30 秒、6 スイート)
./scripts/test_regression.sh

# Lib のみ + walk_stability だけ走らせる (約 5 秒、pre-commit に最適)
./scripts/test_regression.sh --quick

# Debug ビルド (開発中)
./scripts/test_regression.sh --debug
```

スクリプトは `MUJOCO_DOWNLOAD_DIR` / `LD_LIBRARY_PATH` を自動補完するので、
通常は引数なしで動く。各 stage の pass/fail がそれぞれの section header に
出るので、どこで regress したかが一目で分かる。

### 推奨ワークフロー

```
1. WBC / MPC / 推定器 / misarta::{qp,jacobian,fk} を変更
2. ./scripts/test_regression.sh
3. PASSED が出たら commit、FAILED ならどの section かを見て修正
4. 大きな変更時は --quick でなく full を走らせる (15-30 秒)
```

### Phase 進捗との対応

| 設計 doc Phase | 担当テスト | 状態 |
|---|---|---|
| Phase A (WBC) | `wbc_walk::wbc_static_stand_balances_gravity` + `wbc_forward_command_advances_body` + `integration_walk::*_plus_wbc` + 3 軸 benchmark | ✅ 全 pass |
| Phase B (LKF) | `lkf_pipeline::lkf_static_stand_tracks_ground_truth_body_z` + `integration_walk::*_plus_lkf` | ✅ 全 pass |
| Phase C (Contact-driven phase) | `integration_walk::*_plus_wbc` (内部で `ContactDrivenPhase` を使用) | ✅ pass |
| Phase 1.5 残 (forward walk) | `wbc_walk::wbc_forward_command_advances_body` | ✅ **P5b 後 #[ignore] 解除** |
| Phase P1 / P5b (3 軸 benchmark) | `integration_walk_straight_*` / `_lateral_*` / `_yaw_*` | ✅ MPC+WBC pass |
| Phase P5a 診断 | `diag_lateral_no_*`, `diag_swing_leg_sweep_*` | ⚪ #[ignore] 残 (診断専用) |

## GUI で描画付きでベンチマークを再現する

`./scripts/test_regression.sh` はヘッドレス (数値だけ出力) ですが、 同じ
cmd 列を **viewport で目視確認**できる Rhai スクリプトと、 **GUI クリック
0 回**で起動する CLI フラグも用意しています。

### `--script <path>` CLI フラグ

```bash
# 全自動: モデルロード → スクリプト auto-run → 3 軸 benchmark を順次再生
MUJOCO_DOWNLOAD_DIR="$HOME/.mujoco" \
MUJOCO_DYNAMIC_LINK_DIR="$HOME/.mujoco/mujoco-3.8.0/lib" \
LD_LIBRARY_PATH="$HOME/.mujoco/mujoco-3.8.0/lib:$LD_LIBRARY_PATH" \
  cargo run --release --features "mujoco scripting" -- \
    --model tests/fixtures/namiashi/urdf/namiashi.urdf \
    --script scripts/walk_3axis_demo.rhai
```

GUI が起動して:
- ✅ namiashi URDF 自動読み込み
- ✅ Script Console が自動で開く
- ✅ `walk_3axis_demo.rhai` が auto-run
- ✅ スクリプトが GUI 設定 (Mode, Pose source, WBC, Ground plane) を全部適用
- ✅ 18 秒の async timeline で forward → lateral → yaw を順次再生
- ✅ 終了時 `log/walk_3axis_demo.csv` に軌跡保存

### 手動で起動 (途中で介入したい場合)

```bash
cargo run --release --features mujoco
```

→ GUI で URDF 読み込み → Quadruped gait panel で:
- Generator = MPC (capture-point)
- Pose source = MuJoCo ground truth
- Hierarchical WBC = ON

→ Console (📂) → `scripts/walk_3axis_demo.rhai`

### CLI 仕様まとめ

| 起動コマンド | 動作 |
|---|---|
| `articara` | 空 GUI |
| `articara <model>` | GUI でモデルロード |
| `articara --model <path>` | 同上 (明示) |
| `articara --script <s>` | GUI + script auto-run (モデル未ロード) |
| `articara --model <m> --script <s>` | **すべて自動 — クリック 0 回** |
| `articara <m> --script <s>` | 同上 (positional model) |
| `articara --script-headless <s> [m]` | ヘッドレス実行、 終了 |

