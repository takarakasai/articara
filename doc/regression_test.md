# RoboView 回帰テスト仕様書

## 概要

RoboView の回帰テストは `tests/regression.rs` に実装されています。  
Rust 標準の `#[test]` フレームワークを使用し、`cargo test` で全テストを実行できます。

- **テスト総数**: 100
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

## テスト結果サマリ（最終実行）

```
test result: ok. 100 passed; 0 failed; 0 ignored; 0 measured
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
| **合計** | **100** | |

## テスト対象外

以下のモジュールはGUI/OpenGLコンテキスト依存のため、自動テストの対象外です：

- `app.rs` — eframe/eguiのUIロジック（手動テストで確認）
- `renderer.rs` — glow OpenGLレンダリング（GPUコンテキストが必要）

これらは `src/lib.rs` からエクスポートされておらず、統合テストからはアクセスできません。
