# Articara 機能仕様書

本ドキュメントは 2026-04-12〜2026-04-13 に追加・変更した機能をまとめたものである。

---

## 1. Undo/Redo（操作履歴）システム

### 概要

モデル編集操作に対する Undo（元に戻す）/ Redo（やり直し）機能を実装した。すべてのプロパティ変更・ビューポート操作がスナップショットベースの履歴に記録され、任意の時点に復元できる。

### 対象ファイル

- `src/history.rs` — 履歴管理コア（約340行）
- `src/app.rs` — UI 計装・履歴パネル

### 仕様詳細

#### 1.1 スナップショット方式

- 編集前のモデル状態を丸ごとスナップショットとして保存する方式を採用
- `History` 構造体が undo スタックと redo スタックを管理
- `mark_edit(description, model)`: 編集開始時に呼び出し、現在の状態をスナップショットとして undo スタックに積む
- `finalize()`: フレーム終了時に呼び出し、連続する同種編集のマージを確定する
- `undo(model)` / `redo(model)`: モデルを復元する

#### 1.2 プロパティパネルの計装

以下のプロパティ変更が履歴に記録される：

| セクション | 対象 |
|---|---|
| Inertial | mass, ixx, iyy, izz, ixy, ixz, iyz, com_x, com_y, com_z |
| Visual | origin (xyz + rpy), geometry パラメータ, color (RGBA) |
| Collision | origin (xyz + rpy), geometry パラメータ |
| Joint | axis (xyz), lower/upper limit, effort, velocity, damping, friction, armature |

各 `DragValue` の変更時に `props_edit_desc` フラグを立て、フレーム末尾で `mark_edit` → `finalize` の順に処理する。

#### 1.3 ビューポートドラッグ操作の計装

以下のドラッグ操作が履歴に記録される：

- **Gizmo 移動（Translate）**: リンクの平行移動
- **Gizmo 回転（Rotate）**: リンクの回転
- **Gizmo スケール（Scale）**: リンクのスケール変更
- **Joint Drive**: ジョイント角度のドラッグ操作
- **IK ドラッグ**: 逆運動学によるリンクチェーン操作

ドラッグ開始時（マウスボタン押下）に `mark_edit` を呼び出し、ドラッグ終了時（マウスボタン解放）に `finalize` を呼び出す。

#### 1.4 ドラッグ操作の単一エントリ化

**課題**: フレーム毎の `finalize()` 呼び出しにより、数フレームにまたがるドラッグ操作が複数の履歴エントリに分割されてしまう問題があった。

**解決策**: フレーム末尾の finalize 判定時に、ドラッグ中（`drag_state` または `offset_drag_state` が存在する）であれば `any_edit_this_frame = true` を設定し、finalize によるマージ切断を防止する。これにより、マウスボタンを押してから離すまでの一連のドラッグが1つの操作ログとして記録される。

#### 1.5 クリッカブル履歴パネル

- 左パネルに折りたたみ可能な「📋 History」パネルを表示
- Undo エントリ（過去の操作）と Redo エントリ（取り消された操作）をリストで表示
- 各エントリは `selectable_label` で描画され、クリックにより該当時点へ直接ジャンプ可能
- `goto(target_pos, model)` メソッドにより、現在位置から目標位置まで undo/redo を連続実行して任意の履歴位置に遷移する
- 現在位置は強調表示（ `>>` マーカー）される

#### 1.6 キーボードショートカット

- `Ctrl+Z`: Undo
- `Ctrl+Shift+Z` / `Ctrl+Y`: Redo

#### 1.7 テスト

`history.rs` に 11 個のユニットテストを実装：

- `mark_edit_creates_entry` — エントリ作成
- `undo_restores_previous` — Undo 復元
- `redo_after_undo` — Redo 動作
- `redo_cleared_on_new_edit` — 新規編集で redo クリア
- `finalize_after_gap_prevents_merge` — フレーム間ギャップでマージ防止
- `continuous_edits_merge` — 連続編集のマージ
- `max_history_cap` — 最大履歴数制限
- `undo_empty_noop` — 空履歴での undo が何もしない
- `redo_empty_noop` — 空 redo での redo が何もしない
- `multiple_undo_redo_cycles` — 複数回の undo/redo サイクル
- `goto_jumps_to_target` — goto による任意位置へのジャンプ

---

## 2. USDA インポート機能

### 概要

USD ASCII 形式（`.usda` / `.usd`）ファイルの読み込みをサポートした。URDF/MJCF と同様にロボットモデルとしてインポートし、リンク・ジョイント構造を復元する。

### 対象ファイル

- `src/usd_import.rs` — USDA パーサーおよび変換器（新規作成、約600行）
- `src/format.rs` — フォーマット検出の拡張
- `src/robot.rs` — `from_file` でのディスパッチ追加
- `src/main.rs` / `src/lib.rs` — モジュール登録

### 仕様詳細

#### 2.1 パーサー

- 行ベースのパーサーで `def Type "Name" { ... }` 構造をネスト対応で解析
- `parse_usda()` → `UsdPrim` ツリー（名前、型名、プロパティ、子プリム）
- 値パーサー: タプル `(x, y, z)`、クォータニオン `(w, x, y, z)`、配列 `[...]`、文字列

#### 2.2 リンク復元

- `Xform` + `PhysicsRigidBodyAPI` を持つプリムを `LinkData` に変換
- 質量（`physics:mass`）、対角慣性テンソル（`physics:diagonalInertia`）、重心（`physics:centerOfMass`）を読み取り
- 子プリムから Visual/Collision ジオメトリを抽出：
  - `Cube`: `xformOp:scale` の各成分を半長（half-extent）として Box に変換
  - `Cylinder`: `height` ÷ 2 を half-length として Cylinder に変換
  - `Sphere`: `radius` をそのまま使用
  - `Mesh`: `points` + `faceVertexIndices` からフラット頂点バッファに展開

#### 2.3 ジョイント復元

USD のジョイントプリムを `JointData` に変換する：

| USD ジョイント型 | 変換先 |
|---|---|
| `PhysicsRevoluteJoint` | `Revolute`（回転ジョイント） |
| `PhysicsPrismaticJoint` | `Prismatic`（直動ジョイント） |
| `PhysicsFixedJoint` | `Fixed`（固定ジョイント） |

- **軸の復元**: USD の `physics:axis` ("X"/"Y"/"Z") と `physics:localRot1` から実際の回転軸を算出  
  計算: `axis = inv(localRot1) * usd_principal_axis`
- **回転の復元**: `localRot0 * inv(localRot1)` から親子間の相対回転を復元
- **リミット変換**: Revolute ジョイントの `physics:lowerLimit` / `physics:upperLimit` を度からラジアンに変換
- **Continuous 検出**: リミットが ±360° の場合は Continuous（連続回転）ジョイントとして扱う

#### 2.4 マテリアル

- `UsdPreviewSurface` シェーダーの `inputs:diffuseColor` と `inputs:opacity` から色情報を抽出
- マテリアルバインディングを辿り、各ジオメトリに色を適用

#### 2.5 フォーマット検出

`format.rs` を拡張し、以下の拡張子を `RobotFormat::IsaacUsd` として検出：

- `.usda`
- `.usd`

`supports_import()` が `true` を返すように変更。

#### 2.6 テスト

`usd_import.rs` に 12 個のユニットテストを実装：

- `roundtrip_empty` — 空モデル
- `roundtrip_revolute` — Revolute ジョイント
- `roundtrip_fixed` — Fixed ジョイント
- `roundtrip_prismatic` — Prismatic ジョイント
- `roundtrip_material` — マテリアル色
- `roundtrip_box_geom` / `roundtrip_cylinder_geom` / `roundtrip_sphere_geom` — 各種ジオメトリ
- `roundtrip_inertial` — 慣性情報
- `roundtrip_multi_link_tree` — 複数リンクのツリー構造
- その他

---

## 3. 慣性テンソル自動計算機能

### 概要

リンクの Visual ジオメトリから、密度一定の均質材料と仮定して慣性テンソルを自動計算する機能を追加した。プリミティブ形状（Box, Cylinder, Sphere）には解析解を、メッシュには発散定理（Mirtich 法）に基づく数値計算を使用する。

### 対象ファイル

- `src/robot.rs` — 計算ロジック（約314行追加）
- `src/app.rs` — UI（約78行追加）
- `tests/regression.rs` — テスト（170行追加）

### 仕様詳細

#### 3.1 ジオメトリ別慣性テンソル計算

`compute_geometry_inertia(geom, mass)` により、各ジオメトリタイプの慣性テンソルを計算する：

**Box（直方体）**

$$I_{xx} = \frac{m}{12}(h^2 + d^2), \quad I_{yy} = \frac{m}{12}(w^2 + d^2), \quad I_{zz} = \frac{m}{12}(w^2 + h^2)$$

ここで $w, h, d$ はそれぞれ幅・高さ・奥行き（`size` の各成分）。

**Cylinder（円筒）**

$$I_{xx} = I_{yy} = \frac{m}{12}(3r^2 + h^2), \quad I_{zz} = \frac{mr^2}{2}$$

ここで $r$ は半径、$h$ は高さ（`length`）。Articara では Z 軸が円筒の中心軸。

**Sphere（球）**

$$I_{xx} = I_{yy} = I_{zz} = \frac{2mr^2}{5}$$

**Mesh（メッシュ）**

発散定理（Mirtich 法）を使用して体積積分を面積分に変換し数値計算する：
- 各三角形面について、面法線と頂点座標から $\int x^2 \, dV$, $\int y^2 \, dV$, $\int z^2 \, dV$, $\int xy \, dV$, $\int xz \, dV$, $\int yz \, dV$ を計算
- 密度 = mass / volume として慣性テンソルの6成分を算出

#### 3.2 体積計算

`compute_geometry_volume(geom)` により各ジオメトリの体積を計算する：

| ジオメトリ | 体積公式 |
|---|---|
| Box | $w \times h \times d$ |
| Cylinder | $\pi r^2 h$ |
| Sphere | $\frac{4}{3}\pi r^3$ |
| Mesh | 発散定理による符号付き体積（`compute_mesh_volume`） |

#### 3.3 複合リンクの慣性テンソル合成

`compute_link_inertia(visuals, total_mass)` により、リンクに含まれる複数の Visual ジオメトリの慣性テンソルを合成する：

1. 各 Visual の体積を計算し、体積比で質量を配分
2. 各 Visual の重心位置を求め、全体の重心（CoM）を質量加重平均で算出
3. 各 Visual について：
   - ローカル慣性テンソル $I_{\text{local}}$ を計算
   - 回転を考慮: $I_{\text{rot}} = R \cdot I_{\text{local}} \cdot R^T$（回転行列 $R$ で座標変換）
   - 平行軸の定理を適用: $I = I_{\text{rot}} + m(d^2 E - d \otimes d)$（$d$ は CoM からのオフセット）
4. 全 Visual の慣性テンソルを合計して最終結果とする

#### 3.4 UI

**⚡ Auto-compute from geometry ボタン**

- Inertial セクションに配置
- クリックすると、現在のリンクの Visual ジオメトリと質量から慣性テンソルと CoM を自動計算
- 計算結果を `ixx, iyy, izz, ixy, ixz, iyz` および `com_x, com_y, com_z` に反映

**📏 Mass from density ボタン**

- クリックすると密度入力ダイアログを表示
- デフォルト密度: 1000.0 kg/m³（水の密度）
- ダイアログ内容：
  - 密度入力フィールド（`DragValue`）
  - 参考材料リスト（Water: 1000, PLA: 1240, ABS: 1050, Aluminum: 2700, Steel: 7800, Titanium: 4500）
  - 体積プレビュー表示（ジオメトリから自動計算）
  - 計算結果（密度 × 体積 = 質量）のプレビュー
  - Apply / Cancel ボタン
- Apply クリックで質量を設定し、続けて慣性テンソルも自動計算

#### 3.5 テスト

`tests/regression.rs` の `test_inertia` モジュールに 10 個のテストを実装：

- `box_inertia_values` — Box の慣性テンソル値検証
- `cylinder_inertia_values` — Cylinder の慣性テンソル値検証
- `sphere_inertia_values` — Sphere の慣性テンソル値検証
- `volume_box` — Box の体積検証
- `volume_cylinder` — Cylinder の体積検証
- `volume_sphere` — Sphere の体積検証
- `combined_inertia_single_centered_visual` — 単一中心配置の合成慣性
- `combined_inertia_offset_visual` — オフセット配置の合成慣性（平行軸の定理）
- `combined_inertia_two_visuals_parallel_axis` — 2つの Visual の合成（平行軸の定理）
- `inertia_scales_with_mass` — 質量スケーリングの検証

---

## 4. `misarta::native` モジュール (.misa マスタフォーマット, 2026-05-02)

### 概要

URDF + `.misarta.toml` サイドカーの二重管理を解消する独自マスタ形式
`.misa`(中身は TOML)を導入。`misarta::native` モジュールが parser /
writer / 実行時 `Model` 変換を提供。articara の `RobotModel` を**完全
可逆**に永続化できる唯一の形式。

設計の経緯と決定事項は [`refactor_20260502.md`](refactor_20260502.md)、
on-disk スキーマの完全リファレンスは [`misa_schema.md`](misa_schema.md)、
他形式との機能比較は [`comparison.md`](comparison.md) を参照。

### 4.1 アーキテクチャ(3 層)

ファイルシステム依存を疎結合化し、組み込み・WASM 移植性を確保。

```
Layer 3: load(path) / save(path, &MisaFile)              [std + fs]
Layer 2: parse_str(text, &dyn AssetSource) / write_str / build_model  [std]
Layer 1: AssetSource trait + 4 built-in implementations  [no_std + alloc 互換]
```

### 4.2 対象ファイル

| ファイル | 内容 |
|---|---|
| `misarta/src/native/mod.rs` | 公開 API、`load` / `save`、`NativeError`、`ParseOutput` |
| `misarta/src/native/schema.rs` | serde 型定義(`MisaFile`, `Link`, `Joint`, `Geom`, `Origin`, `Actuator` 等) |
| `misarta/src/native/source.rs` | `AssetSource` トレイト + `FileSystemSource` / `InMemorySource` / `StaticBundleSource` / `NullSource` |
| `misarta/src/native/parse.rs` | TOML decode、識別子サニタイズ、構造バリデーション |
| `misarta/src/native/write.rs` | スキーマタグ検証 + canonical-order TOML 書き出し |
| `misarta/src/native/build.rs` | `MisaFile` → `Model + GeometryModel × 2` 変換 |
| `misarta/src/native/report.rs` | `LoadReport`, `sanitize_identifier` |

### 4.3 articara 側 API

| API | 戻り値 | 説明 |
|---|---|---|
| `RobotModel::from_misa(path)` | `Result<Self, String>` | `.misa` から `RobotModel` をロード(レポート破棄) |
| `RobotModel::from_misa_with_report(path)` | `Result<(Self, LoadReport), String>` | サニタイズ等の報告付き(GUI ダイアログ用) |
| `RobotModel::from_misa_file(&MisaFile, path)` | `Result<Self, String>` | 既パース済 `MisaFile` から構築(テスト/スクリプト向け) |
| `RobotModel::to_misa()` | `Result<MisaFile, String>` | `RobotModel` → メモリ上 `MisaFile` |
| `RobotModel::save_as_misa(path)` | `Result<(), String>` | `to_misa` + ファイル書き出しの便利関数 |

### 4.4 主要設計決定

- **拡張子**: `.misa`、ヘッダ `schema = "misarta/1"` 必須
- **構造表現**: フラット(URDF 型 + parent/child 参照)
- **姿勢表現**: rpy 既定 + quat 代替の共存
- **形状寸法**: URDF 流の全サイズ表現(`size = [w, h, d]`、`length` 全長)
- **メッシュ**: 外部参照のみ、`meshes/` サブディレクトリ前提
- **Actuator-Joint**: N:M 対応(`joints = [{name, gear}]`)、armature/damping は受動物理特性として joint 側に分離
- **マテリアル**: インライン色既定 + `[[material]]` 名前参照は任意
- **識別子規約**: `^[A-Za-z_][A-Za-z0-9_]*$`、違反は自動 sanitize + `LoadReport` 記録

### 4.5 既存ワークフローとの関係

- **`.misa` ソース** → `from_misa` / `save_as_misa`(完全可逆、サイドカー不要)
- **URDF + `.misarta.toml` サイドカー** → `from_urdf` + `load_sidecar_config`
  経路を **legacy として継続サポート**(既存ユーザ保護)
- **新規エクスポート**: Misa 選択時はサイドカーを生成しない
  (重複)、それ以外の format(URDF/MJCF/USD/SDF)は従来通り
  サイドカー併出力

### 4.6 テスト

- `misarta::native` モジュール内: 44 ユニットテスト(スキーマ
  ラウンドトリップ、サニタイズ、AssetSource 4 実装、N:M actuator、
  rpy/quat 切替、mesh asset 欠損非致命、二重 child 拒否 等)
- `articara::tests::regression::test_misa`: 16 統合テスト
  (basic round-trip、色解決、mimic、N:M actuator、loop_closure +
   collision_pair、pose + home、サニタイズレポート、from_file
   ディスパッチ、format detection、`namiashi.urdf + .misarta.toml`
   → `.misa` → `from_misa` の実機ラウンドトリップ全要素検証 等)

### 4.7 実機検証

`sample/namiashi_description/namiashi.misa` を URDF + サイドカーから
変換生成。URDF (19 link / 18 joint) + `.misarta.toml` (4 pose / 13
actuator / 6 collision_pair / 1 sequence / home) を単一 1894 行 TOML
に統合し、ラウンドトリップで構造完全一致を確認。

変換コマンド:
```bash
cargo run --example convert_to_misa -- \
  sample/namiashi_description/urdf/namiashi.urdf \
  sample/namiashi_description/namiashi.misa
```

---

## 5. テスト概況

全 171 テストが通過（PASS）している。

| カテゴリ | テスト数 |
|---|---|
| 回帰テスト（regression.rs） | 161 |
| 履歴ユニットテスト（history.rs） | 11 |
| USDA インポートテスト（usd_import.rs） | 12 |
| 慣性テンソルテスト（regression.rs::test_inertia） | 10 |
| **合計** | **171+α**（一部モジュール内テストを含む） |

> ⚠ 上記カウントは 2026-04-13 時点。`misarta::native` (2026-05-02)
> 追加で articara regression は 234 + misarta lib は 458 となっている。

---

## 6. 変更ファイル一覧

| ファイル | 変更内容 |
|---|---|
| `src/history.rs` | Undo/Redo コアロジック、`goto()` メソッド追加、ユニットテスト11個 |
| `src/app.rs` | プロパティパネル計装、ビューポートドラッグ計装、履歴パネル UI、慣性テンソル計算 UI、密度入力ダイアログ |
| `src/robot.rs` | `InertiaTensor` 構造体、`compute_geometry_inertia()`, `compute_geometry_volume()`, `compute_mesh_inertia()`, `compute_mesh_volume()`, `compute_link_inertia()`、`from_misa` / `to_misa` (2026-05-02) |
| `src/usd_import.rs` | USDA パーサー・インポーター（新規作成）、ユニットテスト12個 |
| `src/format.rs` | `.usda`/`.usd` 拡張子検出、`IsaacUsd.supports_import()` 有効化、`Misa` バリアント追加 (2026-05-02) |
| `src/main.rs` | `mod usd_import` 登録 |
| `src/lib.rs` | `pub mod usd_import` 登録 |
| `tests/regression.rs` | `supports_import` テスト更新、`test_inertia` モジュール追加（10テスト）、`test_misa` モジュール追加 (16テスト, 2026-05-02) |
| `misarta/src/native/` (2026-05-02) | `.misa` マスタフォーマットの parser / writer / model builder / asset source / report |
| `examples/convert_to_misa.rs` (2026-05-02) | URDF + sidecar → .misa CLI converter |
