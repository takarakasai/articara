# ロボットモデルフォーマット比較

articara が入出力としてサポート (予定含む) するロボットモデルフォーマットの
機能対応比較表。

対象フォーマット:

0. **Misa** — articara/misarta ネイティブのマスタフォーマット (`.misa` / TOML)
1. **URDF** — ROS1/2 用の Unified Robot Description Format
2. **SDF** — Gazebo の Simulation Description Format
3. **MJCF** — MuJoCo の XML フォーマット
4. **USD** — Isaac Sim / Omniverse の Universal Scene Description

`.misa` がマスタ表現で、その他4形式は派生エクスポート対象。設計議論の
経緯は `doc/refactor_20260502.md`、スキーマ詳細は `doc/misa_schema.md`
を参照。

## 5項目の比較表

| 項目 | **Misa** | URDF | SDF | MJCF | USD (Isaac) |
|---|---|---|---|---|---|
| **Collision pair (有効/除外)** | ✓ `[[collision_pair]]`<br>(マスタ表現) | ✗ ネイティブ無し<br>SRDF サイドカーで `<disable_collisions>` | ✓ `<collide_bitmask>` (bitmask)<br>`<self_collide>` flag<br>`<collision_filter>` | ✓ `<contact><pair>` (enable)<br>`<contact><exclude>` (disable)<br>`contype/conaffinity` bitmask | ✓ `PhysicsFilteredPairsAPI`<br>`PhysicsCollisionGroup` |
| **連動関節 (mimic)** | ✓ `[[mimic]]` 線形<br>(マスタ表現) | ✓ `<mimic joint=… multiplier=… offset=…>`<br>**線形のみ・単一source** | ✓ `<axis><mimic>`<br>(SDF 1.7+, **線形のみ**) | ✓ `<equality><joint>` 多項式係数<br>`<tendon>` (複数関節を腱で結ぶ複雑な連動も可) | ✗ ネイティブ無し<br>(Isaac 側で driver / Python script で実現) |
| **形状モデル形式** | ✓ box / cylinder / sphere / capsule / mesh<br>(`size = [w,h,d]` 全長表現) | プリミティブ: box / cylinder / sphere<br>メッシュ: STL, DAE | プリミティブ: box / sphere / cylinder / **capsule** (1.10+) / ellipsoid / plane / heightmap / polyline<br>メッシュ: STL, DAE, OBJ | プリミティブ: box / sphere / cylinder / capsule / ellipsoid / plane / hfield<br>メッシュ: STL, OBJ, MSH (プラグイン経由) | UsdGeom 全種 (Cube/Sphere/Cylinder/Cone/Capsule/Mesh/…)<br>外部参照: USD/OBJ/FBX (plugin) |
| **閉リンク機構** | ✓ `[[loop_closure]]`<br>(マスタ表現、3-DoF / 6-DoF) | ✗ **木構造のみ**<br>(workaround: ROS の transmission や外部 constraint で分解) | ✓ ネイティブ対応<br>(joint で任意リンク同士を接続可) | ✓ `<equality><connect>` 点拘束<br>`<equality><weld>` 6-DoF 拘束<br>`<equality><joint>` 関節間拘束 | ✓ Joint prim で任意 RigidBody 接続可<br>(ループでも宣言可、solver 安定性は要注意) |
| **センサ** | ✓ camera / lidar / imu / force_torque / contact / generic<br>(マスタ表現) | ✗ ネイティブ無し<br>(`<gazebo>` extension で SDF を埋め込む慣例) | ✓ camera / lidar (ray) / imu / force_torque / contact / gps / magnetometer / altimeter / sonar | ✓ accelerometer / gyro / framepos / framequat / framelinvel / jointpos / jointvel / jointactuatorfrc / force / torque / touch / tendonpos など多数 | 拡張 prim (Isaac 特有): RTX LiDAR / Camera / IMU / ContactSensor / ProximitySensor<br>(USD core 定義は無し) |

## 補足比較 (articara から見た重要項目)

| 項目 | **Misa** | URDF | SDF | MJCF | USD |
|---|---|---|---|---|---|
| **関節型** | revolute / continuous / prismatic / fixed / floating / planar | revolute / continuous / prismatic / fixed / planar / floating | revolute / prismatic / ball / universal / fixed / screw / revolute2 | hinge / slide / ball / free | Revolute / Prismatic / Spherical / Distance / Fixed / D6 |
| **慣性パラメータ** | mass + 6成分テンソル + COM origin (rpy/quat) | mass + 3×3 慣性テンソル (origin 指定可) | 同上 | mass + diaginertia or fullinertia | physics:mass + UsdPhysics:Inertia |
| **アクチュエータ** | ✓ N:M 対応<br>`[[actuator]] joints = [{name, gear}]`<br>4 modes (Position/Velocity/Torque/ComputedTorque) | ✗ (transmission で部分対応) | `<actuator>` (SDF 1.7+) | ✓ position / velocity / motor / muscle / general | Drive API (PhysicsDrive) |
| **マテリアル/色** | ✓ `[[material]]` 共有 + visual インライン | `<material>` (RGBA + texture) | 同上 | rgba on geom + `<texture>` asset | UsdShade Material network |
| **物理エンジン互換** | (description only — 任意 engine へ export 可) | (description only) | Gazebo (ODE/Bullet/DART/Simbody) | MuJoCo native | PhysX (Isaac) |
| **再ロードしやすさ (テキスト構造)** | TOML フラット、Git diff 行独立 | XML, シンプル | XML, やや冗長 | XML, 階層 body | テキスト USDA / バイナリ USDC, 大規模 |
| **編集メタ (pose/sequence/gait/home)** | ✓ ネイティブ (`[[pose]]` 等) | ✗ サイドカー (`.misarta.toml`) | ✗ サイドカー | ✗ サイドカー | ✗ サイドカー |
| **articara 内部往復** | ✓ 完全可逆 (`from_misa`/`to_misa`) | △ + サイドカー必須 | △ + サイドカー必須 | △ + サイドカー必須 | △ + サイドカー必須 |

## 要点と articara の現状

### Misa (新マスタ)

`articara/misarta` ネイティブの単一マスタフォーマット。`misarta::native`
モジュールが parser / writer / `Model` 変換を提供。articara の
`RobotModel` を **完全に可逆** に永続化できる唯一の形式で、URDF /
SDF / MJCF / USD はここから派生する lossy エクスポート対象。

- ファイル形式: TOML (`.misa` 拡張子、`schema = "misarta/1"` ヘッダ必須)
- メッシュ: 外部参照のみ (`meshes/foo.stl`)、`AssetSource` トレイト
  で fs 依存を疎結合化(組み込み・WASM 対応設計)
- サニタイズ: 識別子規約 `^[A-Za-z_][A-Za-z0-9_]*$` を自動修正、
  `LoadReport` で全変更を一覧

### Collision pair

URDF は SRDF サイドカー必須なので、articara は `.misa` で
`[[collision_pair]]` をネイティブ表現。MJCF export 時
`<contact><exclude>` / USD export 時 `physics:filteredPairs` に変換
する形で正規化済み。URDF + 旧 `.misarta.toml` 経路も legacy として
継続サポート。

### 連動関節 (mimic)

`.misa` の `[[mimic]]` でネイティブ保持。URDF/SDF の `<mimic>` を
パース → 保存し、MJCF export 時は `<equality><joint>` に変換する path
は今後追加予定。USD 側は coupling を表現できないため、import で警告を
出すのが現実的。

### 形状

articara の `GeomData` は `Box / Cylinder / Sphere / Capsule / Mesh`
で、`.misa` では URDF流の全サイズ表現 (`size = [w, h, d]`、`length`
全長、`radius` のみ片側) を採用。capsule が URDF では出力できない
(URDF は capsule 非対応) ため、URDF export 時は `analyze_export_compatibility`
が警告を出して cylinder + 2 sphere に分解する。

### 閉リンク

`.misa` の `[[loop_closure]]` でネイティブ保持。MJCF export 時に
`<equality><connect>` への変換は未実装(現在 URDF export では tree
構造に従う)。

### センサ

`.misa` の `[[sensor]]` で 6 種 (camera / lidar / imu / force_torque /
contact / generic) をネイティブ保持。URDF export では DROP、その他
形式では各形式のセンサ型に変換予定。

## 推奨ロードマップ

| 優先度 | 項目 | 理由 |
|---|---|---|
| ★★★ | **連動関節 (mimic) export 拡充** | `.misa` `[[mimic]]` から MJCF `<equality><joint>` への export を追加すれば URDF/SDF/MJCF の3形式が揃う |
| ★★ | **MJCF → 閉リンク export** | 既に `.misa` で `LoopClosure` を保持しているので、`<equality><connect>` 出力を1関数足せば済む |
| ★★ | **形状の format-specific 制限を明示** | `analyze_export_compatibility` が capsule → URDF 警告を出すように既に実装済み。他フォーマットも同様に拡張 |
| ★ | **センサの export 経路** | `.misa` でネイティブ保持済み、各 export 関数に変換ルーチンを追加 |
| ★ | **USD の articulation drive 対応** | Isaac との連携に重要だが、Articulation API がやや複雑 |
