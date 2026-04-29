# ロボットモデルフォーマット比較

articara が入出力としてサポート (予定含む) するロボットモデルフォーマットの
機能対応比較表。

対象フォーマット:

1. **URDF** — ROS1/2 用の Unified Robot Description Format
2. **SDF** — Gazebo の Simulation Description Format
3. **MJCF** — MuJoCo の XML フォーマット
4. **USD** — Isaac Sim / Omniverse の Universal Scene Description

## 5項目の比較表

| 項目 | URDF | SDF | MJCF | USD (Isaac) |
|---|---|---|---|---|
| **Collision pair (有効/除外)** | ✗ ネイティブ無し<br>SRDF サイドカーで `<disable_collisions>` | ✓ `<collide_bitmask>` (bitmask)<br>`<self_collide>` flag<br>`<collision_filter>` | ✓ `<contact><pair>` (enable)<br>`<contact><exclude>` (disable)<br>`contype/conaffinity` bitmask | ✓ `PhysicsFilteredPairsAPI`<br>`PhysicsCollisionGroup` |
| **連動関節 (mimic)** | ✓ `<mimic joint=… multiplier=… offset=…>`<br>**線形のみ・単一source** | ✓ `<axis><mimic>`<br>(SDF 1.7+, **線形のみ**) | ✓ `<equality><joint>` 多項式係数<br>`<tendon>` (複数関節を腱で結ぶ複雑な連動も可) | ✗ ネイティブ無し<br>(Isaac 側で driver / Python script で実現) |
| **形状モデル形式** | プリミティブ: box / cylinder / sphere<br>メッシュ: STL, DAE | プリミティブ: box / sphere / cylinder / **capsule** (1.10+) / ellipsoid / plane / heightmap / polyline<br>メッシュ: STL, DAE, OBJ | プリミティブ: box / sphere / cylinder / capsule / ellipsoid / plane / hfield<br>メッシュ: STL, OBJ, MSH (プラグイン経由) | UsdGeom 全種 (Cube/Sphere/Cylinder/Cone/Capsule/Mesh/…)<br>外部参照: USD/OBJ/FBX (plugin) |
| **閉リンク機構** | ✗ **木構造のみ**<br>(workaround: ROS の transmission や外部 constraint で分解) | ✓ ネイティブ対応<br>(joint で任意リンク同士を接続可) | ✓ `<equality><connect>` 点拘束<br>`<equality><weld>` 6-DoF 拘束<br>`<equality><joint>` 関節間拘束 | ✓ Joint prim で任意 RigidBody 接続可<br>(ループでも宣言可、solver 安定性は要注意) |
| **センサ** | ✗ ネイティブ無し<br>(`<gazebo>` extension で SDF を埋め込む慣例) | ✓ camera / lidar (ray) / imu / force_torque / contact / gps / magnetometer / altimeter / sonar | ✓ accelerometer / gyro / framepos / framequat / framelinvel / jointpos / jointvel / jointactuatorfrc / force / torque / touch / tendonpos など多数 | 拡張 prim (Isaac 特有): RTX LiDAR / Camera / IMU / ContactSensor / ProximitySensor<br>(USD core 定義は無し) |

## 補足比較 (articara から見た重要項目)

| 項目 | URDF | SDF | MJCF | USD |
|---|---|---|---|---|
| **関節型** | revolute / continuous / prismatic / fixed / planar / floating | revolute / prismatic / ball / universal / fixed / screw / revolute2 | hinge / slide / ball / free | Revolute / Prismatic / Spherical / Distance / Fixed / D6 |
| **慣性パラメータ** | mass + 3×3 慣性テンソル (origin 指定可) | 同上 | mass + diaginertia or fullinertia | physics:mass + UsdPhysics:Inertia |
| **アクチュエータ** | ✗ (transmission で部分対応) | `<actuator>` (SDF 1.7+) | ✓ position / velocity / motor / muscle / general | Drive API (PhysicsDrive) |
| **マテリアル/色** | `<material>` (RGBA + texture) | 同上 | rgba on geom + `<texture>` asset | UsdShade Material network |
| **物理エンジン互換** | (description only) | Gazebo (ODE/Bullet/DART/Simbody) | MuJoCo native | PhysX (Isaac) |
| **再ロードしやすさ (テキスト構造)** | XML, シンプル | XML, やや冗長 | XML, 階層 body | テキスト USDA / バイナリ USDC, 大規模 |

## 要点と articara の現状

### Collision pair

URDF は SRDF サイドカー必須なので、articara は `.misarta.toml` を独自サイドカー
として使い、MJCF export 時 `<contact><exclude>` / USD export 時
`physics:filteredPairs` に変換する形で正規化済み。

### 連動関節 (mimic)

現状 articara のモデルでは未保持。URDF/SDF の `<mimic>` をパース → 保存し、
MJCF export 時は `<equality><joint>` に変換する path を増やすのが筋。USD 側は
articara で coupling を表現できないため、import で警告を出すのが現実的。

### 形状

articara の `GeomData` は `Box / Cylinder / Sphere / Capsule / Mesh` で、
capsule が URDF では出力できない (URDF は capsule 非対応)。今は Mesh に
decompose して export している箇所はあるか要確認。

### 閉リンク

articara は misarta の `LoopClosure` で扱っており、`.misarta.toml` で
永続化している。MJCF export 時に `<equality><connect>` への変換は未実装
(現在 export では URDF の tree 構造に従う)。

### センサ

articara は完全に未対応。受け入れる場合は `RobotModel` に
`Vec<SensorSpec>` を追加するか、各 import 時に丸ごと無視するか方針決定が必要。

## 推奨ロードマップ

| 優先度 | 項目 | 理由 |
|---|---|---|
| ★★★ | **連動関節 (mimic) 対応** | URDF/SDF/MJCF どれも保有概念。namiashi 含め多くのロボットで使われる。articara のモデル / sidecar / 各 export に追加するだけで4形式が揃う |
| ★★ | **MJCF → 閉リンク export** | 既に LoopClosure を保持しているので、`<equality><connect>` 出力を1関数足せば済む |
| ★★ | **形状の format-specific 制限を明示** (URDF で capsule を出力しない等) | 警告 or 自動 fallback (capsule → cylinder) を実装すれば他フォーマットへの export 時のサプライズを減らせる |
| ★ | **センサの最小サポート** | 利用範囲が広いが現状ゼロから — 設計負荷大、優先度は中 |
| ★ | **USD の articulation drive 対応** | Isaac との連携に重要だが、Articulation API がやや複雑 |
