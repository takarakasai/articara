# `misarta::native` の `no_std + alloc` 対応 — 調査ノート

`refactor_20260502.md` §9.1 で「設計上は疎結合化済みだが実装は std 必須」
として暫定対応した項目の追加調査。本ドキュメントは作業着手前の前提
整理 + 候補比較で、実装は将来別タスクとして行う。

## 1. なぜ `no_std + alloc` か

`misarta::native::AssetSource` トレイトを定義した時点で、ファイルシステム
依存は Layer 1 から既に追い出してある(`AssetSource` の実装が
`FileSystemSource` / `InMemorySource` / `StaticBundleSource` /
`NullSource` の 4 通り)。残るは:

- **Layer 2 のコアパーサ + ライタ**(`parse.rs` / `write.rs` / `build.rs`)
  を `std` への依存なしでビルドできるようにする
- **Layer 3 の `load` / `save` 関数**は `std` 必須のまま据え置き
  (これらが `std::fs::read_to_string` / `write` を呼ぶ)

期待される効果:
- 組み込み Linux + alloc 環境で `.misa` を読める
- WASM (no_std + alloc) ターゲットでバイナリサイズ削減
- pure embedded(allocator なし)は対象外 — `String` / `Vec` を多用
  しているため、本気で組み込みするなら別形式(Protobuf / FlatBuffers)
  を検討すべき

## 2. 現状の `std` 依存マップ

| ファイル | std 依存 | no_std + alloc 化の難易度 |
|---|---|---|
| `schema.rs` | `std::collections::BTreeMap`, `std::f64::consts::*` | 低: `alloc::collections::BTreeMap` + `core::f64::consts` |
| `report.rs` | `serde` のみ | 低: 既に std-light |
| `source.rs` | `std::path::PathBuf`, `std::fs::*`(`FileSystemSource` 内のみ) | 中: `FileSystemSource` を `#[cfg(feature = "std")]` で gate |
| `parse.rs` | `std::collections::{HashMap, HashSet, BTreeMap}`, `std::any::type_name` | 低: `alloc::collections::*`, `type_name` 削除可 |
| `build.rs` | `std::collections::{HashMap, HashSet, VecDeque}` | 低: `alloc::collections::*` |
| `write.rs` | `toml::to_string_pretty` | **高**: `toml` クレートが std 必須 — 別パーサ要 |
| `mesh_load.rs` | `std::io::Cursor`(MeshData::from_stl_bytes 経由) | 中: `stl_io` も std 必須なので一緒に判断 |
| `mod.rs` | `std::path::Path`, `std::fs::*`, `std::error::Error` | 中: Layer 3 を `#[cfg(feature = "std")]` で gate |

最大の障害は **TOML パーサの選定**。

## 3. TOML パーサ候補比較

| クレート | バージョン (1.x 時点) | std 必要 | 備考 |
|---|---|---|---|
| **`toml`** (現状採用) | 0.8 | ✓ | 公式デファクト、`toml_edit` ベース。`serde` 統合最強 |
| **`toml_edit`** | 0.22 | ✓ | format-preserving editor、`indexmap` 依存(std) |
| **`basic-toml`** | 0.1 | ✓ (alloc) | 公式の serde_toml 後継 fork、シンプル、ただし std/alloc サポートは要確認 |
| **`tinytoml`** | (検索要) | 不明 | 名前から推測すると軽量化指向 |
| **`tomling`** | (検索要) | 不明 | 比較的新しい代替実装 |

**結論**(推測): 公式 `toml` クレートの no_std サポートを待つか、
独自に minimum subset パーサを書くのが現実的。今回は調査のみ。

### 現実的な選択肢

1. **そのまま `toml` を使い続けて std 必須**
   - Pros: 既存実装そのまま、互換性最高
   - Cons: 組み込み・WASM への適用不可
2. **`basic-toml` 系の軽量パーサに切り替える**
   - Pros: alloc-only で動作する可能性
   - Cons: serde 統合の質、バグ密度、機能網羅率が未知数
3. **JSON / CBOR / Postcard など別形式を no_std エクスポートに併設**
   - Pros: 組み込み配布専用に最適化、std 版 .misa は維持
   - Cons: フォーマット二重管理、スキーマ同期コスト

## 4. 推奨アプローチ

**Phase 1**(本セッション完了 — このドキュメント): 調査・現状整理

**Phase 2**(将来): no_std 対応の最小ステップ
- `Cargo.toml` に `default = ["std"]` / `std = []` / `alloc` features 追加
- `schema.rs` / `report.rs` / `parse.rs` / `build.rs` を `alloc` 移行
  (`std::collections::*` → `alloc::collections::*`)
- `source.rs` の `FileSystemSource` を `#[cfg(feature = "std")]` で gate
- `mod.rs` の `load` / `save` を `#[cfg(feature = "std")]` で gate
- `write.rs` は **`std` feature が無いと使えない**ことを明示
  (`toml` クレートの no_std 対応待ち、または別実装の決定待ち)

**Phase 3**(将来): no_std + alloc + バイナリ配布対応
- `StaticBundleSource` + `include_bytes!` で `.misa` を組み込み
- 軽量 TOML パーサへの切り替え or 別フォーマット導入
- 実機(STM32 / ESP32 等)での動作確認

## 5. 今すぐ取れる無痛な準備

実装を後回しにする前提でも、以下の準備をしておくと将来の no_std 化が
楽になる:

- [ ] `schema.rs` 内の `std::collections::BTreeMap` を `alloc::collections::BTreeMap` でも動くよう抽象化(`pub use std::collections::BTreeMap as BTreeMap;` 等)
- [ ] `parse.rs` / `build.rs` の `HashMap` 利用箇所を `BTreeMap` に置き換え可能か検証(順序依存ない箇所だけ)
- [ ] `write.rs` のシグネチャに `#[cfg(feature = "std")]` を付ける準備
- [ ] integration test に `#[cfg(feature = "std")]` ガードを設けて
      no_std build でも compile 通るようにする土台

これらは std 環境で動作変化なく適用可能で、将来 Phase 2 を開始する際の
変更差分を小さくする。

## 6. 結論

- `no_std + alloc` 化は技術的に可能だが、**TOML パーサ選定**で大きく
  左右される
- 現状の `toml` クレートに据え置く限り、`std` 必須は妥当な選択
- 組み込み配布が現実のニーズになった時点で Phase 2 着手

未着手のまま `refactor_20260502.md` §9.1 のチェックリストに残置。
本ドキュメントが調査の出発点になる。
