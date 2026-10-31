//! Environment -> `Config`: the walker's knobs by name. The table is
//! `misa_biped::kv` (moved there 2026-10-31, with the presets' check against
//! their controls); `main.rs` calls it with `|k| std::env::var(k).ok()`.
//! Keys the walker does not own (the rig, the scenario, the command, the
//! diagnostics) are read in `main.rs`.
pub use misa_biped::kv::*;
