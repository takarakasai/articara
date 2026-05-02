# articara

A high-performance robot model editor built in Rust. articara is built around
**`.misa`** — its own native master format that captures the full kinematic
tree, geometry, materials, sensors, actuators, mimic / loop-closure
constraints, and editor metadata (poses, sequences, gaits, home pose) in a
single TOML file. URDF / SDF / MJCF / USD are supported as derivative
import / export targets.

## Master format: `.misa`

`.misa` is articara's canonical on-disk representation. A single file holds
everything `RobotModel` carries; round-tripping through `from_misa` /
`to_misa` is lossless. Mesh files live alongside as a `meshes/` sibling
directory. URDF / SDF / MJCF / USD remain available for interop with other
tooling but are treated as **lossy derivatives** of the `.misa` master.

- Schema reference: [`doc/misa_schema.md`](doc/misa_schema.md)
- Format comparison: [`doc/comparison.md`](doc/comparison.md)
- Design rationale: [`doc/refactor_20260502.md`](doc/refactor_20260502.md)
- Sample: [`sample/namiashi_description/namiashi.misa`](sample/namiashi_description/)
- Quick conversion: `cargo run --example convert_to_misa -- <urdf> <out.misa>`

## Features

- **`.misa` master format** — single-file robot description in TOML
  (kinematics + geometry + sensors + actuators + editor metadata),
  via the [`misarta::native`](misarta/src/native/) module.
- **Multi-format model loading** — `.misa` (native), URDF, MJCF (MuJoCo XML),
  USD, SDF. Legacy URDF + `.misarta.toml` sidecar workflows continue to work.
- **Rigid body dynamics** — O(n) Featherstone algorithms via [`misarta`](misarta/) (kinematics, dynamics, autodiff-ready).
- **3D viewer & GUI** — `eframe` + `glow` renderer with `egui_plot` overlays (default `gui` feature).
- **Physics backend** — optional MuJoCo integration (`--features mujoco`, requires `mujoco-rs`).
- **Scripting** — Rhai-based scene/robot scripting (default `scripting` feature; see `examples/script_repl.rs`).
- **Gait planning** — quadruped gait generation via [`quadruped-gait`](quadruped-gait/).
- **Jump simulation** — native ([`jump-sim-runner`](jump-sim-runner/)) and WASM ([`jump-sim-wasm`](jump-sim-wasm/)) builds.
- **Plugin API** — extension interface in [`plugin-api`](plugin-api/).

## Workspace layout

| Crate                | Purpose                                              |
| -------------------- | ---------------------------------------------------- |
| `articara`           | Main editor / viewer application                     |
| [`misarta`](misarta/)              | Rigid body kinematics & dynamics library |
| [`quadruped-gait`](quadruped-gait/)      | Quadruped gait planning                  |
| [`plugin-api`](plugin-api/)              | Plugin / extension interface             |
| [`jump-sim-runner`](jump-sim-runner/)    | Native jump-simulation harness           |
| [`jump-sim-wasm`](jump-sim-wasm/)        | WASM build of jump simulation            |

## MuJoCo setup

The `mujoco` feature requires **MuJoCo 3.8.0** at runtime to match the
`mujoco-rs` 4.0.x FFI bindings. See [MUJOCO_SETUP.md](MUJOCO_SETUP.md)
for full per-platform instructions; the minimal Linux quickstart is:

```bash
# Download MuJoCo 3.8.0
mkdir -p ~/.mujoco && cd ~/.mujoco
wget https://github.com/google-deepmind/mujoco/releases/download/3.8.0/mujoco-3.8.0-linux-x86_64.tar.gz
tar -xzf mujoco-3.8.0-linux-x86_64.tar.gz

# Build & run with the cross-platform xtask helper (auto-detects the
# install and exports MUJOCO_DYNAMIC_LINK_DIR for cargo).
cargo xtask run --release --features mujoco
```

`cargo xtask` is the recommended entry point on macOS / Linux / Windows.
A bare `cargo run --features mujoco` works too if you've sourced
`./setup-mujoco.sh` first (PowerShell: `. .\setup-mujoco.ps1`).

## License

Copyright 2026 Takara Kasai

Licensed under the Apache License, Version 2.0 ([LICENSE](LICENSE) or
<http://www.apache.org/licenses/LICENSE-2.0>).
