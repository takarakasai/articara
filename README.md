# articara

A high-performance robot model editor built in Rust. articara focuses on
seamless URDF/SRDF manipulation and kinematic structure visualization.

## Features

- **Multi-format model loading** — URDF, MJCF (MuJoCo XML), USD, SDF; SRDF-style collision allow/disallow lists are preserved via `.misarta.toml`.
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

When using the `mujoco` feature, configure the MuJoCo download directory:

```
export MUJOCO_DOWNLOAD_DIR="$HOME/.mujoco"
echo 'export MUJOCO_DOWNLOAD_DIR="$HOME/.mujoco"' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH="$HOME/.mujoco/mujoco-3.6.0/lib:$LD_LIBRARY_PATH"' >> ~/.bashrc
```

## License

Copyright 2026 Takara Kasai

Licensed under the Apache License, Version 2.0 ([LICENSE](LICENSE) or
<http://www.apache.org/licenses/LICENSE-2.0>).
