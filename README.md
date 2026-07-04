# articara

[![CI](https://github.com/takarakasai/articara/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/takarakasai/articara/actions/workflows/ci.yml)

A high-performance robot model editor built in Rust. articara is built around
**`.misa`** — its own native master format that captures the full kinematic
tree, geometry, materials, sensors, actuators, mimic / loop-closure
constraints, and editor metadata (poses, sequences, gaits, home pose) in a
single TOML file. URDF / SDF / MJCF / USD are supported as derivative
import / export targets.

## Build

### Prerequisites

- **Rust 1.85 or newer** (the workspace uses edition 2024). Install via
  [rustup](https://rustup.rs/):
  `curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh`
- **MuJoCo 3.8.0** — only needed for the `--features mujoco` physics
  backend; see [MuJoCo setup](#mujoco-setup).

The default `gui` build also needs the usual OpenGL / X11 / Wayland
development libraries provided by your distro's desktop toolchain.

### Clone & build

`misarta` (the kinematics/dynamics core) is a **git submodule**, so clone
with `--recursive`:

```bash
git clone --recursive git@github.com-takarakasai:takarakasai/articara.git
cd articara
cargo build            # GUI build (default features)
```

If you forgot `--recursive` (or pulled changes that bump the submodule):

```bash
git submodule update --init --recursive
```

To use the MuJoCo physics backend, see [MuJoCo setup](#mujoco-setup) for
the extra runtime dependency and build command.

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
- **Rigid body dynamics** — O(n) Featherstone algorithms via [`misarta`](https://github.com/takarakasai/misarta) (kinematics, dynamics, autodiff-ready).
- **3D viewer & GUI** — `eframe` + `glow` renderer with `egui_plot` overlays (default `gui` feature).
- **Physics backend** — optional MuJoCo integration (`--features mujoco`, requires `mujoco-rs`).
- **Scripting** — Rhai-based scene/robot scripting (default `scripting` feature; see `examples/script_repl.rs`).
- **Gait planning + closed-loop control** — quadruped gait generation
  + 3 MPC controllers (CHAMP open-loop, body-root SRBD, centroidal-SRBD)
  + Hierarchical WBC + 18-state Linear Kalman Filter via
  [`quadruped-gait`](quadruped-gait/). See
  [Gait & control modes](#gait--control-modes) below.
## Workspace layout

| Crate                | Purpose                                              |
| -------------------- | ---------------------------------------------------- |
| `articara`           | Main editor / viewer application                     |
| [`misarta`](https://github.com/takarakasai/misarta) | Rigid body kinematics & dynamics library (git dependency) |
| [`misarta-py`](misarta-py/)        | Python bindings for `misarta` (PyO3 + maturin) |
| [`quadruped-gait`](quadruped-gait/)      | Quadruped gait planning                  |

## Gait & control modes

`quadruped-gait` ships three interchangeable `GaitMode` controllers, all
driven by the same trot phase scheduler and Hierarchical WBC tracking
layer. They differ only in **how the desired body acceleration is
predicted** before being passed to the WBC:

| `GaitMode` | Predictor | State (continuous) | When to use |
|---|---|---|---|
| `Champ` | Open-loop CHAMP heuristic | — | Baseline / no MPC build |
| `Mpc` | Body-root **SRBD MPC** | `[v_body; ω_body; p; rpy]` (12) | Default — robust on flat ground |
| `CentroidalSrbd` | **Centroidal-SRBD MPC** | `[v_com; ω_world; p_base; e_zyx; g]` (13) | CoM-offset platforms, asymmetric inertia |

`CentroidalSrbd` corresponds to legged_control's `centroidalModelType=1`
and uses CoM-aware moment arms (`r_i = foot_i − CoM_world`) plus
linearized Euler-ZYX kinematics. It re-linearizes within a single MPC
solve via **SQP multiple shooting** (`sqp_iterations`, default 3).

The Hierarchical WBC tracks the predicted base acceleration with three
priority levels (contact + friction → joint accel + swing-leg PD →
contact-force regularization), solved as a sequence of QPs in
[`quadruped-gait/src/wbc/`](quadruped-gait/src/wbc/). Body pose is
estimated by an 18-state Linear Kalman Filter (`LkfPipeline`) fusing
IMU + leg-kinematics. Stance/swing phase is driven by physical contact
sensors via `ContactDrivenPhase`.

Switching modes from a Rhai script:

```rhai
robot.set_gait_mode("centroidal");  // or "mpc" / "champ"
```

References:

- Architecture & phase plan: [`doc/mpc_wbc_gait_control.md`](doc/mpc_wbc_gait_control.md)
- D1 (centroidal MPC) / D2 (SQP) implementation log: [`doc/recent_features.md`](doc/recent_features.md)
- Regression suite: `cargo test -p articara --test integration_walk -- --ignored`

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
