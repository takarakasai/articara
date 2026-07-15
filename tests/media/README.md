# Media

- `go2_wbc_trot_walk.mp4` — visualization of `wbc_walk_go2.rs`'s
  `go2_wbc_forward_command_advances_body_force_space_active_set` test:
  the real Go2 model (real mesh, real ~15.6 kg mass, real actuator
  gains) walking Trot under misa-wbc's WBC (ForceSpace + ActiveSet)
  in actual MuJoCo physics — every body's world pose queried directly
  from MuJoCo per tick, not a kinematic replay.
- `render_go2_walk.py` — regenerates the video. Needs:
  1. A trace CSV, written by `wbc_walk_go2.rs` when run with
     `WBC_WALK_CSV_OUT=<path> cargo test --release --features mujoco
     --test wbc_walk_go2 go2_wbc_forward_command_advances_body_force_space_active_set
     -- --nocapture` (remember to `source ./setup-mujoco.sh` first).
  2. `go2_mesh_manifest.csv` + `go2_topology.csv` — written by the
     *misa-wbc* repo's `go2_leg_singularity_demo` example (see
     `misa-wbc/examples/models/README.md`). This script joins the two
     by parent-joint index to resolve each mesh piece's real link name
     (`FL_hip`, `FR_calf`, …), since the trace above is keyed by
     MuJoCo body *name*, not misarta joint index.
