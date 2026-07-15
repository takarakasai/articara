# Media

- `go2_wbc_trot_walk.mp4` — visualization of `wbc_walk_go2.rs`'s
  `go2_wbc_forward_command_advances_body_force_space_active_set` test:
  the real Go2 model (real mesh, real ~15.6 kg mass, real actuator
  gains) walking Trot under misa-wbc's WBC (ForceSpace + ActiveSet)
  in actual MuJoCo physics — every body's world pose queried directly
  from MuJoCo per tick, not a kinematic replay.
- `go2_velocity_staircase.mp4` / `staircase_tracking.png` —
  `wbc_walk_go2.rs`'s `go2_wbc_velocity_staircase` stress test
  (`#[ignore]`d, not a regression check): commands 0 to 5 m/s in
  0.5 m/s steps over 30s (11 levels, ~2.73s each). Forward tracking
  saturates around 0.46 m/s near a 1.0-1.5 m/s command and degrades
  past that, going *negative* (-0.17 m/s, drifting backward) at the
  5.0 m/s command — but the body never falls (min z stays
  0.229-0.249m throughout, well above the 0.15m fall threshold).
  A graceful-saturation failure mode, not a catastrophic one; the
  low top speed reflects this Trot config's tuning (footstep
  planner / MPC horizon), not a hard WBC ceiling — see
  `ref/wbc_comparison.md` Sec.5r.
- `render_go2_walk.py` — regenerates either video. Needs:
  1. A trace CSV, written by `wbc_walk_go2.rs` when run with
     `WBC_WALK_CSV_OUT=<path> cargo test --release --features mujoco
     --test wbc_walk_go2 <test-name> -- [--ignored] --nocapture`
     (remember to `source ./setup-mujoco.sh` first). Pass
     `--staircase-step-s 2.7273` when rendering the staircase trace
     so the on-screen commanded-speed readout tracks correctly.
  2. `go2_mesh_manifest.csv` + `go2_topology.csv` — written by the
     *misa-wbc* repo's `go2_leg_singularity_demo` example (see
     `misa-wbc/examples/models/README.md`). This script joins the two
     by parent-joint index to resolve each mesh piece's real link name
     (`FL_hip`, `FR_calf`, …), since the trace above is keyed by
     MuJoCo body *name*, not misarta joint index.
