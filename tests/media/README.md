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
- `go2_velocity_staircase_fine.mp4` / `staircase_fine_tracking.png` —
  `wbc_walk_go2.rs`'s `go2_wbc_velocity_staircase_fine` resweep:
  0 to 1.0 m/s in 0.05 m/s steps over 60s (21 levels, ~2.86s each) —
  finer resolution around the ceiling the coarse sweep above found.
  Peak measured speed (~0.34 m/s) lands almost exactly at the
  calculated footstep-clamp threshold (0.5 m/s commanded, where the
  Raibert step `cmd_vx * 0.5 * stance_duration` first hits
  `0.5 * max_step_length_m` in `mpc_controller.rs::compute_mpc_footstep`),
  then declines gently with no reversal (the milder command range
  never drives the capture-point feedback term as far out as the 5 m/s
  sweep did). Body stays stable throughout (min z 0.229-0.240m). See
  `ref/wbc_comparison.md` Sec.5s.
- `horizon_comparison_tracking.png` — same fine 0-1.0 m/s staircase
  re-run with the SRBD MPC's `horizon_steps x dt_per_step` overridden
  after `GaitController::build` (`go2_wbc_velocity_staircase_fine_long_horizon`
  / `_full_horizon` tests): 0.3s (default) vs 0.6s vs 1.0s
  (`ref/legged_control`'s OCS2 `mpc.timeHorizon`-matched). 0.6s tracks
  near-ideal up to ~0.5 m/s with no reversal; 1.0s diverges into
  sustained *backward* walking at high commanded speed instead of
  saturating. See `ref/wbc_comparison.md` Sec.5t.
- `horizon_sweet_spot.png` — sweeps `dt_per_step` finely between the
  known-good 0.6s and known-bad 1.0s (`go2_wbc_velocity_staircase_fine_horizon_sweep`
  / `_horizon_zoom` tests), plotting tracking at the cmd_vx=0.5 m/s
  staircase level against horizon length: good tracking exists only
  in a narrow ~0.60-0.65s band, not a broad plateau — just past either
  edge (0.58s or 0.70s) the behavior reverses into backward walking.
  A `cycle_period_s` resonance hypothesis was tested and not confirmed
  (`go2_wbc_velocity_staircase_fine_cycle_resonance`); see
  `ref/wbc_comparison.md` Sec.5t for the full writeup and open
  questions.
- `body_height_sweep.png` — same fine 0-1.0 m/s staircase, MPC horizon
  and gait cycle held at their defaults, sweeping the standing-height
  bias fraction instead (`go2_wbc_velocity_staircase_fine_body_height_sweep`,
  standing height 0.13-0.30 m). Crouching generally raises peak
  tracking, but one height (~0.20 m) is qualitatively different from
  the rest: instead of peaking then rolling off (or, at the most
  crouched setting, reversing), it holds a near-flat plateau all the
  way to the top of the commanded range — the same "reversal-free
  plateau" signature Sec.5t found from a *longer MPC horizon*, this
  time produced by a completely independent parameter left at its
  default horizon. See `ref/wbc_comparison.md` Sec.5u.
- `height_horizon_combo.png` — tests whether Sec.5t's (0.6s horizon)
  and Sec.5u's (h=0.20m) independently-found reversal-free plateaus
  stack when combined (`go2_wbc_velocity_staircase_fine_horizon_and_height_combo`).
  They don't: the combined run tracks *worse* than either solo
  configuration, dipping to ~0.035 m/s around cmd_vx=0.6-0.65 (both
  solo runs hold ~0.35-0.5 m/s there) before partially recovering by
  cmd_vx=1.0. The two fixes interact rather than add — see
  `ref/wbc_comparison.md` Sec.5v.
- `full_centroidal_comparison.png` — first look at
  `GaitMode::FullCentroidal` (24-state, `joint_q` folded into the MPC
  state so the per-leg moment arm updates within the horizon, plus a
  real multi-iteration SQP loop) against the `GaitMode::Mpc` (SRBD)
  baseline, same fine 0-1.0 m/s staircase, no height/horizon tuning
  (`go2_wbc_velocity_staircase_fine_full_centroidal`). Enabling
  `legged_control_parity` alone reproduces the reversal-free plateau
  Sec.5t's 0.6s-horizon search had to hunt for, with zero extra
  tuning; layering `use_mpc_predicted_footstep` on top instead
  destabilizes it (sustained backward walking, roll up to ~15°). See
  `ref/wbc_comparison.md` Sec.5w.
- `dynamic_joint_q_comparison.png` — implements and tests the D3.3.5a
  reversal: `FullCentroidalMpcGaitController::dynamic_joint_q_reference`
  (new, opt-in, in quadruped-gait) makes the MPC's joint_q tracking
  reference a real per-horizon-step trajectory (sampled from the same
  open-loop swing/stance foot curve `tick()` uses) instead of a flat
  hold, against the Sec.5w `legged_control_parity` baseline
  (`go2_wbc_velocity_staircase_fine_full_centroidal_dynamic_q`). The
  two curves are nearly identical — the wiring works but
  `FullCentroidalMpcConfig::q_diag`'s joint_q weight (0.1, deliberately
  light to avoid fighting the stance no-slip constraint) is too weak
  for the now-dynamic reference to visibly change behavior either way.
  See `ref/wbc_comparison.md` Sec.5x.
- `sqp_tuning_comparison.png` — sweeps `FullCentroidalMpcConfig`'s
  `sqp_iterations` at two horizon lengths
  (`go2_wbc_velocity_staircase_fine_full_centroidal_sqp_tuning`),
  motivated by `ref/ocs2` desk research into legged_control's
  real-time-iteration-style `sqp_iterations=1`. At the true
  auto-detected default horizon (0.3s), more iterations (1→3) tracks
  clearly better; at 0.6s, more iterations instead flips into
  sustained backward walking, where fewer iterations is what's needed
  for a stable plateau — the same "sign flips with horizon length"
  pattern Sec.5v found for height×horizon. See `ref/wbc_comparison.md`
  Sec.5y (also documents a mislabeled-baseline mistake caught and
  corrected mid-experiment).
- `render_go2_walk.py` — regenerates any of the three videos. Needs:
  1. A trace CSV, written by `wbc_walk_go2.rs` when run with
     `WBC_WALK_CSV_OUT=<path> cargo test --release --features mujoco
     --test wbc_walk_go2 <test-name> -- [--ignored] --nocapture`
     (remember to `source ./setup-mujoco.sh` first). For a staircase
     trace, also pass `--staircase-step-s <total_time_s / n_levels>`
     plus `--staircase-step-mps`/`--staircase-max-mps` matching
     whichever staircase constructor produced it, so the on-screen
     commanded-speed readout tracks correctly (a first pass at the
     fine sweep's video shipped with these still defaulted to the
     coarse sweep's 0.5/5.0 — caught and fixed before committing;
     the underlying motion data was never affected, only the on-
     screen text).
  2. `go2_mesh_manifest.csv` + `go2_topology.csv` — written by the
     *misa-wbc* repo's `go2_leg_singularity_demo` example (see
     `misa-wbc/examples/models/README.md`). This script joins the two
     by parent-joint index to resolve each mesh piece's real link name
     (`FL_hip`, `FR_calf`, …), since the trace above is keyed by
     MuJoCo body *name*, not misarta joint index.
