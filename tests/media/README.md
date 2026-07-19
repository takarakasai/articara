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
- `taskspace_weight_comparison.png` — implements and tests
  `FullCentroidalMpcConfig::joint_vel_nominal_jacobian` (new, opt-in,
  in quadruped-gait): maps the flat per-joint `joint_v` cost through
  each leg's fixed-nominal-pose Jacobian
  (`R_jointspace = J_nom^T · diag(r_taskspace) · J_nom`),
  legged_control/OCS2's own technique (confirmed against `ref/ocs2`),
  against the Sec.5y true default
  (`go2_wbc_velocity_staircase_fine_full_centroidal_taskspace_weight`).
  At the same overall weight scale (`r_taskspace=[1,1,1]`), the
  Jacobian mapping alone makes mid-to-high-speed tracking clearly
  worse (briefly reversing around cmd_vx=0.75-0.80) — a third instance
  of a legged_control design choice not transferring cleanly to our
  formulation. See `ref/wbc_comparison.md` Sec.5z.
- `true_coupling_comparison.png` — implements and tests desk-research
  gap ①: `FullCentroidalMpcConfig::enable_true_centroidal_coupling`
  (new, opt-in, in quadruped-gait) adds an additive bias term (from
  `misarta`'s real CRBA-based centroidal momentum matrix, not a state-
  representation change) coupling joint velocity/acceleration into the
  base's predicted motion, matching OCS2's `FullCentroidalDynamics`
  physics
  (`go2_wbc_velocity_staircase_fine_full_centroidal_true_coupling`).
  Same pattern as ②/③: tracks the Sec.5w/5y baseline closely up to
  cmd_vx≈0.45, then degrades sharply and fully reverses past
  cmd_vx≈0.70. **Superseded by `kcap_confound_comparison.png` below —
  the reversal turned out to be a confound, not a property of ①
  itself.** See `ref/wbc_comparison.md` Sec.5aa for the (now-corrected)
  original write-up.
- `kcap_confound_comparison.png` — the correction: `k_capture` (the
  capture-point footstep-feedback gain, default `0.05`) was tuned in an
  unrelated 2026-05-15 disturbance-recovery experiment against the
  pre-FullCentroidal SRBD plant, and legged_control's own code uses
  `k_capture=0` (its reference-tracking loop closes differently — see
  `DEFAULT_CAPTURE_POINT_GAIN_S`'s doc comment). None of ①②③ ever
  touched this gain
  (`go2_wbc_velocity_staircase_fine_full_centroidal_true_coupling_kcap_zero`).
  Re-running ① with `k_capture=0` **fully eliminates the reversal** —
  tracking matches or slightly exceeds the healthy baseline across the
  whole 0-1.0 m/s range, and `k_capture=0` alone (no coupling) is
  *also* healthy (slightly better than the old default even without
  ①). So the Sec.5aa "①②③ all fail the same way" conclusion was wrong
  — all three shared this one un-retuned leftover gain as a confound.
  See `ref/wbc_comparison.md` Sec.5ab for the full correction.
- `kcap_recheck_23_comparison.png` — extends the Sec.5ab check to ②
  and ③
  (`go2_wbc_velocity_staircase_fine_kcap_zero_recheck_2_3`): both
  reversals **also fully resolve** at `k_capture=0` — ③'s worst case
  (20x0.030s sqp=3, which reversed to -0.356 at cmd_vx=1.0 in Sec.5y)
  becomes the best tracking result in this entire test series (a flat
  ~0.46-0.48 plateau across cmd_vx=0.5-1.0) once the gain is fixed. All
  three (①②③) confirmed independently to share the same confound —
  none of the borrowed legged_control/OCS2 physics was actually at
  fault. See `ref/wbc_comparison.md` Sec.5ac for the conclusive
  write-up.
- `123_reeval_k0.png` — the natural follow-up: confound removed, do
  ①②③ actually *help* on top of the now-healthy `k_capture=0`
  baseline, or just no-longer-hurt? Reuses already-collected Sec.5ab/
  5ac data (no new runs). Answer differs per term: ① tracks the
  baseline almost exactly (neutral), ② still drifts down at high speed
  even without the confound (~24% below baseline at cmd_vx=1.0 — a
  genuine standalone downside, not just the old reversal), ③ (0.6s
  horizon, sqp=3) is mildly *better* than baseline and the smoothest,
  most stable plateau in this entire test series. See
  `ref/wbc_comparison.md` Sec.5ae.
- `base_pos_weight_sweep.png` — a broad legged_control/OCS2 survey
  (beyond ①②③) found legged_control weights base **position**
  tracking at 1000/1000/1500 (x/y/z) against the same velocity-ramp
  reference we already build, vs our own `q_diag[6]=0`/`q_diag[7]=5`
  (`q_diag[8]`, z, is already 50). Sweeping `q_diag[6..8]` on the
  healthy `k_capture=0` baseline
  (`go2_wbc_velocity_staircase_fine_base_pos_weight`) found a
  non-monotonic dip, not a clean win: `(25,25)` degrades noticeably at
  high speed (cmd_vx=1.0: 0.189 vs the default's 0.460), while `(50,50)`
  (matching the z weight) recovers to ≈baseline — no configuration
  tried beat the current default. See `ref/wbc_comparison.md` Sec.5af
  for the full write-up and the broader legged_control survey it came
  from.
- `swing_pd_maxforce.png` — the last two survey items. ⑤: `legged_wbc`'s
  `swingLegTask.kp=350, kd=37` vs our `WbcPipeline`'s `swing_kp=80,
  swing_kd=8` (`go2_wbc_velocity_staircase_fine_swing_pd_gain`) —
  raising toward legged_control's actual value makes high-speed
  tracking monotonically *worse* (cmd_vx=1.0: 0.275 vs the default's
  0.460 at the full 350/37 value; halfway at 175/18.5 is ≈neutral). ⑥:
  removing our `max_normal_force=200N` cap entirely
  (`go2_wbc_velocity_staircase_fine_max_normal_force`) has essentially
  no effect — the cap was never binding in this speed range (200N×4
  legs ≫ the robot's ~153N weight). Across all six items (①-⑥) tested
  this session, only ③ (MPC horizon + SQP iterations) was a genuine
  win; the rest are neutral-to-harmful when pushed toward
  legged_control's literal values. See `ref/wbc_comparison.md` Sec.5ag
  for the full write-up and the ①-⑥ summary.
- `swing_pd_jacobian_converted.png` — the ⑤ story's resolution.
  legged_control's `swingLegTask.kp=350, kd=37` is a **Cartesian-space**
  (foot position/velocity, metre-error) gain; our own `WbcPipeline`'s
  `swing_kp`/`swing_kd` is **joint-space** (radian-error) — comparing
  "350" to "80" directly was never dimensionally valid.
  `go2_diag_swing_pd_gain_jacobian_conversion` computes Go2's actual FL
  leg Jacobian at its nominal stance pose (singular values
  0.317/0.280/0.133 m/rad, Frobenius norm 0.443) and uses it to convert
  legged_control's gain into the joint-space equivalent (≈111/12 or
  ≈155/16, depending on which norm). Tested on the healthy
  `k_capture=0` baseline
  (`go2_wbc_velocity_staircase_fine_swing_pd_gain_jacobian_converted`):
  both converted values track almost identically to the default
  (80/8) — the degradation the raw 350/37 import caused (Sec.5ag)
  disappears entirely once the units are converted correctly. See
  `ref/wbc_comparison.md` Sec.5ai for the full write-up, including a
  correction to Sec.5ah's over-broad claim that ②④⑤ were all A1-number
  imports — only ⑤ actually was.
- `max_step_length_sweep.png` — the biggest single win in this test
  series. The observed ~0.46-0.48 m/s velocity-tracking plateau
  matches almost exactly the Raibert footstep planner's own
  theoretical kinematic ceiling, `v_max = max_step_length_m /
  (cycle_period_s * duty_factor) = 0.10 / 0.2 = 0.5 m/s` — i.e. it's
  this specific `GaitConfig::trot()` setting's own limit (Go2's leg
  reach is ~0.426m, so 0.10m is only ~23% of it), not an algorithmic
  bug or a property of Trot as a gait. Sweeping `max_step_length_m`
  on the healthy `k_capture=0` baseline
  (`go2_wbc_velocity_staircase_fine_max_step_length`) confirms this
  directly: all three curves overlap exactly below their respective
  theoretical thresholds (0.5/0.75/1.0 m/s), then each one alone peels
  off upward right at its own threshold. At `0.20m`, tracking reaches
  0.852 at cmd_vx=1.0 (0.881 at cmd_vx=0.95) — up from 0.460 (46%) at
  the 0.10m default. See `ref/wbc_comparison.md` Sec.5ak for the full
  write-up.
- `true_coupling_at_new_speed.png` — re-evaluates ① on top of the new
  `max_step_length_m=0.20` baseline
  (`go2_wbc_velocity_staircase_fine_max_step_length_true_coupling`).
  Sec.5ae characterized ① as "neutral," but that test never actually
  reached much past ~0.48 m/s (the old footstep-clamp ceiling), so the
  swing-leg momentum coupling ① models was barely exercised. Once
  actually reaching ~0.85 m/s (Sec.5ak's higher ceiling), ① degrades
  clearly: the two curves track identically up to cmd_vx≈0.6-0.65,
  then diverge — with ① declining to 0.276 at cmd_vx=1.0 vs the
  baseline's 0.852. A parameter's "neutral" verdict can depend on
  whether the test ever actually reaches the speed range where its
  effect becomes physically significant. See `ref/wbc_comparison.md`
  Sec.5al for the full write-up, including the theory-vs-measured gap
  analysis (Sec.5ak's plateau ratio: 96%→87%→88% as
  `max_step_length_m` rises, plausibly from the no-integral-term
  structural gap plus increasing body roll disturbance at longer
  strides) that motivated re-testing ① here.
- `roll_pitch_weight_sweep.png` — tests the natural follow-up to
  Sec.5al's rising-peak-roll observation: does raising
  `q_diag[9]`/`q_diag[10]` (roll/pitch attitude weight, default 25/25)
  reduce that disturbance and narrow the theory-vs-measured gap?
  (`go2_wbc_velocity_staircase_fine_roll_pitch_weight`, on the
  `max_step_length_m=0.20` baseline). Counter-intuitively, no: both
  50/50 and 100/100 make tracking *and* stability worse — peak roll
  itself rises further (0.10-0.11 → 0.13-0.15) rather than shrinking,
  and high-speed tracking reverses into backward walking
  (cmd_vx=1.0: -0.183 at 50/50 vs the default's +0.852). Plausible
  mechanism (unconfirmed): the reference trajectory holds roll/pitch
  at a constant near-zero target, and Trot naturally involves cyclic
  body sway — over-weighting deviation from that flat reference may
  fight the gait's natural motion and provoke the correction it was
  meant to prevent. Default (25/25) stays best; see
  `ref/wbc_comparison.md` Sec.5am for the full write-up.
- `ceiling_coarse_vs_fine.png` — finds the actual ceiling of the
  current best config (`legged_control_parity=true, k_capture=0,
  max_step_length_m=0.20`) by widening the staircase to 0-2.0 m/s at
  0.10 m/s steps
  (`go2_wbc_velocity_staircase_coarse_max_step_length_ceiling`), since
  Sec.5ak's 0-1.0 m/s fine sweep hadn't clearly plateaued by its top
  command. Found a real discrepancy: at the same cmd_vx=1.0, the
  coarse (0.10-step) sweep measures 0.567 vs the fine (0.05-step)
  sweep's 0.852 — the two curves overlap almost exactly up to
  cmd_vx≈0.6, then diverge. The coarse sweep reveals the *actual*
  behavior: a peak of 0.605 at cmd_vx=0.7 (86% ratio), a gentle
  decline, then a clear reversal starting around cmd_vx≈1.4, settling
  near -0.86 by cmd_vx=1.8-2.0 — the same qualitative
  peak-then-decline-then-reversal shape Sec.5r/5s found for the old
  `max_step_length_m=0.10` setting, just shifted to a higher speed
  range. Sec.5ak's "still climbing at cmd_vx=1.0" result likely
  reflects some not-yet-understood effect of the fine staircase's
  smaller steps letting the system "sneak past" the reversal that a
  larger, more abrupt step change triggers — flagged as an open
  question, not resolved. See `ref/wbc_comparison.md` Sec.5an.
- `bound_flight_phase_check.png` — Canter/Gallop scoping: does a
  genuine flight phase (0 legs in stance) survive real MuJoCo dynamics
  at all? `GaitConfig::bound()`'s duty factor lowered from its 0.5
  default to 0.35 (30% of each cycle airborne, twice per cycle, per
  `go2_diag_bound_duty_factor_flight_phase_sweep`'s schedule-level
  confirmation), run through `GaitMode::FullCentroidal` +
  `legged_control_parity` + `k_capture=0` (the established healthy
  Trot baseline) at cmd_vx=0.3
  (`go2_wbc_bound_flight_phase_duty_sweep`). Good news: the flight
  phase itself doesn't crash or diverge — both duty=0.50 (no flight)
  and duty=0.35 (30% flight) stay finite and well above the fall
  threshold (min_z 0.216-0.219m vs 0.15m). Bad news, more fundamental:
  **even the no-flight duty=0.50 baseline reverses hard** against the
  +0.3 m/s command (meas_vx=-0.166) — the Trot-tuned configuration
  (`legged_control_parity`, `k_capture=0`, `max_step_length_m`, …)
  doesn't transfer to Bound at all, flight phase or not. Oddly, adding
  the flight phase *reduces* the reversal (-0.021 at duty=0.35) rather
  than worsening it — unexplained, flagged for follow-up. WBC's HoQp
  solver also logged frequent `Infeasible`/`MaxIterations` warnings in
  both trials, suggesting the task formulation itself doesn't fit
  Bound's faster cycle well. See `ref/wbc_comparison.md` Sec.5ao.
- `bound_baseline_survey.png` — isolates *why* Bound reverses:
  is it the Trot-specific tuning (`legged_control_parity`,
  `k_capture=0`, …) fighting Bound's very different footfall pattern,
  or something more basic? `go2_wbc_bound_baseline_survey` runs four
  increasingly-sophisticated configurations — plain legacy
  `GaitMode::Mpc`, `FullCentroidal` with parity off, `FullCentroidal`
  + parity at the *default* `k_capture=0.05` (pre-Sec.5ab fix), and
  the full Sec.5ao "healthy Trot baseline" — all on `GaitConfig::
  bound()`'s own untouched defaults, at a modest cmd_vx=0.15. All four
  reverse similarly (-0.114 to -0.140 m/s) — the Trot-specific tuning
  is cleared; even the oldest, simplest SRBD path (never touched by
  any of this session's Trot work) reverses just as badly. Every
  config also shows a large sustained pitch oscillation (0.27-0.34
  rad, ~15-20°), the suspected real culprit — flagged for follow-up,
  not yet confirmed. See `ref/wbc_comparison.md` Sec.5ap.
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
