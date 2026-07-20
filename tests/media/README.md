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
- `go2_bound_reversal.mp4` — real-mesh MuJoCo video of the Sec.5ap
  reversal case (`go2_wbc_bound_forward_walk_video_source`: Bound's
  own defaults, `legged_control_parity=true, k_capture=0`,
  cmd_vx=0.15) requested so the large pitch oscillation and net
  backward drift could be inspected visually, not just from the
  summary numbers.
- `bound_gentler_sweep.png` / `go2_bound_low_swing.mp4` — Sec.5ap
  follow-up: does softening `GaitConfig::bound()`'s own timing/sizing
  (`cycle_period_s=0.30s`, faster than Trot's 0.4s; `swing_height_m=
  0.05`, higher than Trot's 0.04m) reduce the pitch oscillation and
  reversal? `go2_wbc_bound_gentler_parameters_sweep` tries four
  combinations at cmd_vx=0.15. Lowering `swing_height_m` alone
  (0.05→0.02) cuts peak |pitch| ~4x (0.291→0.067 rad) and *eliminates
  the reversal entirely* (meas_vx -0.124→+0.007) — though the robot is
  now merely not-reversing, not actually progressing forward yet.
  Slowing the cycle alone (0.30s→0.40s) makes both metrics *worse*
  (0.291→0.448 rad, -0.124→-0.157) — longer airborne time at the same
  swing height apparently increases landing impact/moment, not
  reduces it. Combining both is worse than lowering swing height
  alone. `swing_height_m` is the dominant lever, and Bound's stock
  0.05m default is simply too aggressive for the current tuning —
  unrelated to any of the Trot-specific work. The video
  (`go2_wbc_bound_low_swing_video_source`'s trace, same cmd_vx/
  duration as `go2_bound_reversal.mp4` for direct comparison) shows
  the visibly calmer trunk. See `ref/wbc_comparison.md` Sec.5aq.
- `bound_low_swing_cmdvx_flat.png` — Sec.5aq follow-up, and a
  correction to its optimistic framing: with `swing_height_m=0.02`
  fixed, sweeping `max_step_length_m` (0.08/0.12/0.16/0.20,
  `go2_wbc_bound_low_swing_max_step_length_sweep`) produces *exactly*
  the same result every time (dx=0.018m to the millimetre) — the
  footstep planner's stride clamp isn't the binding constraint at all.
  Sweeping `cmd_vx` itself instead (0.05 to 0.30 m/s,
  `go2_wbc_bound_low_swing_cmd_vx_sweep`) shows measured vx staying at
  noise level (-0.066 to +0.028) with **no relationship to the
  command**, and peak |pitch| staying flat too (0.067-0.072 rad)
  regardless of speed. Reinterpretation: `swing_height_m=0.02` didn't
  fix Bound into a working gait — it suppressed the swing motion
  enough to kill translation (and thus the reversal) entirely, leaving
  the robot effectively shuffling in place rather than genuinely
  walking. A working forward-tracking Bound gait is still unsolved.
  See `ref/wbc_comparison.md` Sec.5ar.
- Sec.5as (no new chart): two Explore-agent code audits cleared the
  footstep planner (`compute_mpc_footstep`, all four variants) and the
  `legged_control_parity` k≥1 contact-schedule projection of any
  Bound-specific bug — both are provably leg/gait-agnostic. A new
  dense `Σmpc_f_x` diagnostic (added to `run_wbc_sim`'s existing
  `Σmpc_f_z` printout) then found Bound's MPC-predicted GRF swinging
  wildly (-173 to +200N horizontal, vs Trot's tame -3.91 to +4.66N)
  and repeatedly saturating at `max_normal_force * 2` (400N — Sec.5ag
  found this cap *never* binds for Trot). `go2_wbc_bound_max_normal_
  force_sweep` (200/400/800/∞) ruled this out as the actual cause,
  though: raising or removing the cap doesn't reduce the reversal
  (-0.124 to -0.165 across all four). Footstep planner, contact
  schedule, and force cap are all cleared; the WBC's frequent HoQP
  `Infeasible`/`MaxIterations` warnings (seen since Sec.5ao, never
  directly investigated) remain the most promising unexamined lead.
  See `ref/wbc_comparison.md` Sec.5as.
- `bound_friction_mu_sweep.png` — a dedicated Explore agent read
  `misa-wbc`'s `ho_qp.rs` directly and derived a concrete mechanism:
  Bound's front-pair/rear-pair stance shares the same body-frame `r_x`
  moment arm between its two simultaneously-stance feet, so (unlike
  Trot's diagonal pair, which gets pitch torque nearly free from
  `Δf_z·Δr_x`) pitch authority must come almost entirely from `Σf_x`,
  which is friction-cone-limited (`|f_x|≤μ·f_z`) — the physical "why"
  behind Sec.5as's `Σf_x` chaos and `f_z` saturation. `go2_wbc_bound_
  friction_mu_sweep` raises `friction_mu` (WBC task + FullCentroidal
  MPC, kept in sync) from 0.5 to 5.0 at Bound's stock `swing_height_m
  =0.05` (the actual reversal case), cmd_vx=0.15. 0.5→1.5 shows the
  first genuinely *monotonic* improvement found anywhere in this
  Bound investigation (-0.124→-0.040), supporting the hypothesis —
  but it never crosses zero into real forward tracking, and beyond
  1.5 it degrades non-monotonically again (2.0: -0.111, 5.0: -0.150,
  worse than default). peak |pitch| stays flat throughout (0.257-0.310
  rad) — friction_mu doesn't reduce the oscillation itself, only
  (partially) the forward-force starvation it causes. Session
  conclusion: several real physical/numerical difficulties were found
  (pitch-authority starvation, QP conditioning for collinear stances),
  but no simple parameter tuning reached a genuinely working forward-
  walking Bound gait. See `ref/wbc_comparison.md` Sec.5at.
- `bound_true_coupling_sweep.png` — real model-based bounding
  controllers (Raibert's hopping-machine decomposition; MIT Cheetah
  2/3) treat pitch/attitude control as an independent channel — direct
  hip-joint torque exploiting the leg's own mass/inertia — rather than
  relying solely on friction-limited ground-reaction force. `true_
  centroidal_coupling` (desk-research gap ①, Sec.5aa-5ae — "neutral"
  for Trot) is architecturally the closest thing this codebase has to
  that channel, so `go2_wbc_bound_true_coupling_sweep` retests it for
  Bound: alone (-0.124→-0.116, barely moves) and combined with
  Sec.5at's best `friction_mu=1.5` (-0.061 — *worse* than `friction_mu
  =1.5` alone at -0.040). The literature's mechanism is an explicit
  feedback control law computing hip torque from pitch error; `true_
  centroidal_coupling` as implemented here is instead a passive
  linearization-accuracy correction to the MPC's own dynamics model —
  similar in name, different in kind, which is the likely reason it
  didn't deliver the hoped-for independent pitch-authority channel.
  See `ref/wbc_comparison.md` Sec.5au.
- `bound_pitch_pd_sweep.png` — the more literal reproduction of the
  literature's mechanism: `WbcPipeline` gets a new `pitch_pd_gain:
  (f64, f64)` field (default `(0.0, 0.0)`, a complete no-op) adding an
  *explicit closed-loop* pitch correction (`kp*(0-pitch) - kd*pitch_
  rate`) directly on top of `a_base_des`'s previously pure-feedforward
  angular component — the actual missing piece Sec.5au identified.
  `go2_wbc_bound_pitch_pd_sweep` tests gains from (50,5) to (400,40)
  on Bound's reversal case: **no meaningful effect at any gain**
  (-0.113 to -0.135, indistinguishable from the -0.124 baseline; peak
  pitch also unaffected). Even the most literal reproduction of the
  literature's control law didn't help. Interpretation: `a_base_des`
  is a *soft* priority-1 task, only realizable within whatever
  hard-constraint (`friction_cone`, `floating_base_eom`, `no_contact_
  motion`) feasible region priority 0 allows — asking harder for a
  pitch correction the friction cone can't physically deliver doesn't
  conjure the force budget to do it. `friction_mu` (Sec.5at) helped
  partially because it relaxed an actual hard constraint; `pitch_pd_
  gain` only retargets an already-constrained soft task. Session
  conclusion: footstep planner, contact schedule, `max_normal_force`,
  `true_centroidal_coupling`, and explicit pitch PD are all cleared or
  only partially effective; `friction_mu` remains the sole real (but
  insufficient) lever found. See `ref/wbc_comparison.md` Sec.5av.
- Sec.5aw (no new chart): re-analyzing Sec.5as's already-collected
  dense diagnostic logs (not a new sim run) surfaced a `max|τ|` data
  point not previously highlighted: Bound demands joint torque up to
  44.71 N·m — ~1.9x Go2's real 23.7 N·m hip/thigh limit — in 12.5% of
  sampled ticks (Trot: 0%, never exceeding 17.29 N·m). Since MuJoCo's
  `bake_actuator_limits` silently clips torque commands at the real
  effort limit, the WBC's own returned solution and what the robot
  physically receives diverge whenever this happens — directly tying
  to the HoQP `Infeasible`/`MaxIterations` pattern (a solver that
  fails to converge can return a solution violating its own hard
  torque-limit constraint). Answers the user's "is it friction or
  torque?" question: both are *symptoms* of the same root cause
  (Bound's collinear stance ill-conditioning the QP), not independent
  causes.
- `bound_effort_ground_friction.png` — tests both factors directly in
  the simulation model (not just the solver's internal belief).
  `go2_wbc_bound_actuator_effort_scale_sweep` scales every joint's real
  `effort` (N·m) by 1.0/2.0/5.0 (relaxing MuJoCo's own actuator clamp
  *and* the WBC's `torque_max`, not a solver-internal trick): 2x gives
  a real, substantial improvement (-0.124→-0.056), but 5x is worse
  again (-0.090) with a new roll instability appearing (peak
  |roll|=0.018 rad, previously always ~0). `go2_wbc_bound_matched_
  friction_sweep` tests whether Sec.5at's `friction_mu` improvement was
  really about physical grip: matching the *real* MJCF ground friction
  to the WBC's `friction_mu` belief (0.7→1.5, or 3.0/3.0) changes
  **nothing at all** — identical results to the mismatched case. The
  feet never actually approach the real 0.7 friction's slip limit in
  this scenario; `friction_mu`'s partial benefit (Sec.5at) is a pure
  QP-internal numerical effect (reshaping the friction-cone task
  changes which solution the solver converges to), not a physical
  traction effect. Actuator torque is a real, if partial and non-
  monotonic, lever; ground friction was a red herring all along. See
  `ref/wbc_comparison.md` Sec.5aw/5ax.
- `bound_cmd_vx_ramp_sweep.png` — a user observation: real animals
  never enter a steady bounding gait from a dead stop (a crouch/wind-
  up transient, anticipatory postural adjustments) — every Bound test
  so far stepped `cmd_vx` instantaneously in one tick, exactly the
  "cold start" animals avoid. `WbcParams::cmd_vx_ramp_s` (new) linearly
  ramps the command in instead; `go2_wbc_bound_cmd_vx_ramp_sweep`
  compares ramp durations 0.0 (instant, baseline) / 0.5 / 1.0 / 2.0s,
  measuring only the **post-ramp steady-state window** so different
  ramp lengths are compared fairly. Result: ramping doesn't help —
  moderate ramps (0.5-1.0s) are actually worse (-0.147, -0.164 vs the
  -0.116 baseline), and only the longest ramp (2.0s) returns to about
  the baseline. Peak pitch stays flat (0.265-0.291 rad) regardless of
  ramp duration. This clearly disconfirms the transient-onset
  hypothesis: the pitch oscillation and reversal aren't triggered by
  the moment of command change — they're an intrinsic, continuously-
  present property of Bound's own steady-state dynamics, consistent
  with every other finding pointing at the collinear-stance QP
  ill-conditioning as the real, structural cause. See `ref/wbc_
  comparison.md` Sec.5ay.
- `bound_smoothing_prox_sweep.png` — before any invasive `ho_qp.rs`
  surgery, tests two already-implemented, zero-new-code levers the
  external audit flagged as worth trying first: `WbcPipeline::
  grf_smoothing_alpha` (EMA-smooths the `contact_force` task's target
  instead of Bound's raw, wildly-swinging MPC GRF) and `qp_prox_weight`
  (0.0 disables the warm-start anchor — cold solve every tick, instead
  of anchoring toward the *previous* tick's very different solution).
  `go2_wbc_bound_grf_smoothing_and_prox_sweep` counts HoQP `Infeasible`/
  `MaxIterations` warnings alongside the usual tracking metrics.
  Disabling the warm-start (`prox=0.0`) cuts solver non-convergence by
  ~85-88% (642→74 warnings over 2.5s) — strong confirmation of the
  audit's "stale warm-start seed" hypothesis. **But `meas_vx` barely
  moves** (-0.124→-0.120) — the reversal doesn't meaningfully improve
  even with dramatically better solver convergence. This is the
  session's key clarifying result: the HoQP's numerical ill-
  conditioning is real and fixable, but it isn't the actual cause of
  the reversal — the deeper cause is a *geometric* one (Bound's
  front/rear-pair stance structurally lacks a cheap pitch-torque path
  like Trot's diagonal `Δf_z·Δr_x`, forcing reliance on friction-
  limited `Σf_x` regardless of how cleanly the solver converges).
  Suggests the more promising redesign path may not be Bound-specific
  WBC surgery at all, but a gait like Canter that staggers the two legs
  within each front/rear pair slightly — avoiding the degenerate
  collinear-support geometry from the start. See `ref/wbc_comparison.md`
  Sec.5az.
- `bound_mass_inertia_fix.png` — a user question ("do we know the
  momentum budget Bound actually needs?") led to checking `WbcPipeline`'s
  `mass_kg`/`inertia_diag_body` (feeding `a_base_des`'s dominant
  weight-200 Newton-Euler reference), since answering the momentum
  question properly requires knowing what physical parameters the WBC
  believes it has. `go2_diag_wbc_mass_inertia_mismatch` found these are
  a "Cheetah-class" placeholder (`mass_kg=9.0`, pitch inertia 0.26)
  **never synced to Go2's real, auto-detected values anywhere in this
  file** — real Go2 is 15.606 kg (1.73x heavier) with pitch inertia
  0.098 (the placeholder is 2.65x too large). `go2_wbc_mass_inertia_
  fix_sweep` tests correcting this (`sync_real_mass_inertia`, new):
  Bound's reversal barely changes (-0.124→-0.129), and Trot gets a tiny
  tracking improvement but its roll instability more than doubles
  (0.026→0.061 rad). A real, worth-fixing-eventually bug, but not the
  cause of Bound's reversal — reinforces Sec.5az's geometric-cause
  conclusion rather than overturning it. See `ref/wbc_comparison.md`
  Sec.5ba.
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
