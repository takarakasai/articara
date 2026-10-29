//! The walker's control, phase by phase (S11 of
//! doc/from_claude/20261029_walker_extraction_plan.md).
//!
//! Each `phase_*` function is one block of the tick's control, moved here
//! verbatim from the loop in main.rs: the knobs it reads are bound at its top
//! from the [`Config`] (the same `let x = cfg.sub.x;` aliases main.rs bound
//! once from `wcfg`), the start-up constants come from [`WalkerConsts`], and
//! what it computed for the later phases is returned in its own `*Out`
//! struct. The plan, the controller's state and its counters are passed by
//! `&mut`; the tick's `TickRecord` is still built once, in the loop, from the
//! phases' outputs. The arithmetic, the order of the prints and the order of
//! the plan's rebuilds are the loop's own (the runs are byte-identical).
use super::{CtlState, DiagSinks, WalkerStats};
use articara::biped::contact::Anchors;
use articara::biped::dcm::{DcmRef, SupportBox, StepTiming, com_accel_xy, commanded_zmp, dcm_of, step_timing};
use articara::biped::gait::{Footsteps, GaitParams, Slice, Support};
use articara::biped::rig::G;
use articara::biped::terrain::Stairs;
use misa_biped::config::Config;
use misa_biped::plan::Plan;
use misa_biped::recovery::{RefLib, RefMode};
use misa_biped::robot::{Estimate, Observation, RobotModel, RobotState};
use nalgebra as na;
use std::f64::consts::PI;
use std::io::Write;

/// What the control reads that is fixed at start-up but is not a knob: the
/// robot model, the realised control period, the planned LIPM's omega, the
/// nominal CoM height, and the scenario values the walker is still handed
/// (the known terrain, the sole half-size the harness read).
pub(crate) struct WalkerConsts<'a> {
    pub robot: &'a RobotModel,
    /// Realised control period (substeps x sim dt).
    pub dt: f64,
    /// The DCM plan's omega (`plan.dcm.omega`).
    pub omega: f64,
    pub total_mass: f64,
    /// Nominal CoM height and the terrain height it was measured over.
    pub z_com: f64,
    pub terrain_base: f64,
    /// SOLE_HALF_L / SOLE_HALF_W (read by the harness; S13 moves them to
    /// the robot's Profile).
    pub sole_half_l: f64,
    pub sole_half_w: f64,
    /// The known terrain (`Some` also for the zero-riser "stair" of the flat
    /// benches, §6.2 of the plan).
    pub stairs: &'a Option<Stairs>,
    /// `cfg.descent.land_extend(&cfg.landing)`.
    pub land_extend_desc: f64,
    /// The command (until a command schedule replaces them, §6.9): the
    /// forward speed, the step period and the gait's timing, the squat, the
    /// speed change (VX2: at step, speed, ramp steps; after the stride cap)
    /// with the speed it commands, the time its CoM drop starts and the drop.
    pub vx: f64,
    pub t_step: f64,
    pub gp: &'a GaitParams,
    pub squat_amp: f64,
    pub squat_period: f64,
    pub vx2_change: Option<(usize, f64, usize)>,
    pub vx2_cmd_speed: Option<f64>,
    pub vx2_z_from: Option<f64>,
    pub vx2_z_drop: f64,
    /// The push-recovery references (REF_LIBRARY, read by the harness).
    pub ref_libs: &'a [RefLib],
    /// `cfg.contact_plan.planner()`, and the CoM preview's clocks
    /// (`cfg.com_plan.{preview_dt, replan_dt}.max(dt)`).
    pub contact_plan_cfg: articara::biped::contact_plan::Config,
    pub com_preview_dt: f64,
    pub com_replan_dt: f64,
}

/// The state the observation carries (the harness assembles it until S14).
pub(crate) fn obs_state<'a>(obs: &Observation<'a>) -> &'a RobotState {
    match obs.estimate {
        Estimate::State(st) => st,
        Estimate::Raw(_) => unreachable!("the walker assembles no state before S14"),
    }
}

/// Phase 1: the landing detector's tolerances and extension for the slice
/// the clock is in (ascending, descending, or a step whose end the contact
/// planner owns). Assigned every tick, before the plant is read.
pub(crate) fn phase_landing_setup(ctl: &CtlState, plan: &mut Plan, cfg: &Config, k: &WalkerConsts, t: f64) {
    // Extra landing-deadline slack for the swing that CHANGES HEIGHT, i.e. the
    // step onto a tread. Zero (the default) means "same as everywhere else".
    //
    // Measured 2026-09-22 on G1: the step onto a 20 mm tread needs roughly a
    // second of slack before its load crosses the confirm threshold -- the foot
    // rests on the tread at 4-6 N for ~500 ms first -- but applying that slack
    // to EVERY swing stalls the gait clock afterwards. With
    // `COM_LAND_EXTEND=1.08` the walk reached the tread 15/16 and then died:
    // 2 landing retimes totalling 2.16 s, the stair swing taking 2.035 s
    // against a planned 0.681, and the swings after it collapsing to
    // 1.075 / 0.500 / 0.260 / 0.035 s. With the default 0.25 it never reaches
    // the tread at all (0/16). The two ends of that trade want different
    // numbers, so they get different numbers.
    let land_extend_rise = cfg.landing.extend_rise;
    // STEP_DZ_MIN: planned sole-height change above which a swing is a
    // stair swing (rise/descent dwell and confirmation, stair landing box,
    // landing-timing activation). 0.005 = shipped, bit-identical; a known
    // slope (TERRAIN_SLOPE, 10-15 mm per step) sets ~0.03 so its steps are
    // level steps (measured 2026-10-15: at 0.005 every step waited the
    // 1.08 s rise dwell and the walk collapsed). STEP_DZ_SWING_MIN does the
    // same for the swing PROFILE choice (stair quintic vs smoothstep).
    let step_dz_min = cfg.landing.step_dz_min;
    // 0 = a descending swing keeps its extension as a dwell too (double
    // support starts at the deadline, as the historical timeout did).
    let contact_event_end_on_touch = cfg.landing.event.end_on_touch;
    // xy tolerance of the landing confirmation for DESCENDING swings only.
    // Measured 2026-10-03 on the up-and-over course (8 cm, 2/48): the long
    // step-down swing overshoots its goal by ~23 mm in x, the 12 mm gate
    // never confirms a foot that is flat on the ground, the swing times out
    // and the MPC then moves the landing 15 cm while the foot is down and
    // drags it. Unset = LAND_CONFIRM_XY (bit-identical).
    // (LAND_CONFIRM_XY_DESC_CP, COM_LAND_EXTEND_DESC_CP and
    // LAND_CONFIRM_FZ_DESC_FRAC -- variants of the descent confirmation --
    // and LAND_CONFIRM_XY_RISE -- confirming rising swings, which ended the
    // CEM's 1.08 s ascent dwell (1 step up 192 -> 114) -- were not adopted
    // and are removed.)
    // z tolerance that goes with the wider xy one: the 10 mm default lets
    // the confirmation fire on the HEEL STRIKE of a toe-up foot (sole centre
    // still 8 mm high), 60 ms before the sole is flat (measured: 2 steps x
    // 6 cm 46 -> 33 with the wide xy alone). Unset = LAND_CONFIRM_Z.
    // (Unset = LAND_CONFIRM_Z: the landing clock's `z_tol_override`.)
    let land_confirm_xy_desc = cfg.descent.land_confirm_xy;
    let contact_plan_on = cfg.contact_plan.on;
    // (COM_LAND_PREDICT_FRAC is the landing clock's `min_fraction`.)
    let land_extend_base = cfg.landing.extend;
    let land_extend_desc = k.land_extend_desc;
    if land_extend_rise > 0.0 {
        // Same test the landing detector itself uses to decide a swing is
        // a stair swing (`preview.rs`: the planned sole height changes by
        // more than 5 mm), so the slack applies to exactly the swings that
        // need it and to no others.
        let i = plan.gait.index_at(t);
        let dz_swing = plan.gait.slices[i].support.swing().map_or(0.0, |side| {
            plan.footsteps.at_slice(i).sole[side].z
                - plan.footsteps.at_slice(i.saturating_sub(1)).sole[side].z
        });
        let rising = dz_swing.abs() > step_dz_min;
        let descending = dz_swing < -step_dz_min;
        plan.landing.event_end_on_touch = descending && contact_event_end_on_touch;
        plan.landing.xy_tol_override = if descending { land_confirm_xy_desc } else { None };
        // The contact planner owns the end of a step it has retimed; the
        // landing extension must not re-open it (assigned every tick, so
        // the planner's own zeroing would be undone here otherwise).
        plan.landing.max_extension = if contact_plan_on && ctl.cp_retimed_slice == Some(i) {
            0.0
        } else if descending {
            land_extend_desc
        } else if rising {
            land_extend_rise
        } else {
            land_extend_base
        };
    }
}

/// Phase 2: the landing clock (touchdown, timeout, the confirmed admission's
/// certificate, the touchdown event's shift), DS_FEASIBLE, DS_CAPTURE, and the
/// engagement of the CoM re-plan. `actual` is the measured soles (FK).
#[allow(clippy::too_many_arguments)]
pub(crate) fn phase_landing_clock(
    ctl: &mut CtlState,
    plan: &mut Plan,
    stats: &mut WalkerStats,
    cfg: &Config,
    k: &WalkerConsts,
    obs: &Observation,
    actual: &Footsteps,
    t: f64,
) {
    let com_replan = cfg.com_plan.replan;
    let contact_admit_confirmed = cfg.contact.admit_confirmed;
    // A landing that ends by the extension TIMEOUT counts too when the sole
    // is within this of its planned tread height: measured 2026-09-24, the
    // champion's stair landings mostly end that way (retime median 2.16 s =
    // 1.0 s swing + 1.08 s extension) with the sole on the tread all along.
    let contact_admit_z = cfg.contact.admit_z;
    let debug = cfg.debug;
    let contact_event_shift = cfg.landing.event.shift;
    // Run the DS_FEASIBLE re-plan on a timeout end as well (it is keyed on a
    // non-zero retime, which a timeout never produces). Default off.
    let ds_feasible_on_timeout = cfg.double_support.feasible_on_timeout;
    let ds_feasible = cfg.double_support.feasible;
    let cop_frac = cfg.dcm.cop_frac;
    // Keep the commanded ZMP inside the support polygon, shrunk by this
    // factor. Riding the exact edge means the next disturbance makes P0
    // infeasible outright.
    let zmp_margin = cfg.dcm.zmp_margin;
    // Pivot box for the DS sizing: "both" = the double-support box (the
    // historical choice), "stance" = the OLD stance sole only. At the cut the
    // new foot carries ~25% of the weight (measured), so a pivot under it is
    // not realisable until the load has moved; the stance-only box is what
    // the pressure can reach NOW.
    let ds_feasible_box_stance = cfg.double_support.feasible_box_stance;
    let ds_feasible_continuous = cfg.double_support.feasible_continuous;
    let ds_feasible_max = cfg.double_support.feasible_max;
    // Ramp variant: the double-support ZMP ramps from a solved p0 to the
    // next stance's own entry point p1, so the new foot takes load INSIDE
    // the double support. With the constant solve above the ZMP sits behind
    // the leading foot for the whole DS (measured 2026-09-24: the admitted
    // foot carried 0-6 N until single support). For p(t) = p0 + v t:
    //     xi(0) = p0 A + p1 B + xi_eos e^{-wT},
    //     A = 1 - (1 - e^{-wT})/(wT),  B = (1 - e^{-wT})/(wT) - e^{-wT}
    // solved for p0 and clamped to the DS box. Default off.
    let ds_feasible_ramp = cfg.double_support.feasible_ramp;
    // DS_CAPTURE: end the double support EARLY when the DCM has left the
    // double-support box along x (no pressure point inside it can hold
    // it: 0-step capture is gone, only the next step can). Measured
    // 2026-10-11 on the late-phase forward push (11 N*s at swing 0.63):
    // the foot lands short, then the nominal 0.37 s double support runs
    // while the CoM walks past the front foot, and the catch step comes
    // 0.9 s after the push -- too late (12/12 falls). The DS end moves to
    // `t0 + DS_CAPTURE_T_MIN` (not before the landed foot carries
    // DS_CAPTURE_LOAD of the weight), once per slice; the next single
    // support is then shortened by the contact_plan and its landing by
    // the MPC as usual. DS_FEASIBLE (which only lengthens, when the DS
    // pressure CAN still catch the DCM) is complementary. Default off.
    let ds_capture = cfg.double_support.capture;
    // "Hopeless": the time the DS pressure needs along x exceeds this
    // multiple of the remaining double support.
    let ds_capture_hopeless = cfg.double_support.capture_hopeless;
    // Fire only when the DCM has run AHEAD of the DS-end target (a forward
    // push). Measured 2026-10-11 (N=48, 4 conditions): without the gate the
    // forward push went 36 -> 45 but the backward one 34 -> 32 -- stepping
    // sooner backward does not help a leg that has no authority (the
    // backward failure, design note 10-10). Default on.
    let ds_capture_fwd_only = cfg.double_support.capture_fwd_only;
    // (DS_CAPTURE_LAT, the same test along y, was not adopted and is
    // removed.)
    // The forward excursion must exceed this [m] (noise floor).
    let ds_capture_margin = cfg.double_support.capture_margin;
    let ds_capture_t_min = cfg.double_support.capture_t_min;
    let ds_capture_load = cfg.double_support.capture_load;
    // Only shorten when it saves at least this much [s].
    let ds_capture_min_gain = cfg.double_support.capture_min_gain;
    let k_dcm = cfg.dcm.k_dcm;
    let (dt, omega, total_mass) = (k.dt, k.omega, k.total_mass);
    let (z_com, terrain_base) = (k.z_com, k.terrain_base);
    let (sole_half_l, sole_half_w) = (k.sole_half_l, k.sole_half_w);
    let st = obs_state(obs);
    let com = st.com;
    let com_vel = st.com_vel;
    if com_replan {
        let touchdowns_before = plan.landing.touchdowns;
        let timeouts_before = plan.landing.timeouts;
        let shift = plan.landing.update(&mut plan.gait, &plan.footsteps, &actual, obs.contact_fz, t, dt);
        let landing_timed_out = plan.landing.timeouts > timeouts_before;
        // The slice that just ended is the timer's clock slice, not
        // `gait.index_at(t)`: on a timeout tick the index already points
        // at the double support, whose swing side is None.
        if contact_admit_confirmed
            && (plan.landing.touchdowns > touchdowns_before || landing_timed_out)
            && let Some((ended, _, _)) = plan.landing.clock
            && let Some(sw) = plan.gait.slices[ended].support.swing()
            && (plan.footsteps.at_slice(ended).sole[sw].z - actual.sole[sw].z).abs() < contact_admit_z
        {
            ctl.confirmed_admit[sw] = Some(ended + 1);
            if debug.contact_admit {
                println!("  [adm] t={t:.3} certificate side={sw} ds_slice={} (confirm={} timeout={})", ended + 1, plan.landing.touchdowns > touchdowns_before, landing_timed_out);
            }
        }
        if let Some((li, lsw, err_xy)) = plan.landing.last_event.take()
            && contact_event_shift
            && err_xy.norm() > 1e-4
        {
            // plan - actual = err  ->  move the plan by -err onto the foot.
            plan.footsteps.shift_next(li, lsw, -err_xy);
            plan.rebuild_dcm();
        }
        if shift != 0.0 || (ds_feasible_on_timeout && landing_timed_out) {
            if shift != 0.0 {
                stats.landing_retimes += 1;
                stats.landing_delay += -shift;
            }
            plan.rebuild_dcm();
            ctl.preview_next = t;
            if ds_feasible {
                let ds = plan.gait.index_at(t + 1e-6);
                if matches!(plan.gait.slices[ds].support, Support::Double) {
                    let xi_meas = dcm_of(&com, &com_vel, omega);
                    let seg = plan.dcm.index_at(t + 1e-6);
                    let target = plan.dcm.xi_eos[seg];
                    let ds_box_full = SupportBox::from_stance_yawed(
                        &actual,
                        Support::Double,
                        (sole_half_l * cop_frac, sole_half_w * cop_frac),
                        zmp_margin,
                    );
                    // The support that just ended tells which foot carries the load.
                    let prev_sw = ds
                        .checked_sub(1)
                        .and_then(|j| plan.gait.slices[j].support.swing());
                    let ds_box = match (ds_feasible_box_stance, prev_sw) {
                        (true, Some(sw)) => SupportBox::from_stance_yawed(
                            &actual,
                            Support::Single { stance: 1 - sw, swing: sw },
                            (sole_half_l * cop_frac, sole_half_w * cop_frac),
                            zmp_margin,
                        ),
                        _ => SupportBox { lo: ds_box_full.lo, hi: ds_box_full.hi },
                    };
                    // Pivot: a box point on the far side of the DCM from
                    // the target, so the DCM is pushed TOWARD the target
                    // (`xi_dot = omega (xi - p)`). Clamping the DCM itself
                    // put the pivot on the DCM when it was inside the box
                    // (gap zero, `Degenerate`) and the branch never fired
                    // -- measured 2026-09-24, 0 retimes in 384 runs.
                    let nominal = plan.gait.slices[ds].duration();
                    let p_continuous = |tt: f64| -> na::Vector2<f64> {
                        let d = (-omega * tt).exp();
                        ds_box.clamp(&((xi_meas - target * d) / (1.0 - d))).0
                    };
                    let p = if ds_feasible_continuous {
                        p_continuous(nominal)
                    } else {
                        ds_box.clamp(&(xi_meas * 2.0 - target)).0
                    };
                    let mut need = 0.0_f64;
                    for axis in 0..2 {
                        if let StepTiming::Time(tt) = step_timing(&xi_meas, &target, &p, omega, axis) {
                            need = need.max(tt);
                        }
                    }
                    let want = need.clamp(nominal, ds_feasible_max);
                    if ds_feasible_continuous {
                        if want > nominal + 1e-3 {
                            let dd = plan.gait.retime(ds, plan.gait.slices[ds].t0 + want);
                            stats.n_ds_feasible += 1;
                            stats.ds_feasible_total += dd.abs();
                            plan.rebuild_dcm();
                        }
                        let seg_ds = plan.dcm.index_at(plan.gait.slices[ds].t0 + 1e-6);
                        let p_final = if ds_feasible_ramp && seg_ds + 1 < plan.dcm.segs.len() {
                            let tt = plan.gait.slices[ds].duration();
                            let wt = omega * tt;
                            let d = (-wt).exp();
                            let b = (1.0 - d) / wt - d;
                            let a = 1.0 - (1.0 - d) / wt;
                            let p1 = plan.dcm.segs[seg_ds + 1].p0;
                            let p0 = ds_box.clamp(&((xi_meas - target * d - p1 * b) / a)).0;
                            plan.dcm.set_segment_zmp_ramp(seg_ds, p0, p1);
                            p0
                        } else {
                            let p = p_continuous(plan.gait.slices[ds].duration());
                            plan.dcm.set_segment_zmp(seg_ds, p);
                            p
                        };
                        stats.n_ds_continuous += 1;
                        if debug.ds_feasible {
                            let r = plan.dcm.reference(plan.gait.slices[ds].t0 + 1e-6);
                            println!("  [dsc] t={t:.3} ds={ds} xi=({:.3},{:+.3}) target=({:.3},{:+.3}) p=({:.3},{:+.3}) T={:.3} xi_ref(t0)=({:.3},{:+.3})", xi_meas.x, xi_meas.y, target.x, target.y, p_final.x, p_final.y, plan.gait.slices[ds].duration(), r.xi.x, r.xi.y);
                        }
                    } else if want > nominal + 1e-3 {
                        let dd = plan.gait.retime(ds, plan.gait.slices[ds].t0 + want);
                        stats.n_ds_feasible += 1;
                        stats.ds_feasible_total += dd.abs();
                        plan.rebuild_dcm();
                    }
                    if debug.ds_feasible {
                        println!("  [dsf] t={t:.3} ds={ds} xi=({:.3},{:+.3}) target=({:.3},{:+.3}) p=({:.3},{:+.3}) need={need:.3} nominal={nominal:.3} -> {want:.3}", xi_meas.x, xi_meas.y, target.x, target.y, p.x, p.y);
                    }
                }
            }
        }
        if ds_capture {
            let ds = plan.gait.index_at(t + 1e-6);
            let is_ds = |j: usize| matches!(plan.gait.slices[j].support, Support::Double);
            // Not while a recovery reference owns the phase durations.
            if is_ds(ds) && ctl.ref_mode.is_none() && !ctl.dsc_retimed_last.is_some_and(|l| l >= ds) {
                // The whole run of double-support slices the clock is in
                // (a DS is split into ramp / hold / ramp slices).
                let mut first = ds;
                while first > 0 && is_ds(first - 1) {
                    first -= 1;
                }
                let mut last = ds;
                while last + 1 < plan.gait.slices.len() && is_ds(last + 1) {
                    last += 1;
                }
                let t0_run = plan.gait.slices[first].t0;
                let t1_run = plan.gait.slices[last].t1;
                let remaining = t1_run - t;
                let xi_meas = dcm_of(&com, &com_vel, omega);
                let seg_last = plan.dcm.index_at(t1_run - 1e-6);
                let target = plan.dcm.xi_eos[seg_last];
                let ds_box = SupportBox::from_stance_yawed(
                    &actual,
                    Support::Double,
                    (sole_half_l * cop_frac, sole_half_w * cop_frac),
                    zmp_margin,
                );
                // The pivot that would bring the DCM to the DS-end target
                // in the remaining time, clamped to the box; the time it
                // then actually needs along x (DS_FEASIBLE's own test).
                let dcy = (-omega * remaining.max(1e-3)).exp();
                let p = ds_box.clamp(&((xi_meas - target * dcy) / (1.0 - dcy))).0;
                // Degenerate = the DCM is already at the target (or the
                // pivot sits on it): nothing to do, NOT hopeless. Measured
                // 2026-10-11 (7@90 m19): a 0.1 mm gap was read as
                // unreachable and a healthy double support lost 300 ms.
                let need = match step_timing(&xi_meas, &target, &p, omega, 0) {
                    StepTiming::Time(tt) => tt,
                    StepTiming::Degenerate => 0.0,
                    StepTiming::Unreachable => f64::INFINITY,
                };
                let hopeless_x = remaining > 1e-3
                    && need > remaining * ds_capture_hopeless
                    && (!ds_capture_fwd_only || xi_meas.x > target.x + ds_capture_margin);
                let hopeless = hopeless_x;
                let prev_sw = first.checked_sub(1).and_then(|j| plan.gait.slices[j].support.swing());
                let landed_load = prev_sw.map_or(0.0, |sw| ctl.fz_prev[sw]);
                let earliest = (t0_run + ds_capture_t_min).max(t + 2.0 * dt);
                if debug.ds_capture {
                    println!(
                        "  [dsc-cap?] t={t:.3} run {first}..{last} [{t0_run:.3},{t1_run:.3}] xi.x={:.3} target.x={:.3} p.x={:.3} need={need:.3} rem={remaining:.3} hopeless={hopeless} load={landed_load:.0} earliest={earliest:.3}",
                        xi_meas.x, target.x, p.x
                    );
                }
                if hopeless
                    && landed_load >= ds_capture_load * total_mass * G
                    && earliest < t1_run - ds_capture_min_gain
                {
                    let f = (earliest - t) / remaining;
                    let targets: Vec<(usize, f64)> =
                        (ds..=last).map(|j| (j, t + (plan.gait.slices[j].t1 - t) * f)).collect();
                    for (j, nt) in targets {
                        stats.ds_capture_total += plan.gait.retime(j, nt).abs();
                    }
                    ctl.dsc_retimed_last = Some(last);
                    stats.n_ds_capture += 1;
                    plan.rebuild_dcm();
                    if debug.ds_capture {
                        println!("  [dsc-cap] t={t:.3} DS end {t1_run:.3} -> {earliest:.3}");
                    }
                }
            }
        }
        // (COM_REPLAN_ALWAYS, engaging before the first landing clock, is
        // removed.)
        if !ctl.replan_engaged && plan.landing.clock.is_some() {
            let cmd_ref = plan.dcm.reference(t);
            ctl.preview_p = commanded_zmp(
                &dcm_of(&com, &com_vel, omega),
                &cmd_ref,
                omega,
                k_dcm,
            );
            plan.height = articara::biped::preview::HeightPlan::constant(
                z_com + ctl.terrain_track - terrain_base,
            );
            ctl.replan_engaged = true;
        }
    }
}

/// What phase 3 found: the schedule's slice now, the support (all `Double`
/// with NO_LIFT), the steps taken, the planned soles of this slice, and the
/// swing's rotation mode (the stair zone's own inside it).
pub(crate) struct SupportOut {
    pub slice: Slice,
    pub slice_frac: f64,
    pub support: Support,
    pub step_idx: usize,
    pub slice_idx: usize,
    pub steps: Footsteps,
    pub swing_rot_mode: u32,
}

/// Phase 3: the schedule -> the support, and the contact anchors released at
/// a lift-off and set at a touchdown.
pub(crate) fn phase_support(
    ctl: &mut CtlState,
    plan: &Plan,
    anchors: &mut Anchors,
    cfg: &Config,
    k: &WalkerConsts,
    obs: &Observation,
    t: f64,
) -> SupportOut {
    let no_lift = cfg.gait.no_lift;
    let swing_rot_mode = cfg.swing.rot_mode;
    // STAIR_ZONE_SWING_ROT=mode: the SWING_ROT mode used only while the
    // mid-feet point is inside a known staircase's zone (the same zone as
    // STAIR_ZONE_CROUCH, including its lead/tail). Measured 2026-10-20 on
    // kyo46rs (N=24): holding the swing foot's pitch (mode 3) is what lets
    // it take the 0.13-0.16 m stair step (8 -> 12 of 12) and the 10 mm
    // descent (1 -> 24 of 24 with the zone crouch), but applied everywhere
    // it costs the 0.9 N.s forward push (21 -> 16). 0 = off = SWING_ROT
    // everywhere, bit-identical.
    let stair_zone_swing_rot = cfg.swing.zone_rot_mode;
    let stair_crouch_lead = cfg.stair_zone.lead;
    let stair_crouch_tail = cfg.stair_zone.tail;
    let debug = cfg.debug;
    let robot = k.robot;
    let prof = &robot.prof;
    let stairs = k.stairs;
    let data = &obs_state(obs).data;
    let (slice, slice_frac) = {
        let (sl, f) = plan.gait.at(t);
        (*sl, f)
    };
    let support = if no_lift {
        Support::Double
    } else {
        slice.support
    };
    let step_idx = plan.gait.steps_taken(t.max(1e-9));
    let slice_idx = plan.gait.index_at(t);
    // Everything below reads the footsteps for THIS slice. With stride 0
    // they are the same pair every time and this is the old behaviour.
    let steps = *plan.footsteps.at_slice(slice_idx);
    let swing_rot_mode = if stair_zone_swing_rot > 0
        && stairs.as_ref().is_some_and(|st| {
            articara::biped::height::stair_zone_crouch(steps.mid_xy().x, st, stair_crouch_lead, stair_crouch_tail, 1.0) > 0.0
        }) {
        stair_zone_swing_rot
    } else {
        swing_rot_mode
    };

    // Touchdown and lift-off are the only two events that may edit an
    // anchor. Doing it on the schedule (rather than lazily, the way
    // standing did with get_or_insert) is what makes a foot that has been
    // somewhere else since stop arguing for where it used to be.
    if support != ctl.prev_support {
        match (ctl.prev_support, support) {
            (Support::Double, Support::Single { swing, .. }) => {
                anchors.release(swing);
                let p = misarta::se3::translation(&data.oMi[robot.foot_mi[swing]]);
                ctl.swing_lift_off = Some(p);
            }
            (Support::Single { swing, .. }, Support::Double) => {
                // The foot has landed somewhere other than nominal, so
                // every future segment is relative to where it ACTUALLY
                // is. Rebuilding the DCM plan here is safe rather than
                // discontinuous: a segment's influence on the present
                // reference decays by exp(-omega*T) per step -- about a
                // factor of 7 at these timings -- so a few mm one step
                // ahead moves the current reference by a fraction of a
                // millimetre.
                // Re-anchor the landed foot where it ACTUALLY is, not
                // where it was planned to be: the anchor's job is to stop
                // drift from here, and seeding it with a position the
                // foot is not at demands a force to close a gap that the
                // contact cannot close.
                let mi = robot.foot_mi[swing];
                let mut p = misarta::se3::translation(&data.oMi[mi]);
                // Rotation from the PLAN (flat sole at the planned yaw),
                // not from the measurement. The anchor's rotation is
                // deliberately leak-free ("roll cannot be conceded"), so
                // anchoring a toe-first landing at its measured pitch
                // HOLDS the foot on its toe -- measured on the descent,
                // the sole stayed at +27..+47 deg after touchdown, the
                // robot vaulted over the toe and fell forward. Anchoring
                // flat makes the Baumgarte roll the heel down instead,
                // which is what a toe-first landing is FOR.
                // (STAIR_ANCHOR_FLAT, the plan rotation, is removed.)
                let r = misarta::se3::rotation_matrix(&data.oMi[mi]);
                // x, y and orientation from the measurement -- but z on
                // the WALKING SURFACE (the plan's sole height), same as
                // the contact-driven branch below. Anchoring a foot at
                // its measured z was fine when landings finished on
                // time; a stair swing that is still 20 mm up at the
                // slice boundary was being anchored IN MID-AIR, held
                // there by Baumgarte, and never loaded -- the double
                // support then had one real foot, and the next lift-off
                // fell. Measured: the entire knife-edge sensitivity of
                // the 4-step recipe was "does this landing finish
                // before the boundary".
                let p_meas_z = p.z;
                p.z = steps.sole[swing].z + prof.sole_below_origin;
                if debug.anchor {
                    eprintln!(
                        "  [anchor] t={t:.3} swing {} touchdown: meas z {:.4} -> plan z {:.4}",
                        if swing == 0 { "L" } else { "R" },
                        p_meas_z,
                        p.z
                    );
                }
                anchors.touchdown(swing, p, r);
                ctl.swing_lift_off = None;
            }
            _ => {}
        }
        ctl.prev_support = support;
    }
    SupportOut { slice, slice_frac, support, step_idx, slice_idx, steps, swing_rot_mode }
}

/// The double support's load ramp: the foot that just landed takes its share
/// on over `ramp_frac` of the slice, the foot about to lift gives it up.
pub(crate) struct LoadRamp {
    pub landing: Option<usize>,
    pub lifting: Option<usize>,
    slice_frac: f64,
    ramp_frac: f64,
    foot_mi: [usize; 2],
}

impl LoadRamp {
    fn f_share(&self, side: usize) -> f64 {
        if let Some(l) = self.landing {
            if l == side {
                return (self.slice_frac / self.ramp_frac).clamp(0.0, 1.0);
            }
        }
        if let Some(l) = self.lifting {
            if l == side {
                return ((1.0 - self.slice_frac) / self.ramp_frac).clamp(0.0, 1.0);
            }
        }
        1.0
    }

    /// Load share of the foot whose model index is `mi` (floored at 2%).
    pub fn load_share(&self, mi: usize) -> f64 {
        let side = usize::from(mi != self.foot_mi[0]);
        self.f_share(side).max(0.02)
    }
}

/// What phase 4 found: the measured vertical forces as the controller read
/// them, the feet the QP solves with as contacts (sides, model indices,
/// count), and the load ramp.
pub(crate) struct ContactsOut {
    pub fz_meas: [f64; 2],
    pub stance_sides: Vec<usize>,
    pub stance: Vec<usize>,
    pub nc: usize,
    pub ramp: LoadRamp,
}

/// Phase 4: the managed-contact rate trigger, the contact set (the measured
/// correction, the scheduled / loaded / confirmed admissions, the fresh
/// anchors) and the double support's load ramp.
#[allow(clippy::too_many_arguments)]
pub(crate) fn phase_contacts(
    ctl: &mut CtlState,
    plan: &Plan,
    stats: &mut WalkerStats,
    anchors: &mut Anchors,
    cfg: &Config,
    k: &WalkerConsts,
    obs: &Observation,
    sup: &SupportOut,
    t: f64,
) -> ContactsOut {
    // Trigger Managed on the RATE of the normal force, not only its level.
    // Measured on the fixed-arm stair transfer: during the chatter fz swings
    // 15 <-> 78 N, so a level test catches only the dips and leaves the
    // contact rigid through the rest of the oscillation. The chatter runs at
    // ~1200 N/s against ~200 N/s for an intended transfer, so the rate
    // separates them. 0 = off (shipped behaviour).
    let stair_managed_rate = cfg.managed.rate;
    // Hold Managed this long once tripped, so the trigger does not itself
    // chatter at the rate it is suppressing.
    let stair_managed_latch = cfg.managed.latch;
    // Solve against the contact set the FEET report, not the one the schedule
    // assumes. CONTACT_DRIVEN=0 restores the schedule-only behaviour for A/B.
    let contact_driven = cfg.contact.driven;
    let contact_admit_scheduled = cfg.contact.admit_scheduled;
    let contact_admit_z = cfg.contact.admit_z;
    // CONTACT_ADMIT_LOADED="z_tol,fz_frac": a scheduled landing whose sole
    // is OFF its planned height by more than CONTACT_ADMIT_Z but within
    // z_tol, and which measures at least fz_frac of body weight, is admitted
    // too, anchored at its MEASURED height. Measured 2026-10-15 on the
    // unmodelled-bump bench: a foot landing on a 12 mm bump failed the 10 mm
    // gate, carried 27 N (under the 10% BW hysteresis) for the whole double
    // support while the QP balanced on the rear foot (n_stance = 1), the DCM
    // error grew, the next swing was retimed to 0.2 s and stomped at 1860 N.
    // Unset = the 10 mm gate alone (bit-identical).
    let contact_admit_loaded = cfg.contact.admit_loaded;
    let debug = cfg.debug;
    let contact_admit_confirmed = cfg.contact.admit_confirmed;
    let no_lift = cfg.gait.no_lift;
    // Double-support load ramp: the fraction of the double support over
    // which the landed foot takes the load.
    let ramp_frac = cfg.contact.load_ramp_frac;
    let (dt, total_mass) = (k.dt, k.total_mass);
    let robot = k.robot;
    let prof = &robot.prof;
    let data = &obs_state(obs).data;
    let (support, steps, slice_frac) = (sup.support, &sup.steps, sup.slice_frac);
    let fz_meas = [obs.contact_fz[0], obs.contact_fz[1]];
    if stair_managed_rate > 0.0 {
        for side in 0..2 {
            // EMA over ~4 ticks: one noisy sample must not trip it.
            let r = (fz_meas[side] - ctl.fz_prev[side]).abs() / dt.max(1e-9);
            ctl.fz_rate_ema[side] += 0.25 * (r - ctl.fz_rate_ema[side]);
            if ctl.fz_rate_ema[side] > stair_managed_rate {
                ctl.managed_until[side] = t + stair_managed_latch;
            }
        }
    }
    ctl.fz_prev = fz_meas;
    let stance_sides: Vec<usize> = if contact_driven {
        let (mut sides, mut fresh) = ctl.correction.update(support, fz_meas);
        // Planned admission on ANY scheduled touchdown, flat included:
        // entering a double support, a foot the schedule now calls
        // stance whose sole is within CONTACT_ADMIT_Z of its planned
        // height is certified for that double support. Measured
        // 2026-09-24 under the weighted QP: a gentle flat touchdown
        // carried 5 N, stayed under the 10% BW hysteresis for the whole
        // double support, and the QP balanced on the rear foot alone.
        // (CONTACT_ADMIT_SCHEDULED_OFF_STAIRS, only outside the stair
        // zone, is removed.)
        if contact_admit_scheduled
            && matches!(support, Support::Double)
        {
            let now_slice = plan.gait.index_at(t);
            if let Some(prev) = now_slice.checked_sub(1)
                && let Some(landed) = plan.gait.slices[prev].support.swing()
                && ctl.confirmed_admit[landed].is_none()
                && !sides.contains(&landed)
                && ctl.last_scheduled_admit[landed] != Some(now_slice)
            {
                let mi = robot.foot_mi[landed];
                let sole_z = misarta::se3::translation(&data.oMi[mi]).z - prof.sole_below_origin;
                let dz_plan = (sole_z - steps.sole[landed].z).abs();
                if dz_plan < contact_admit_z {
                    ctl.confirmed_admit[landed] = Some(now_slice);
                    ctl.last_scheduled_admit[landed] = Some(now_slice);
                    ctl.admit_measured_z[landed] = None;
                } else if let Some((z_tol, fz_frac)) = contact_admit_loaded
                    && dz_plan < z_tol
                    && fz_meas[landed] >= fz_frac * total_mass * G
                {
                    ctl.confirmed_admit[landed] = Some(now_slice);
                    ctl.last_scheduled_admit[landed] = Some(now_slice);
                    ctl.admit_measured_z[landed] = Some(sole_z);
                    if debug.contact_admit {
                        println!("  [adm-loaded] t={t:.3} side={landed} sole_z={sole_z:.4} plan_z={:.4} fz={:.0}", steps.sole[landed].z, fz_meas[landed]);
                    }
                }
            }
        }
        if contact_admit_confirmed || contact_admit_scheduled {
            let now_slice = plan.gait.index_at(t);
            for side in 0..2 {
                let Some(ds_slice) = ctl.confirmed_admit[side] else { continue };
                if now_slice > ds_slice {
                    // The double support after the landing is over.
                    ctl.confirmed_admit[side] = None;
                    continue;
                }
                // Only inside that double support (not on the confirm
                // tick itself, whose schedule may still say "swing").
                // Re-admit as often as the hysteresis drops the foot:
                // measured 2026-09-24, a confirmed landing is admitted
                // by its own 106 N impact, bounces under the 2% BW
                // release threshold two ticks later, and then rests at
                // 30 N -- under the 10% BW re-entry -- for the rest of
                // the double support. The lifting foot at the END of a
                // later double support is not covered by this: its
                // certificate expired with its own landing's DS.
                if now_slice == ds_slice
                    && support.is_stance(side)
                    && !sides.contains(&side)
                    && ctl.correction.admit(side)
                {
                    sides.push(side);
                    sides.sort_unstable();
                    fresh.push(side);
                    stats.n_confirmed_admissions += 1;
                    if debug.contact_admit {
                        let mi = robot.foot_mi[side];
                        let sole_z = misarta::se3::translation(&data.oMi[mi]).z - prof.sole_below_origin;
                        println!(
                            "  [adm] t={t:.3} side={side} sole_z={sole_z:.4} plan_z={:.4} fz=({:.0},{:.0}) support={support:?}",
                            steps.sole[side].z, fz_meas[0], fz_meas[1]
                        );
                    }
                }
            }
        }
        for side in fresh {
            // A foot that has just become load-bearing needs a fresh
            // anchor: inheriting the stale one demands a force to close a
            // gap the contact cannot close.
            //
            // Take x, y and orientation from the measurement, but put z on
            // the GROUND. A contact anchor describes where the foot meets
            // the floor, and admitting a foot that is still 3-5 mm up --
            // which is exactly what a bouncing touchdown looks like --
            // anchors it in mid-air, so the Baumgarte term then works to
            // HOLD IT THERE at kp_c = 1600 while the QP plans a reaction
            // that does not exist. Measured, that is what turned a landed
            // foot into a hovering one for the rest of the run.
            let mi = robot.foot_mi[side];
            let mut p = misarta::se3::translation(&data.oMi[mi]);
            // A landing admitted on an unmodelled surface (see
            // CONTACT_ADMIT_LOADED) is anchored where it actually is.
            p.z = ctl.admit_measured_z[side].unwrap_or(steps.sole[side].z) + prof.sole_below_origin;
            let rr = misarta::se3::rotation_matrix(&data.oMi[mi]);
            anchors.touchdown(side, p, rr);
        }
        let sched: Vec<usize> = (0..2).filter(|&s| support.is_stance(s)).collect();
        if sides != sched {
            stats.n_corrected += 1;
        }
        sides
    } else {
        (0..2).filter(|&s| support.is_stance(s)).collect()
    };
    let stance: Vec<usize> = stance_sides.iter().map(|&s| robot.foot_mi[s]).collect();
    let nc = stance.len();

    // Load ramps across double support. The foot that just landed takes
    // its share on gradually and the foot about to lift gives it up
    // gradually: the CoP box is |m| <= L*fz, so ramping fz is the only
    // way to shrink a foot's authority without a step change in the
    // constraint set.
    let (landing, lifting) = if matches!(support, Support::Double) && !no_lift {
        let i = plan.gait.index_at(t);
        let landed = if i > 0 {
            plan.gait.slices[i - 1].support.swing()
        } else {
            None
        };
        let next = plan.gait.slices.get(i + 1).and_then(|s| s.support.swing());
        (landed, next)
    } else {
        (None, None)
    };
    // (LOAD_RAMP_TD, keeping the share the landed foot already carries,
    // and LOAD_RAMP_CONTACT_T, ramping from the contact-entry tick too,
    // were not adopted and are removed.)
    let ramp = LoadRamp { landing, lifting, slice_frac, ramp_frac, foot_mi: robot.foot_mi };
    ContactsOut { fz_meas, stance_sides, stance, nc, ramp }
}

/// What phase 5 found: the measured DCM, the DCM reference (a landed
/// recovery reference's own when one is engaged) and the error between them.
pub(crate) struct DcmOut {
    pub xi: na::Vector2<f64>,
    pub r: DcmRef,
    pub dcm_err: f64,
}

/// Phase 5: the DCM and its reference, then the plan's push responses in
/// their order: PUSH_YIELD (slow the walk), PUSH_REPLAN (re-lay the landing
/// after a backward push), PUSH_Z_DROP (detect; the drop is phase 8's) and
/// the engagement of a recovery reference.
#[allow(clippy::too_many_arguments)]
pub(crate) fn phase_dcm(
    ctl: &mut CtlState,
    plan: &mut Plan,
    stats: &mut WalkerStats,
    cfg: &Config,
    k: &WalkerConsts,
    obs: &Observation,
    actual: &Footsteps,
    sup: &SupportOut,
    t: f64,
) -> DcmOut {
    // PUSH_YIELD_V [m/s]: when a push is detected during a speed command
    // (DCM error above PUSH_YIELD_TRIG), re-plan the remaining steps at this
    // lower speed with the schedule's stride cap and step time, once per
    // run. Measured 2026-10-16: the pushes at 0.50 m/s are lost 2-3 steps
    // after the push through the lateral authority, and the same pushes at
    // 0.30 survive 133/192. 0 = off (bit-identical).
    let push_yield_v = cfg.speed.yield_v;
    let vx2_sched = cfg.speed.sched;
    let push_yield_vmin = cfg.speed.yield_vmin;
    let push_yield_trig = cfg.speed.yield_trig;
    let push_yield_no_fwd = cfg.speed.yield_no_fwd;
    let vx2_tscale_min = cfg.speed.tscale_min;
    let debug = cfg.debug;
    let push_replan_trig = cfg.push_replan.trig;
    let stair_crouch_lead = cfg.stair_zone.lead;
    let stair_crouch_tail = cfg.stair_zone.tail;
    let push_replan_slope_lead = cfg.push_replan.slope_lead;
    let push_replan_freeze = cfg.push_replan.freeze;
    // PUSH_REPLAN_ANKLE="ax,ay": fraction of the sole's half length / width
    // the ankle is credited with holding before the error is carried forward.
    // Measured 2026-10-21: crediting the full 85 mm half length took the
    // backward 14 N.s fallers from 8/12 to 0/12 (a heel CoP does not hold a
    // backward push); laterally it keeps a small error from reading as a 1 m
    // step. Default 0 / 1.
    let push_replan_ankle = cfg.push_replan.ankle;
    // PUSH_REPLAN_BACK_ONLY=1: engage only on a backward-dominant error
    // (behind, and more than sideways). PUSH_REPLAN_VMAX [m/s]: never while a
    // VX2 speed command above this is in force (the speed yield re-plans
    // there). Measured 2026-10-21 (tu42, all directions): backward 14 N.s
    // 9 -> 42, but forward / lateral 3-5 members lost each and 0.50 m/s
    // backward 44 -> 21.
    let push_replan_back_only = cfg.push_replan.back_only;
    let push_replan_vmax = cfg.push_replan.vmax;
    // The current DCM error must also exceed this [m] (a 2 mm error at the
    // start of a long first support carried 0.8 s forward read as 4 cm).
    let push_replan_emin = cfg.push_replan.emin;
    let push_replan_max_back = cfg.push_replan.max_back;
    let push_replan_max_fwd = cfg.push_replan.max_fwd;
    let push_replan_min_w = cfg.push_replan.min_w;
    // Widest outward step [m] from the stance foot.
    let push_replan_max_w = cfg.push_replan.max_w;
    let push_replan_stand = cfg.push_replan.stand;
    let push_replan_ramp = cfg.push_replan.ramp;
    let push_z_drop = cfg.push_crouch.drop;
    let push_z_drop_trig = cfg.push_crouch.trig;
    let push_z_drop_lat_max = cfg.push_crouch.lat_max;
    let push_z_drop_t = cfg.push_crouch.t;
    // No reference when the push lands this late in the swing (the walker
    // copes on its own there; measured in-place 5 N s at phase 0.7: 3/3).
    let ref_trig_max_frac = cfg.recovery.trig_max_frac;
    // Detector: lateral DCM error toward the stance side [m] that engages a
    // reference (0 = off). (The bench trigger at the push, REF_TRIG_AT_PUSH /
    // REF_TRIG_MIN_IMPULSE -- an oracle a robot does not have -- and the
    // forward-push detector REF_TRIG_DCM_X are removed.)
    let ref_trig_dcm = cfg.recovery.trig_dcm;
    // 0 = enter the reference at its start even when the detector fired late
    // (for references solved with the detection delay in their initial state).
    let ref_delay_comp = cfg.recovery.delay_comp;
    let omega = k.omega;
    let (sole_half_l, sole_half_w) = (k.sole_half_l, k.sole_half_w);
    let (vx, t_step, gp, stairs) = (k.vx, k.t_step, k.gp, k.stairs);
    let (vx2_change, vx2_cmd_speed, vx2_z_from) = (k.vx2_change, k.vx2_cmd_speed, k.vx2_z_from);
    let vxs_cap_of = |v: f64| -> f64 { cfg.speed.stride_cap(v) };
    let ref_libs = k.ref_libs;
    let ref_any = !ref_libs.is_empty();
    let st = obs_state(obs);
    let (com, com_vel) = (st.com, st.com_vel);
    let (support, slice_frac, slice_idx) = (sup.support, sup.slice_frac, sup.slice_idx);
    let xi = dcm_of(&com, &com_vel, omega);
    let mut r = plan.dcm.reference(t);
    // For a LANDED (re-step) reference the DCM law tracks the
    // reference's own DCM and CoP (its CoM and contact forces) instead of
    // the re-planned LIPM. Measured 2026-10-13 on ld7_a: without it the
    // walker's CoM x lagged the reference by 6 cm at the second landing
    // and 12 cm at the end and the robot fell backward; with it the CoM
    // tracks within 3 cm. (REF_WBC_LANDED, default on, is removed with
    // REF_WBC; the detectors only pick a landed reference when the
    // library has no other.)
    if let (Some(mode), Some(lib)) = (ctl.ref_mode, ctl.ref_lib.as_ref())
        && lib.landed
    {
        let tau = t - mode.t0 + mode.tau0;
        if tau <= lib.total() {
            let xi = mode.world_xy(lib, lib.dcm_at(tau, omega));
            let xi1 = mode.world_xy(lib, lib.dcm_at(tau + 0.01, omega));
            r.xi = xi;
            r.xi_dot = (xi1 - xi) / 0.01;
            r.p = mode.world_xy(lib, lib.cop_at(tau));
        }
    }
    let dcm_err = (xi - r.xi).norm();
    if push_yield_v > 0.0
        && !ctl.push_yield_done
        && vx2_sched
        && let Some(v_hi) = vx2_cmd_speed
        && v_hi > push_yield_v
        // PUSH_YIELD_VMIN: only from this commanded speed up (at 0.30 m/s
        // the lowering alone handles the pushes and the yield cost the
        // backward 11 N.s 48 -> 46, 2026-10-16). 0 = any speed.
        && v_hi >= push_yield_vmin
        && vx2_z_from.is_some_and(|t0| t >= t0)
        && dcm_err > push_yield_trig
        // PUSH_YIELD_NO_FWD=1: never yield on a FORWARD error. Measured
        // 2026-10-16 at 0.50 m/s (N=48): yielding to 0.20 took the
        // backward 11 N.s from 19 to 44 but the forward 11 N.s from 45
        // to 34 -- slowing the plan under a forward push leaves the body
        // ahead of it.
        && !(push_yield_no_fwd && (xi.x - r.xi.x) > 0.5 * push_yield_trig)
    {
        ctl.push_yield_done = true;
        let v_lo = push_yield_v;
        let stride_hi = (v_hi * t_step).min(vxs_cap_of(v_hi));
        let stride_lo = (v_lo * t_step).min(vxs_cap_of(v_lo));
        let scale_hi = (stride_hi / (v_hi * t_step)).clamp(vx2_tscale_min, 1.0);
        let scale_lo = (stride_lo / (v_lo * t_step)).clamp(vx2_tscale_min, 1.0);
        let ratio = scale_lo / scale_hi;
        // Every future landing advances by stride_lo instead of
        // stride_hi: the n-th future step is pulled back n * delta.
        let delta = stride_hi - stride_lo;
        let i_now = plan.gait.index_at(t);
        let mut n = 0usize;
        let mut shift = [0.0_f64; 2];
        for j in (i_now + 1)..plan.gait.slices.len() {
            if let Some(sw) = plan.gait.slices[j].support.swing() {
                n += 1;
                shift[sw] = n as f64 * delta;
            }
            if j < plan.footsteps.at.len() {
                for side in 0..2 {
                    plan.footsteps.at[j].sole[side].x -= shift[side];
                }
            }
        }
        // And every future slice runs at the slower speed's duration.
        for j in (i_now + 1)..plan.gait.slices.len() {
            let t0 = plan.gait.slices[j].t0;
            let d = plan.gait.slices[j].duration() * ratio;
            plan.gait.retime(j, t0 + d);
        }
        plan.rebuild_dcm();
        stats.n_replan += 1;
        if debug.replan { println!("  [diag-replan] site 0 t={t:.3} (line 6798)"); }
        println!("  [yield] t={t:.3} dcm_err={dcm_err:.3}: speed {v_hi:.2} -> {v_lo:.2} m/s, stride {stride_hi:.3} -> {stride_lo:.3}, slices x{ratio:.3}, {n} steps re-planned");
    }
    if push_replan_trig > 0.0 {
        let i = plan.gait.index_at(t);
        let sl = plan.gait.slices[i].clone();
        match sl.support {
            Support::Single { swing, stance }
                if ctl.ref_mode.is_none()
                    // Not on a known staircase (its zone, with the crouch
                    // lead / tail): a deep step-down reads as a backward
                    // error and the re-plan cost the -150 mm descent
                    // 41 -> 30 (measured 2026-10-22).
                    && !stairs.as_ref().is_some_and(|st| {
                        // A zero-riser "stair" (the flat benches park one
                        // at x = 2 m) is not a staircase.
                        (st.riser.abs() > 1e-6 || st.down_n > 0)
                            && articara::biped::height::stair_zone_crouch(
                            plan.footsteps.at_slice(i).mid_xy().x,
                            st,
                            stair_crouch_lead,
                            stair_crouch_tail,
                            1.0,
                        ) > 0.0
                    })
                    // PUSH_REPLAN_SLOPE_LEAD (m): nor on a known slope,
                    // from this far before its start to its end. The
                    // zero-riser test above lets a slope through, and
                    // there the re-plan costs the backward 11 N.s 18 -> 4
                    // (measured 2026-10-19: the re-laid plan meets the
                    // slope entry with a backward DCM error, and the
                    // second firing steps back off the slope). 0 = off.
                    && !(push_replan_slope_lead > 0.0
                        && stairs.as_ref().is_some_and(|st| {
                            let x = plan.footsteps.at_slice(i).mid_xy().x;
                            st.n == 0
                                && st.slope != 0.0
                                && x >= st.start - push_replan_slope_lead
                                && x <= st.start + st.slope_len
                        })) =>
            {
                // The plan in force when this single support began.
                if ctl.push_replan_state.as_ref().is_none_or(|st| st.0 != i) {
                    let base = plan.footsteps.at_slice(i).xy(swing);
                    ctl.push_replan_state = Some((i, plan.footsteps.clone(), plan.dcm.clone(), base, None));
                }
                let frac = (t - sl.t0) / sl.duration().max(1e-9);
                let rem = (sl.t1 - t).clamp(0.0, gp.t_ss);
                if let Some((_, orig_fs, orig_plan, base, last)) = ctl.push_replan_state.as_mut()
                    && frac < push_replan_freeze
                {
                    let e = xi - orig_plan.reference(t).xi;
                    // The ankle can hold an error up to the sole's half
                    // size by moving the CoP; only the rest diverges.
                    let half = na::Vector2::new(sole_half_l * push_replan_ankle[0], sole_half_w * push_replan_ankle[1]);
                    let grow = (omega * rem).exp();
                    let d = na::Vector2::new(
                        {
                            let dp = e.x.clamp(-half.x, half.x);
                            dp + (e.x - dp) * grow
                        },
                        {
                            let dp = e.y.clamp(-half.y, half.y);
                            dp + (e.y - dp) * grow
                        },
                    );
                    let v_now = (vx2_sched && vx2_z_from.is_some_and(|t0| t >= t0))
                        .then_some(vx2_cmd_speed)
                        .flatten();
                    let back_ok = (!push_replan_back_only || (e.x < 0.0 && e.y.abs() < -e.x))
                        && !v_now.is_some_and(|v| v > push_replan_vmax);
                    // (PUSH_REPLAN_LAT, the outward lateral step too, is
                    // removed.)
                    if ctl.push_replan_slice == Some(i)
                        || (back_ok && d.norm() > push_replan_trig && e.norm() > push_replan_emin)
                    {
                        let st = orig_fs.at_slice(i).xy(stance);
                        let mut l = *base + d;
                        l.x = l.x.clamp(st.x - push_replan_max_back, st.x + push_replan_max_fwd);
                        // Lateral: +1 when the swing foot normally lands at +y of the stance foot.
                        let side = if base.y > st.y { 1.0 } else { -1.0 };
                        let w = (l.y - st.y) * side;
                        // Never widen beyond what the plan already had: the
                        // floor is the smaller of MIN_W and the planned gap
                        // (G1 walks at ~0.104 m; a 0.12 floor widened every
                        // backward re-plan by 2 cm, measured 2026-10-21).
                        let min_w = push_replan_min_w.min(((base.y - st.y) * side).max(0.0));
                        // Inward of the minimum gap (a crossing): leave this
                        // landing to the MPC (its crossing gate,
                        // MPC_FOOTSTEP_CROSS_*) -- do not engage, and once
                        // engaged keep the last landing.
                        let inward_skip = w < 0.0;
                        let w = if w >= min_w {
                            w.min(push_replan_max_w)
                        } else {
                            min_w
                        };
                        l.y = st.y + side * w;
                        if !inward_skip && last.is_none_or(|p| (p - l).norm() > 0.005) {
                            let mut fs = orig_fs.clone();
                            fs.replan_from_landing(&plan.gait, i, swing, l, push_replan_stand, push_replan_ramp);
                            plan.footsteps = fs;
                            plan.rebuild_dcm();
                            if ctl.push_replan_slice != Some(i) {
                                stats.n_push_replan += 1;
                                stats.n_replan += 1;
                                if debug.replan { println!("  [diag-replan] site 1 t={t:.3} (line 6901)"); }
                            }
                            ctl.push_replan_slice = Some(i);
                            ctl.push_replan_shift = (l - *base).norm();
                            if debug.push_replan {
                                println!("  [replan] t={t:.3} slice {i} frac={frac:.2} e=({:+.3},{:+.3}) d=({:+.3},{:+.3}) landing ({:.3},{:+.3}) -> ({:.3},{:+.3})", e.x, e.y, d.x, d.y, base.x, base.y, l.x, l.y);
                            }
                            *last = Some(l);
                        }
                    }
                }
            }
            _ => {}
        }
    }
    // Backward-dominant only, and never under an engaged recovery
    // reference (measured: firing on lateral pushes cost the lateral
    // ladder 136 -> 129 while the reference was running).
    // (The rate-predicted error, REF_TRIG_PRED, is removed: at its
    // default 0 the prediction was the error itself.)
    let back_err = (xi - r.xi).x;
    if push_z_drop > 0.0
        && back_err < -push_z_drop_trig
        && (xi - r.xi).y.abs() < push_z_drop_lat_max
        && ctl.ref_mode.is_none()
        && t >= ctl.push_z_drop_until
    {
        ctl.push_z_drop_until = t + push_z_drop_t;
        stats.n_push_z_drop += 1;
        println!("  [zdrop] t={t:.3} backward DCM error {:+.3} m -> CoM {:.3} m lower for {push_z_drop_t:.2} s", (xi - r.xi).x, push_z_drop);
    }
    // Engage a recovery reference when the DCM error toward the STANCE
    // side exceeds REF_TRIG_DCM in single support (the detector).
    // (REF_SLOPE_GATE, no reference on a known slope, is removed.)
    if ref_any
        && ctl.ref_mode.is_none()
        && t >= ctl.ref_cooldown_until
        && slice_frac <= ref_trig_max_frac
        && let Support::Single { stance, swing } = support
    {
        let lateral = (xi - r.xi).y * if stance == 0 { 1.0 } else { -1.0 };
        if lateral.abs() < 0.005 {
            ctl.ref_lat_quiet_t = t;
        }
        let forward = (xi - r.xi).x;
        // The library's lateral-push references (`impulse_deg` 90).
        let want_deg = 90.0;
        if ref_trig_dcm > 0.0 && lateral > ref_trig_dcm && ref_libs.iter().any(|l| l.impulse_deg == want_deg) {
            // The push happened when the lateral error last was quiet;
            // the reference is entered that far in.
            let tau0 = if !ref_delay_comp { 0.0 } else { (t - ctl.ref_lat_quiet_t).clamp(0.0, 0.3) };
            println!("  [ref] t={t:.3} trigger: dcm (lateral DCM error toward stance {lateral:+.3} m, forward {forward:+.3} m, dir {want_deg:.0}, tau0 {tau0:.3})");
            // Pick the reference meant for the nearest swing phase.
            // (Selecting by the swing foot's height was tried first:
            // on the flat in-place gait the apex comes late, so the
            // height does not order the phases.)
            // Phase AT THE PUSH (the detector fires tau0 later).
            let frac_push = slice_frac - tau0 / plan.gait.slices[slice_idx].duration().max(1e-6);
            // Nearest walking speed first (the commanded speed once the
            // speed change has ramped in, else the nominal), then phase.
            let v_now = match vx2_change {
                Some((at, v2, ramp2)) if plan.gait.steps_taken(t) >= at + ramp2 => v2,
                _ => vx,
            };
            let pick = (0..ref_libs.len())
                .filter(|&k| ref_libs[k].impulse_deg == want_deg && !ref_libs[k].landed)
                .min_by(|&a, &b| {
                    let da = ((ref_libs[a].vx - v_now).abs() * 10.0, (ref_libs[a].swing_phase - frac_push).abs());
                    let db = ((ref_libs[b].vx - v_now).abs() * 10.0, (ref_libs[b].swing_phase - frac_push).abs());
                    (da.0 + da.1).partial_cmp(&(db.0 + db.1)).unwrap()
                })
                .unwrap_or(0);
            ctl.ref_lib = Some(ref_libs[pick].clone());
            let lib = ctl.ref_lib.as_ref().unwrap();
            println!("  [ref] swing phase {slice_frac:.2} -> reference {} (meant for {:.2})", pick, lib.swing_phase);
            let mode = RefMode { t0: t, tau0, stance, origin: actual.sole[stance], mirror: if stance == 0 { 1.0 } else { -1.0 } };
            // Phases: this single support ends after phases[0]; the
            // double support after it (every DS slice up to the next
            // single support) gets phases[1]; the next single support
            // phases[2]; the double support after that phases[3].
            let i0 = slice_idx;
            let mut j = i0;
            let mut ph = 0usize;
            let mut t_cursor = t;
            let mut first_ss_after: Option<usize> = None;
            while j < plan.gait.slices.len() && ph < lib.phases.len() {
                let is_ss = matches!(plan.gait.slices[j].support, Support::Single { .. });
                // Count the slices of this phase (DS phases may be split).
                let mut jend = j + 1;
                if !is_ss {
                    while jend < plan.gait.slices.len() && matches!(plan.gait.slices[jend].support, Support::Double) {
                        jend += 1;
                    }
                }
                let n = (jend - j) as f64;
                for jj in j..jend {
                    let remaining = if ph == 0 { (lib.phases[0] - tau0).max(0.05) } else { lib.phases[ph] };
                    let end = t_cursor + remaining / n;
                    plan.gait.retime(jj, end);
                    t_cursor = end;
                }
                if is_ss && j != i0 && first_ss_after.is_none() {
                    first_ss_after = Some(j);
                }
                j = jend;
                ph += 1;
            }
            // Footfalls: every single-support phase of the reference
            // lands its swing foot (reference right, left, right, ...)
            // where the reference puts it at the end of that phase.
            let mut t_end = 0.0;
            let mut jj = i0;
            let mut ref_sw = 1usize;
            let mut land1 = na::Vector3::zeros();
            let mut land2 = na::Vector3::zeros();
            for (ph, dur) in lib.phases.iter().enumerate() {
                t_end += dur;
                if ph % 2 == 0 {
                    // find the single-support slice of this phase
                    while jj < plan.gait.slices.len() && plan.gait.slices[jj].support.swing().is_none() {
                        jj += 1;
                    }
                    if jj >= plan.gait.slices.len() {
                        break;
                    }
                    let foot = mode.foot(ref_sw);
                    let land = mode.world(lib, lib.sole_at(ref_sw, t_end));
                    let d = (land - plan.footsteps.at_slice(jj).sole[foot]).xy();
                    plan.footsteps.shift_next(jj, foot, d);
                    if ph == 0 { land1 = land; } else if ph == 2 { land2 = land; }
                    ref_sw = 1 - ref_sw;
                    jj += 1;
                }
            }
            let _ = first_ss_after;
            plan.rebuild_dcm();
            plan.landing.event_mode = true;
            ctl.ref_mode = Some(mode);
            stats.ref_engagements += 1;
            println!(
                "  [ref] t={t:.3} engaged: stance={} mirror={:+.0} origin=({:.3},{:.3}) phases={:?} land1=({:.3},{:.3}) land2=({:.3},{:.3}) total={:.3}",
                stance, mode.mirror, mode.origin.x, mode.origin.y, lib.phases, land1.x, land1.y,
                land2.x, land2.y, lib.total()
            );
        
        }
    }
    DcmOut { xi, r, dcm_err }
}

/// What phase 6 found: the commanded ZMP clamped into the support, how far
/// the clamp moved it, and the pressure it refused (MOM_CMP's input).
pub(crate) struct CmdZmpOut {
    pub p_cmd: na::Vector2<f64>,
    pub clamped: f64,
    pub zmp_deficit: na::Vector2<f64>,
}

/// Phase 6: the commanded ZMP and the contact planner (CONTACT_PLAN: the
/// landing shift and the step's end time).
#[allow(clippy::too_many_arguments)]
pub(crate) fn phase_cmd_zmp(
    ctl: &mut CtlState,
    plan: &mut Plan,
    stats: &mut WalkerStats,
    cfg: &Config,
    k: &WalkerConsts,
    sup: &SupportOut,
    dcm: &DcmOut,
    t: f64,
) -> CmdZmpOut {
    let k_dcm = cfg.dcm.k_dcm;
    let cop_frac = cfg.dcm.cop_frac;
    let zmp_margin = cfg.dcm.zmp_margin;
    let clamp_zmp = cfg.dcm.clamp_zmp;
    let contact_plan_on = cfg.contact_plan.on;
    // Only plan when the DCM is actually off the plan (same reason as the
    // removed ADAPT_TIME_TRIG: the undisturbed gait must reproduce bit for
    // bit).
    let contact_plan_trig = cfg.contact_plan.trig;
    let debug = cfg.debug;
    let (dt, omega) = (k.dt, k.omega);
    let (sole_half_l, sole_half_w) = (k.sole_half_l, k.sole_half_w);
    let contact_plan_cfg = k.contact_plan_cfg;
    let (support, steps) = (sup.support, &sup.steps);
    let (xi, r, dcm_err) = (dcm.xi, &dcm.r, dcm.dcm_err);
    let p_raw = commanded_zmp(&xi, &r, omega, k_dcm);
    // Clamp against the support that is REALLY there, which in NO_LIFT is
    // both feet even where the plan says one.
    let sbox = SupportBox::from_stance_yawed(
        &steps,
        support,
        (sole_half_l * cop_frac, sole_half_w * cop_frac),
        zmp_margin,
    );
    let (p_cmd, clamped) = if clamp_zmp {
        sbox.clamp(&p_raw)
    } else {
        (p_raw, 0.0)
    };
    // Pressure the polygon refused (world x, y), for MOM_CMP.
    let zmp_deficit = p_raw - p_cmd;

    // ---- contact plan (CONTACT_PLAN) --------------------------------
    if contact_plan_on
        && ctl.ref_mode.is_none()
        && let Support::Single { swing, .. } = support
        && dcm_err >= contact_plan_trig
    {
        let i = plan.gait.index_at(t);
        if ctl.cp_slice != Some(i) {
            ctl.cp_slice = Some(i);
            ctl.cp_applied = na::Vector2::zeros();
        }
        let seg = &plan.dcm.segs[plan.dcm.index_at(t)];
        let sit = articara::biped::contact_plan::Situation {
            xi,
            p: seg.at(t),
            p_dot: seg.slope(),
            omega,
            t,
            nominal_end: plan.gait.slices[i].t1,
            xi_eos_plan: plan.dcm.eos_at(t),
            swing_sign: if swing == 0 { 1.0 } else { -1.0 },
        };
        // (CP_J_AUTO, the sensitivity differenced from the plan, and
        // CP_TOUCHDOWN, planning for the DCM at touchdown, were not
        // adopted and are removed.)
        let dec = articara::biped::contact_plan::plan(&sit, &contact_plan_cfg);
        if debug.cp {
            println!(
                "  [cp] t={t:.3} slice={i} err={dcm_err:.4} xi=({:.3},{:+.3}) eos_plan=({:.3},{:+.3}) pred_nom=({:.3},{:+.3}) shift=({:+.3},{:+.3}) dT={:+.3} unmet={:.4} cost={:.3}",
                xi.x, xi.y, sit.xi_eos_plan.x, sit.xi_eos_plan.y,
                articara::biped::contact_plan::predict(&sit, (sit.nominal_end - t).max(0.0)).x,
                articara::biped::contact_plan::predict(&sit, (sit.nominal_end - t).max(0.0)).y,
                dec.shift.x, dec.shift.y, dec.new_end.map_or(0.0, |e| e - sit.nominal_end), dec.unmet, dec.cost
            );
        }
        // (CP_TREAD_BOX, snapping the landing into the tread's sole
        // window, MEASURED HARMFUL 2026-09-25: landing fully on the tread
        // does not predict survival at the 60 mm riser, and snapping to
        // the window edge took 26/32 -> 17/32. Removed; terrain awareness
        // belongs in a cost with margin, not a snap to an edge.)
        let mut rebuilt = false;
        // Like ADAPT_STEP did: place the foot once per step, keeping the
        // largest demand seen, instead of re-placing it every tick.
        //
        // That is a RATCHET: the landing only ever moves further from
        // nominal. With the shipped 15 mm trigger it fires a handful of
        // times in an emergency and the ratchet is harmless, but run
        // continuously it walks the foot out to its bound and the stair
        // approach dies (measured 2026-09-24 on the randomised stairs:
        // 0/192 at every shift weight, falling at 21-23 s). (CP_TRACK,
        // following the current decision instead, is removed: with the
        // champion's CP_MAX_*=0 the shift is always zero and both rules
        // give zero.)
        let d = if dec.shift.norm() > ctl.cp_applied.norm() {
            dec.shift - ctl.cp_applied
        } else {
            na::Vector2::zeros()
        };
        if d.norm() > 1e-9 && !plan.landing.landing_closed(i) {
            // `shift_from` shifts the swing foot's landing in slice `i`
            // AND every later slice, so the adjusted position becomes the
            // nominal that the NEXT step's residual is measured against.
            // Under continuous firing the foot walks outward one bound per
            // step (measured 2026-09-25: lateral spacing 127 -> 153 mm
            // median against a 30 mm limit). CP_SHIFT_NEXT, moving only
            // this landing, was not adopted and is removed.
            plan.footsteps.shift_from(i, swing, d);
            ctl.cp_applied = dec.shift;
            stats.n_cp_shift += 1;
            rebuilt = true;
        }
        if let Some(ne) = dec.new_end
            && ne > t + dt
            && ctl.cp_retimed_slice != Some(i)
        {
            let dd = plan.gait.retime(i, ne);
            stats.n_cp_retime += 1;
            stats.cp_retime_total += dd.abs();
            ctl.cp_retimed_slice = Some(i);
            // The planner has now set this step's end; the landing
            // extension must not move it again (the two would fight
            // over the same boundary, and extension IS mechanism B).
            plan.landing.max_extension = 0.0;
            rebuilt = true;
        }
        if rebuilt {
            plan.rebuild_dcm();
            stats.n_replan += 1;
            if debug.replan { println!("  [diag-replan] site 2 t={t:.3} (line 7454)"); }
        }
    }
    CmdZmpOut { p_cmd, clamped, zmp_deficit }
}

/// Phase 7: the CoM preview (COM_REPLAN once engaged): re-solve on its own
/// clock and pull the commanded ZMP's x toward the preview's. Returns the
/// commanded ZMP after that correction. Writes PREVIEW_TRAJ_CSV and
/// COM_PREVIEW_RUNTIME_CSV (the harness's sinks, until S13's hook).
#[allow(clippy::too_many_arguments)]
pub(crate) fn phase_preview(
    ctl: &mut CtlState,
    plan: &Plan,
    stats: &mut WalkerStats,
    diag: &mut DiagSinks,
    cfg: &Config,
    k: &WalkerConsts,
    obs: &Observation,
    actual: &Footsteps,
    stance_sides: &[usize],
    mut p_cmd: na::Vector2<f64>,
    t: f64,
) -> na::Vector2<f64> {
    let cop_frac = cfg.dcm.cop_frac;
    let zmp_margin = cfg.dcm.zmp_margin;
    let com_preview_n = cfg.com_plan.preview_n;
    let com_preview_smooth = cfg.com_plan.preview_smooth;
    let com_replan_forward_gain = cfg.com_plan.replan_forward_gain;
    let omega = k.omega;
    let (sole_half_l, sole_half_w) = (k.sole_half_l, k.sole_half_w);
    let (com_preview_dt, com_replan_dt) = (k.com_preview_dt, k.com_replan_dt);
    let st = obs_state(obs);
    let (com, com_vel) = (st.com, st.com_vel);
    if ctl.replan_engaged {
        let measured_support = if stance_sides.len() == 2 {
            Support::Double
        } else {
            let side = stance_sides[0];
            Support::Single {
                stance: side,
                swing: 1 - side,
            }
        };
        let current_box = SupportBox::from_stance_yawed(
            &actual,
            measured_support,
            (sole_half_l * cop_frac, sole_half_w * cop_frac),
            zmp_margin,
        );
        if t + 1e-9 >= ctl.preview_next {
            let mut samples = Vec::with_capacity(com_preview_n);
            for k in 0..com_preview_n {
                let begin = t + k as f64 * com_preview_dt;
                let end = begin + com_preview_dt;
                let f0 = articara::biped::preview::forecast_feet(
                    &plan.gait,
                    &plan.footsteps,
                    &actual,
                    &stance_sides,
                    t,
                    begin,
                );
                let f1 = articara::biped::preview::forecast_feet(
                    &plan.gait,
                    &plan.footsteps,
                    &actual,
                    &stance_sides,
                    t,
                    end - 1e-9,
                );
                let b0 = SupportBox::from_stance_yawed(
                    &f0,
                    plan.gait.support_at(begin),
                    (sole_half_l * cop_frac, sole_half_w * cop_frac),
                    zmp_margin,
                );
                let b1 = SupportBox::from_stance_yawed(
                    &f1,
                    plan.gait.support_at(end - 1e-9),
                    (sole_half_l * cop_frac, sole_half_w * cop_frac),
                    zmp_margin,
                );
                let mut bounds = SupportBox {
                    lo: b0.lo.sup(&b1.lo),
                    hi: b0.hi.inf(&b1.hi),
                };
                if k == 0 {
                    bounds = SupportBox {
                        lo: current_box.lo,
                        hi: current_box.hi,
                    };
                }
                let (zmp_ref, xi_ref) = (
                    plan.dcm.reference(begin + 0.5 * com_preview_dt).p,
                    plan.dcm.reference(end).xi,
                );
                samples.push(articara::biped::preview::PreviewSample {
                    duration: com_preview_dt,
                    bounds,
                    zmp: zmp_ref,
                    xi: xi_ref,
                });
            }
            let preview_solve_start = std::time::Instant::now();
            let preview_solution = articara::biped::preview::replan(
                omega,
                com.xy(),
                com_vel.xy(),
                ctl.preview_p,
                &samples,
                com_preview_smooth,
            );
            let preview_solve_ns = preview_solve_start.elapsed().as_nanos();
            let preview_accepted = preview_solution.is_some();
            match preview_solution {
                Some(plan) => {
                    ctl.preview_p = plan.zmp[0];
                    ctl.preview_c = plan.com[0];
                    ctl.preview_v = plan.velocity[0];
                    stats.preview_error = plan.terminal_error;
                    stats.preview_count += 1;
                    if let Some(f) = diag.preview_traj_file.as_mut() {
                        use std::io::Write as _;
                        for (k, smp) in samples.iter().enumerate() {
                            let (Some(z), Some(c)) = (plan.zmp.get(k), plan.com.get(k)) else { break };
                            writeln!(
                                f,
                                "{t:.4},{k},{:.4},{:.5},{:.5},{:.5},{:.5},{:.5},{:.5},{:.5},{:.5},{:.5},{:.5}",
                                smp.duration, z.x, z.y, c.x, c.y, smp.zmp.x, smp.zmp.y,
                                smp.bounds.lo.x, smp.bounds.lo.y, smp.bounds.hi.x, smp.bounds.hi.y
                            )
                            .unwrap();
                        }
                    }
                }
                None => {
                    ctl.preview_p = current_box.clamp(&p_cmd).0;
                    stats.preview_failures += 1;
                }
            }
            if let Some(writer) = &mut diag.com_preview_runtime {
                writeln!(writer, "{t:.6},{preview_solve_ns},{},{},{:.9},{},{}",
                    u8::from(preview_solve_ns > 50_000_000),
                    u8::from(preview_accepted),
                    stats.preview_error,
                    0, // applied_lateral: COM_REPLAN_LATERAL (removed) was off
                    com_preview_n,
                ).unwrap();
            }
            // (COM_PREVIEW_PHASE_LOCK, a discrete-clock update rate, is
            // removed.)
            ctl.preview_next = t + com_replan_dt;
        }
        let optimized = current_box.clamp(&ctl.preview_p).0;
        // Forward only (COM_REPLAN_LATERAL and its gain are removed).
        p_cmd.x += com_replan_forward_gain * (optimized.x - p_cmd.x);
    }
    p_cmd
}

/// What phase 8 found: the CoM height reference (after the speed / push
/// drops) and the commanded CoM acceleration.
pub(crate) struct HeightOut {
    pub z_ref: f64,
    pub a_com: na::Vector3<f64>,
}

/// Phase 8: the horizontal CoM acceleration from the commanded ZMP (the
/// variable-height form on stairs, the jerk limit), the height reference
/// (terrain, stance level, descent lead, stair crouch, squat, the speed and
/// push drops) and the vertical PD.
#[allow(clippy::too_many_arguments)]
pub(crate) fn phase_height(
    ctl: &mut CtlState,
    plan: &mut Plan,
    cfg: &Config,
    k: &WalkerConsts,
    obs: &Observation,
    sup: &SupportOut,
    stance_sides: &[usize],
    p_cmd: &na::Vector2<f64>,
    t: f64,
) -> HeightOut {
    // VH_DCM=1: the height-varying DCM law on a modelled terrain (see its use).
    let vh_dcm = cfg.dcm.vh_dcm;
    let com_xy_jerk_max = cfg.dcm.com_xy_jerk_max;
    // Risers per step the height reference is allowed to move. Small spreads
    // one riser's climb over several steps; large finishes it within the
    // swing that takes the tread.
    let stair_climb_rate = cfg.com_plan.stair_climb_rate;
    let com_height_replan = cfg.com_plan.height_replan;
    // (LAND_WAIT_HOLD, holding the DCM reference at the stance sole while a
    // stair landing waits, was not adopted and is removed.)
    // Lower the CoM height target while the feet are in the stair zone.
    // Measured 2026-09-24 on G1: the second step onto an 80 mm riser puts
    // the swing sole 0.20 m ahead of the stance sole with the CoM 0.684 m
    // above the tread -- a 0.71 m reach, the leg's full length. The knee
    // arrives straight (0.01-0.06 rad), the swing task can no longer push
    // the sole down (residual 20 -> 600), and the QP throws the leg up.
    // The 0.233 m landing stride (STAIR_LANDING_FRAC) has the same reach.
    // Zone: mid-feet x from `start - lead` to `landing_start + tail`. The
    // target moves through the usual `COM_HEIGHT_TIME` retarget, so it is
    // smooth. Default 0 = shipped height.
    // (Distinct from the older global `STAIR_CROUCH` (removed), which
    // lowered the CoM from t=0 at 50 mm/s and rescaled the LIPM height;
    // measured 2026-09-24 that one fells the flat approach at t=5 on G1.)
    // Aim the CoM height at the LOWER level a descent is about to step
    // onto, before the swing. Measured 2026-10-03 on G1 (v6 + descent
    // extension, 80 mm step-down, 45 fallers of 48): the CoM reference
    // follows the STANCE soles, so it stays at platform height through the
    // whole step-down swing; the swing leg reaches 99.7-99.9% of its length
    // and stops 15-20 mm above the lower surface; the schedule then admits
    // the airborne foot, whose orientation is uncontrolled for ~80 ms while
    // it is loaded, and the corner it happens to land on decides the run
    // (heel-outer corner -> survives, toe-outer corner -> slides 12 cm
    // forward on the toe and falls). Fraction of the drop to the coming
    // landing that the height reference takes in advance: 1 = the full
    // riser, through the usual COM_HEIGHT_TIME retarget. Only a landing
    // BELOW the stance soles counts, so ascents and level walking are
    // bit-identical. Default 0 = off = shipped.
    let com_height_desc_lead = cfg.descent.com_height_lead;
    let stair_crouch_lead = cfg.stair_zone.lead;
    let stair_crouch_tail = cfg.stair_zone.tail;
    let stair_zone_crouch = cfg.descent.zone_crouch;
    let com_height_time = cfg.com_plan.height_time;
    let push_z_drop = cfg.push_crouch.drop;
    let kp_com = cfg.gains.kp_com;
    let kd_com = cfg.gains.kd_com;
    let diag_vz_bias_height = cfg.gains.diag_vz_bias_height;
    let (dt, omega, z_com, terrain_base) = (k.dt, k.omega, k.z_com, k.terrain_base);
    let (vx, t_step, stairs) = (k.vx, k.t_step, k.stairs);
    let (squat_amp, squat_period) = (k.squat_amp, k.squat_period);
    let (vx2_z_from, vx2_z_drop) = (k.vx2_z_from, k.vx2_z_drop);
    let st = obs_state(obs);
    let (com, com_vel) = (st.com, st.com_vel);
    let (slice_idx, steps) = (sup.slice_idx, &sup.steps);
    // Variable-height form when a stair is in the scene: the LIPM's
    // constant-height omega is ~20% too stiff while the CoM is being
    // driven DOWN a riser. Gated so level and validated runs are bit-
    // identical (VH_DCM=0 restores the classic form).
    let a_xy_raw = if stairs.is_some() && vh_dcm {
        // Height above the surface the pressure actually acts on, which
        // on a stair is the STANCE sole -- not the fixed `z_lip` omega
        // was built from. Measured on a descent: omega^2 claims 32.9
        // while the true g/h is 39.7, a 21% error in the commanded ZMP
        // response, in the unsafe direction (too soft).
        let surf = if stance_sides.is_empty() {
            0.0
        } else {
            stance_sides.iter().map(|&s| steps.sole[s].z).sum::<f64>()
                / stance_sides.len() as f64
        };
        articara::biped::dcm::com_accel_xy_vh(&com, &p_cmd, omega, 0.0, surf)
    } else {
        com_accel_xy(&com, &p_cmd, omega)
    };
    let a_xy = if let Some(previous) = ctl.previous_a_xy {
        let delta = a_xy_raw - previous;
        let max_delta = com_xy_jerk_max * dt;
        if delta.norm() > max_delta {
            previous + delta * (max_delta / delta.norm())
        } else {
            a_xy_raw
        }
    } else {
        a_xy_raw
    };
    ctl.previous_a_xy = Some(a_xy);
    // z stays a PD on the nominal height: the LIPM says nothing about it,
    // and a constant-height assumption is exactly what makes omega a
    // constant in the first place.
    // Height reference. `omega` stays at the nominal height: the LIPM is
    // derived for a constant one, and re-deriving it per tick is the
    // variable-height DCM formulation, which this is not. The squat is
    // therefore a disturbance the horizontal loop has to survive, which is
    // the point of having it in the benchmark.
    // Stairs: the height reference rides the terrain. The target is the
    // mean of the two planned sole heights for THIS slice, slewed at
    // 1.5 risers per step so a riser is absorbed over roughly one step.
    // omega stays at the flat-ground value -- same argument as the squat
    // below: this is not the variable-height DCM formulation, and the
    // height change is a disturbance the horizontal loop must survive.
    let zd_terrain = if let Some(st) = &stairs {
        // A DS-only ramp was tried, but left the height reference a
        // riser behind the feet. Slew through both support phases.
        // Foot-over-foot climbs one riser per step, so the rate limit
        // turns the staircase of slice targets into a near-constant
        // slope; no gating on support phase (a DS-only ramp was tried
        // and left the reference a riser behind the feet by tread 2).
        let fs = plan.footsteps.at_slice(slice_idx);
        // The mean planned sole height (STAIR_DESC_MIN, the lower of the
        // two on a descent, is removed).
        let tgt_h = 0.5 * (fs.sole[0].z + fs.sole[1].z);
        // A slope (TERRAIN_SLOPE) has no riser: its rise per step is
        // the grade times the nominal stride, else the rate is zero and
        // the height reference never leaves the spawn level (measured
        // 2026-10-15 on +3 deg: CoM 0.683 absolute for the whole climb).
        let riser_eff = if st.n == 0 && st.slope != 0.0 {
            st.slope.abs() * (vx.abs() * t_step).max(0.10)
        } else {
            st.riser.abs()
        };
        let rate = stair_climb_rate * riser_eff / t_step.max(1e-6);
        let dh = (tgt_h - ctl.terrain_track).clamp(-rate * dt, rate * dt);
        ctl.terrain_track += dh;
        dh / dt
    } else {
        0.0
    };
    let (z_ref, zd_ref, zdd_ref) = if ctl.replan_engaged && com_height_replan && plan.landing.clock.is_some() {
        // Where the CoM height aims while BOTH feet count: historically
        // the mean sole height. Measured 2026-09-24 on the 80 mm riser
        // with the landed foot admitted at the start of double support
        // (CONTACT_ADMIT_CONFIRMED): the half-rise then completes before
        // the trailing leg has left the lower tread, that leg reaches
        // single support with its knee at 0.09 rad (champion: 0.34), and
        // the next swing task has no authority left -- the foot launches
        // to 1.2 m. (COM_HEIGHT_DS_FRAC, keeping the CoM at the lower
        // sole through double support, was not adopted and is removed;
        // the level is the mean.)
        // All height rules live in src/biped/height.rs (one place for the
        // stance level, the descent lead, the stair crouch, the reach
        // caps and the slewed drops); the arithmetic is the one used
        // inline before 2026-10-19, bit-identical.
        use articara::biped::height as hr;
        let soles_z: Vec<f64> = stance_sides.iter().map(|&side| steps.sole[side].z).collect();
        let h = hr::stance_level(&soles_z, None);
        // The landing this swing is heading for, or -- in a double
        // support -- the landing of the swing that follows it.
        let next_landing = {
            let i = plan.gait.index_at(t);
            (i..plan.gait.slices.len().min(i + 4))
                .find_map(|j| plan.gait.slices[j].support.swing().map(|sw| (j, sw)))
                .map(|(j, sw)| (sw, plan.footsteps.at_slice(j).sole[sw]))
        };
        let h = hr::descent_lead(h, next_landing.map(|(_, l)| l.z), com_height_desc_lead);
        let crouch = match &stairs {
            Some(st) => hr::stair_zone_crouch(steps.mid_xy().x, st, stair_crouch_lead, stair_crouch_tail, stair_zone_crouch),
            None => 0.0,
        };
        let target = if crouch > 0.0 {
            z_com + h - terrain_base - crouch
        } else {
            z_com + h - terrain_base
        };
        // (REACH_HEIGHT / _FLOOR, the height capped by the legs' reach
        // over every sole, and COM_HEIGHT_DESC_REACH, the same cap for
        // the landing of a descent, were not adopted and are removed.)
        if (target - plan.height.target).abs() > 1e-6 {
            plan.height.retarget(t, target, com_height_time);
        }
        plan.height.at(t)
    } else if squat_amp > 0.0 {
        let w = 2.0 * PI / squat_period;
        let ph = w * t;
        (
            z_com - squat_amp * (1.0 - ph.cos()) * 0.5,
            -squat_amp * 0.5 * w * ph.sin(),
            -squat_amp * 0.5 * w * w * ph.cos(),
        )
    } else {
        (
            z_com + (ctl.terrain_track - terrain_base),
            zd_terrain,
            0.0,
        )
    };
    // VX2_Z_DROP: eased in at 50 mm/s on top of whichever height
    // reference is live (the flat walk never enters the replan branch).
    // PUSH_Z_DROP: lower the CoM for PUSH_Z_DROP_T after a BACKWARD
    // push (DCM error behind the plan by more than PUSH_Z_DROP_TRIG).
    // Measured 2026-10-05: 24 mm lower for the whole run doubles the
    // backward-push survival (21 -> 43/48) but trades descents away;
    // tied to the detection it leaves the other benches alone.
    let z_drop_target = articara::biped::height::drop_target(
        if vx2_z_from.is_some_and(|t0| t >= t0) { vx2_z_drop } else { 0.0 },
        if t < ctl.push_z_drop_until { push_z_drop } else { 0.0 },
    );
    let (z_ref, zd_ref) = if z_drop_target > 0.0 || ctl.vx2_z_track.track > 0.0 {
        let d = ctl.vx2_z_track.step(z_drop_target, 0.05, dt);
        (z_ref - ctl.vx2_z_track.track, zd_ref - d / dt)
    } else {
        (z_ref, zd_ref)
    };
    // (MPC_REACH, the CoM height capped by the MPC from the legs' reach,
    // was not adopted and is removed.)
    let a_z = zdd_ref + kp_com * (z_ref - com.z) + kd_com * (zd_ref - com_vel.z - diag_vz_bias_height);
    let a_com = na::Vector3::new(a_xy.x, a_xy.y, a_z);
    HeightOut { z_ref, a_com }
}
