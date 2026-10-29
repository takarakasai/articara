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
use super::{CtlState, WalkerStats};
use articara::biped::contact::Anchors;
use articara::biped::dcm::{SupportBox, StepTiming, commanded_zmp, dcm_of, step_timing};
use articara::biped::gait::{Footsteps, Slice, Support};
use articara::biped::rig::G;
use articara::biped::terrain::Stairs;
use misa_biped::config::Config;
use misa_biped::plan::Plan;
use misa_biped::robot::{Estimate, Observation, RobotModel, RobotState};
use nalgebra as na;

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
