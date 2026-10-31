//! What the run measures about itself, and the report after the loop.
//!
//! `RunMetrics` is harness state: written from the plant, the walker's record
//! and the plan, read by the report, never by the controller (S3 of
//! doc/from_claude/20261029_walker_extraction_plan.md). Moved out of main.rs
//! in S15, expression for expression; the per-tick blocks became the
//! `observe_*` methods, called from the loop at the points they ran.
use crate::env_f64;
use crate::scene::{FixProbe, PushScenario};
use articara::biped::actuate::DegradedTally;
use articara::biped::gait::Support;
use articara::biped::log::MeasuredContact;
use articara::biped::rig::{BipedRig, G};
use misa_biped::walker::{self, TickRecord};

/// What is measured about the push (the scenario fires it; these are the
/// measurement's thresholds), read from the environment.
pub struct PushThresholds {
    pub recover_m: f64,
    pub recover_s: f64,
    pub window_s: f64,
    pub edge_frac: f64,
    pub transfer_s: f64,
}

impl PushThresholds {
    pub fn from_env() -> Self {
        // Recovery is declared when the DCM error comes back under this and
        // STAYS under it -- a single tick dipping below the line is the error
        // crossing zero on its way past, not a recovery.
        let recover_m = env_f64("PUSH_RECOVER_MM", 20.0) * 1e-3;
        // How long it has to stay under. The DCM time constant is 1/omega =
        // 0.179 s on this machine, so anything shorter than a couple of those is
        // indistinguishable from the error passing through the band.
        let recover_s = env_f64("PUSH_RECOVER_S", 0.5);
        // Early window. Every post-push maximum below is reported TWICE: once over
        // this window and once over the rest of the run.
        //
        // WHY: the whole-run versions measure the crash. On the 0.20 N*s run that
        // falls, the 2.77x "landing impact" peak lands 0.69 s after touchdown with
        // the trunk already at 28 deg -- that is the robot hitting the floor, not
        // the disturbance. The unloading has the same problem: the total ground
        // force first drops below half body weight at t=5.40, by which point the
        // tilt has already run to 0.122 rad. A quantity that only separates
        // survivors from fallers over the whole run may be restating "it fell".
        // Windowed to the first 1/omega or so after the push, a quantity that
        // still separates is a PRECURSOR, and only that is usable for judging a
        // change.
        let window_s = env_f64("PUSH_WINDOW_S", 0.20);
        // ---- weight-transfer quality ---------------------------------------
        //
        // What actually separated the 0.20 N*s run that falls from the 0.30 that
        // survives, tick by tick: after the support changes, the survivor's CoP
        // leaves the sole EDGE within 10 ms and settles 2-4 mm from the sole
        // centre, and the total ground force locks at 1.00x. The faller's CoP
        // stays pinned at 18-19 mm -- the edge of a 19 mm half-width sole -- for
        // the whole rest of the run, while the force oscillates between 0 and
        // 1.97x.
        //
        // A CoP on the edge is a foot that cannot modulate its own pressure. The
        // peak `cop_use` does not show this (it reads 1.00 in benign runs too);
        // what matters is whether the foot comes OFF the edge, so this measures
        // TIME AT THE EDGE, anchored on the support change rather than on the
        // clock, so the number is not contaminated by however the fall unfolds.
        let edge_frac = env_f64("PUSH_EDGE_FRAC", 0.9);
        let transfer_s = env_f64("PUSH_TRANSFER_S", 0.10);
        PushThresholds { recover_m, recover_s, window_s, edge_frac, transfer_s }
    }
}

/// The constants the measurements are taken against.
pub struct RunSetup {
    pub th: PushThresholds,
    pub total_mass: f64,
    /// The sole's half-size (fore/aft, lateral) [m].
    pub sole_half: (f64, f64),
    /// The friction the WBC assumes, and the plant's.
    pub friction_mu: f64,
    pub mu_ground: f64,
    pub dt: f64,
    /// The spawn pose (body position, yaw).
    pub p0_body: [f64; 3],
    pub yaw0_body: f64,
    /// Achievement is measured from the end of the stride ramp to the end
    /// of the last step.
    pub t_ramp_end: f64,
    pub t_walk_end: f64,
}

/// ankle_roll range of motion. The model review measured that holding the
/// sole flat already consumes 82% of the +-20 deg travel, and stepping
/// adds lateral sway on top. If this saturates, that is a MODEL limit and
/// the answer is a wider joint, not a different gain.
pub struct AnkleRoll {
    ji: [Option<usize>; 2],
    pub lim: [f64; 2],
}

impl AnkleRoll {
    /// The two joints and their limits (printed).
    pub fn new(rig: &BipedRig) -> Self {
        let ankle_roll_names = ["left_ankle_roll_joint", "right_ankle_roll_joint"];
        let ji: [Option<usize>; 2] = std::array::from_fn(|i| rig.robot.joint_map.get(ankle_roll_names[i]).copied());
        let lim: [f64; 2] = std::array::from_fn(|i| {
            ji[i]
                // The tighter side, so "82% of limit" never flatters an
                // asymmetric joint.
                .map(|ji| {
                    rig.robot.joints[ji]
                        .lower
                        .abs()
                        .min(rig.robot.joints[ji].upper.abs())
                })
                .unwrap_or(f64::INFINITY)
        });
        println!(
            "ankle_roll limit: +-{:.1} deg / +-{:.1} deg",
            lim[0].to_degrees(),
            lim[1].to_degrees()
        );
        AnkleRoll { ji, lim }
    }

    /// The plant's two ankle_roll angles (0 for a joint the robot lacks).
    pub fn read(&self, rig: &BipedRig) -> [f64; 2] {
        std::array::from_fn(|i| self.ji[i].map(|ji| rig.robot.joint_positions[ji]).unwrap_or(0.0))
    }
}

/// Per-step accounting, printed at the end. A run-total hides the thing
/// that matters, which is whether the steps are all alike -- a periodic
/// orbit -- or drifting.
pub struct StepStat {
    pub idx: usize,
    pub slice: usize,
    pub t0: f64,
    pub stance: usize,
    pub dcm_err_max: f64,
    pub cop_use_max: f64,
    pub fz_peak: f64,
    pub degraded: u32,
    pub td_vz: f64,
    pub swing_apex: f64,
}

/// What the run measures about itself: achievement, extremes, the target
/// hits, the push response and the weight transfer after it. Harness state:
/// written from the plant, the solve and the plan, read by the report after
/// the loop, and never by the controller -- except `push_at_t`, which also
/// gates the FIX probe and the push-window bookkeeping (both scenario).
pub struct RunMetrics {
    pub step_stats: Vec<StepStat>,
    pub walk_end_state: Option<([f64; 3], f64, f64)>,
    /// Pose at the end of the stride ramp (`t_ramp_end`): achievement is
    /// measured from there.
    pub ramp_end_state: Option<([f64; 3], f64, f64)>,
    /// Last state before the loop ended, so a case that FELL reports the rate
    /// it was actually holding rather than a post-fall pose divided by a
    /// duration that never happened.
    pub last_state: ([f64; 3], f64, f64),
    /// Command tracking is integrated in the BODY frame, tick by tick, rather
    /// than taken as a displacement in the start frame. Two reasons, both
    /// measured: euler yaw wraps at +-pi, so a turn past 180 deg read as
    /// -69% of its command; and a curved path's chord is not its arc, so
    /// vx+wz together read 56% on a run whose speed was right.
    pub travel: [f64; 3], // body-frame x, y, unwrapped yaw
    pub prev_pose: Option<([f64; 3], f64)>,
    pub fell: bool,
    pub min_z: f64,
    pub max_tilt: f64,
    pub max_dcm_err: f64,
    pub max_cop_use: f64,
    pub max_zmp_clamp: f64,
    pub max_ankle_roll_use: f64,
    pub n_open_loop: u32,
    /// Per-joint travel, so a joint that swings without hitting anything is
    /// still visible. Sec.30 found the v6 arm at 111 deg peak-to-peak in a clip
    /// that logged 44 self-collision ticks: the collision count is a count of
    /// collisions, and it was being read as if it were a measure of behaviour.
    pub q_lo: Vec<f64>,
    pub q_hi: Vec<f64>,
    /// Hits on `spar_target`, kept OUT of the self-collision tally: a contact
    /// with the thing the robot is aiming at is not the QP solving against a
    /// contact it has no model for, which is what that counter means.
    pub hit_ticks: u32,
    pub hit_peak_f: f64,
    pub hit_impulse: f64,
    pub hit_first_t: Option<f64>,
    pub hit_last_t: f64,
    /// Per-PUNCH, not per-run: the totals above sum every contact in the run,
    /// and "how long does one punch stay on the target" is the question Sec.42.4
    /// actually asked. An episode is a contiguous run of ticks in contact.
    pub hit_episodes: Vec<(f64, f64)>, // (duration s, impulse N*s)
    pub ep_t0: Option<f64>,
    pub ep_imp: f64,
    /// This tick's force on the target, hoisted so the CSV can carry it: a
    /// replay that draws the punch should not have to be told separately
    /// what it delivered (same reason `push_fx`/`push_fy` are logged).
    pub hit_f_now: f64,
    pub n_selfcollide: u32,
    pub max_selfcollide_f: f64,
    // ---- disturbance response ------------------------------------------
    /// Whole-run maxima say nothing about a push: they are dominated by the
    /// walk itself. Everything about the recovery has to be measured from the
    /// moment of the push forward, which is what these hold.
    pub push_at_t: Option<f64>,
    pub push_at_step: usize,
    pub push_dcm_err_max: f64,
    pub push_tilt_max: f64,
    pub push_degraded_at: u32,
    pub push_cop_max: f64,
    pub push_slip_wbc_max: f64,
    pub push_slip_plant_max: f64,
    pub push_slip_ticks: u32,
    /// Landing impact and the unloading that follows it. Measured on the
    /// straddling pair at 0.20 (fell) against 0.15 and 0.30 (both survived):
    /// the faller spiked to 2.77x body weight and then spent 100 ms with the
    /// two feet carrying less than half the robot's weight between them --
    /// effectively ballistic -- while both survivors never dropped below 0.63x
    /// and peaked at 1.22x / 1.41x. Neither friction use nor CoP use nor
    /// ankle_roll range separated those three runs; this did.
    pub push_fz_total_min: f64,
    pub push_fz_total_max: f64,
    pub push_unloaded_ticks: u32,
    /// Early-window counterparts (see `push_window_s`).
    pub win_dcm_err_max: f64,
    pub win_tilt_max: f64,
    pub win_cop_max: f64,
    pub win_slip_plant_max: f64,
    pub win_slip_ticks: u32,
    pub win_fz_total_min: f64,
    pub win_fz_total_max: f64,
    pub win_unloaded_ticks: u32,
    pub win_degraded_at: Option<u32>,
    pub win_degraded_end: u32,
    /// First support change after the push, and the CoP-edge accounting over
    /// the `transfer_s` that follows it.
    pub transfer_t: Option<f64>,
    pub support_prev: Option<Support>,
    pub xfer_edge_ticks: u32,
    pub xfer_ticks: u32,
    pub xfer_leave_t: Option<f64>,
    pub xfer_wrongfoot_ticks: u32,
    /// Second single-support entry after the push. The first-window metric is
    /// anchored to a transition that step-timing adaptation MOVES, so it scores
    /// that intervention badly by construction (+27/-1 survivals against only
    /// 36/29 on the metric). Measuring the next step too separates "the metric
    /// is truncated" from "the intervention only helps survival".
    pub transfer2_t: Option<f64>,
    pub xfer2_edge_ticks: u32,
    pub xfer2_ticks: u32,
    /// Centroidal angular momentum about the roll axis. If damping it is going
    /// to help, it has to be large and growing in runs that fall -- and the
    /// arms are shoulder_pitch + elbow, both PITCH, so they cannot make roll
    /// momentum at all. Measured before implementing the task, not after.
    pub h_roll_max: f64,
    pub h_roll_at_xfer: f64,
    /// DCM offset relative to the foot the plan is about to commit to, at the
    /// instant of commitment. The Khadiv/Englsberger nominal for a periodic
    /// walk is b_y = l_p / (1 + exp(omega * t_ss)) with the sign alternating;
    /// entering single support at the wrong sign or magnitude is the candidate
    /// mechanism for the failures traced in Sec.19-21. Zero new machinery is
    /// needed to check it -- the footstep plan and the measured DCM are both
    /// already here.
    pub xfer_dcm_off: [f64; 2],
    pub xfer_stance: i32,
    pub push_ankle_max: f64,
    pub push_exceeded: bool,
    pub push_last_exceed: Option<f64>,
    pub t_last: f64,
    /// The FIX probe fires at most once per run.
    pub fix_fired: bool,
    /// This tick is inside the early window after the push (`begin_tick`).
    pub in_window: bool,
    pub setup: RunSetup,
}

impl RunMetrics {
    /// Everything zero / empty, except the joint-travel extremes (sized by
    /// `n_joints`) and `last_state`, which starts at the spawn pose.
    pub fn new(n_joints: usize, setup: RunSetup) -> Self {
        Self {
            step_stats: Vec::new(),
            walk_end_state: None,
            ramp_end_state: None,
            last_state: (setup.p0_body, setup.yaw0_body, 0.0),
            travel: [0.0f64; 3],
            prev_pose: None,
            fell: false,
            min_z: f64::INFINITY,
            max_tilt: 0.0,
            max_dcm_err: 0.0,
            max_cop_use: 0.0,
            max_zmp_clamp: 0.0,
            max_ankle_roll_use: 0.0,
            n_open_loop: 0,
            q_lo: vec![f64::INFINITY; n_joints],
            q_hi: vec![f64::NEG_INFINITY; n_joints],
            hit_ticks: 0,
            hit_peak_f: 0.0,
            hit_impulse: 0.0,
            hit_first_t: None,
            hit_last_t: 0.0,
            hit_episodes: Vec::new(),
            ep_t0: None,
            ep_imp: 0.0,
            hit_f_now: 0.0,
            n_selfcollide: 0,
            max_selfcollide_f: 0.0,
            push_at_t: None,
            push_at_step: 0,
            push_dcm_err_max: 0.0,
            push_tilt_max: 0.0,
            push_degraded_at: 0,
            push_cop_max: 0.0,
            push_slip_wbc_max: 0.0,
            push_slip_plant_max: 0.0,
            push_slip_ticks: 0,
            push_fz_total_min: f64::INFINITY,
            push_fz_total_max: 0.0,
            push_unloaded_ticks: 0,
            win_dcm_err_max: 0.0,
            win_tilt_max: 0.0,
            win_cop_max: 0.0,
            win_slip_plant_max: 0.0,
            win_slip_ticks: 0,
            win_fz_total_min: f64::INFINITY,
            win_fz_total_max: 0.0,
            win_unloaded_ticks: 0,
            win_degraded_at: None,
            win_degraded_end: 0,
            transfer_t: None,
            support_prev: None,
            xfer_edge_ticks: 0,
            xfer_ticks: 0,
            xfer_leave_t: None,
            xfer_wrongfoot_ticks: 0,
            transfer2_t: None,
            xfer2_edge_ticks: 0,
            xfer2_ticks: 0,
            h_roll_max: 0.0,
            h_roll_at_xfer: 0.0,
            xfer_dcm_off: [0.0f64; 2],
            xfer_stance: -1,
            push_ankle_max: 0.0,
            push_exceeded: false,
            push_last_exceed: None,
            t_last: 0.0,
            fix_fired: false,
            in_window: false,
            setup,
        }
    }
}

impl RunMetrics {
    /// The tick's start: the time, and whether it is inside the early window
    /// after the push (read before the push fires later in this tick, so the
    /// window opens on the tick AFTER the impulse -- the same convention the
    /// whole-run accumulators use), with the degraded count at its ends.
    pub fn begin_tick(&mut self, t: f64, degraded_n: u32) {
        self.t_last = t;
        self.in_window = self.push_at_t.map(|tp| t - tp <= self.setup.th.window_s).unwrap_or(false);
        if self.in_window {
            if self.win_degraded_at.is_none() {
                self.win_degraded_at = Some(degraded_n);
            }
            self.win_degraded_end = degraded_n;
        }
    }

    /// After the walker's step: the DCM error, the ZMP clamp and the CoP
    /// use, and after the push its response (the transfer windows, the roll
    /// momentum `h_roll` = `(A_G v)[0]`, asked only when it is recorded, the
    /// ground force and the friction use). Returns the FIX probe's trigger on
    /// this tick, `Some(at_commitment)`; the caller fires it (it is marked
    /// fired here).
    pub fn observe_step(
        &mut self,
        t: f64,
        rec: &TickRecord,
        measured: &MeasuredContact,
        h_roll: &dyn Fn() -> f64,
        fix: &FixProbe,
    ) -> Option<bool> {
        let mut fire = None;
        // Run measurements from the record (they were taken inside the
        // control before; the values are the same).
        self.max_dcm_err = self.max_dcm_err.max(rec.dcm_err);
        // Post-push accumulation. The push fires late in the tick, after this
        // point, so the firing tick's error is still the undisturbed one and
        // the window starts one tick later -- which is what we want.
        if self.in_window {
            self.win_dcm_err_max = self.win_dcm_err_max.max(rec.dcm_err);
        }
        if self.push_at_t.is_some() {
            self.push_dcm_err_max = self.push_dcm_err_max.max(rec.dcm_err);
            // Recovery is decided BACKWARD from the end of the run, from the
            // LAST time the error was out of tolerance -- not forward from the
            // first time it dipped back in.
            //
            // Both cheaper definitions are wrong, and each was measured wrong
            // here before this comment existed. Counting the first N ticks
            // under the line credits recovery 5 ms after the shove, because
            // the impulse has not reached the CoM yet. Counting the first
            // N-tick dwell after the error exceeds the line credits recovery
            // at 5.150 s to a robot that fell at 5.915 s, because the error is
            // not monotonic on the way down and crosses the band on the way
            // past. Only "went under and STAYED under" is a recovery.
            if rec.dcm_err >= self.setup.th.recover_m {
                self.push_exceeded = true;
                self.push_last_exceed = Some(t);
            }
        }
        self.max_zmp_clamp = self.max_zmp_clamp.max(rec.clamped);
        self.max_cop_use = self.max_cop_use.max(rec.cop_use);
        if self.push_at_t.is_some() {
            self.push_cop_max = self.push_cop_max.max(rec.cop_use);
            // Friction utilisation, against BOTH denominators, because they
            // answer different questions and swapping them silently is how a
            // friction sweep comes to measure its own assumption.
            //
            //   ...wbc   -- tangential / (self.setup.friction_mu * fz): what the WBC
            //               thinks it is allowed. Sweeping `friction_mu` moves
            //               this by construction, so it cannot show whether
            //               the foot slipped.
            //   ...plant -- tangential / (self.setup.mu_ground * fz): what the ground
            //               actually offers. Above 1.0 the foot IS sliding,
            //               regardless of what the controller assumed.
            // The first support change after the push opens the transfer
            // window. `support` is the SCHEDULE's opinion; that is the right
            // anchor, because the question is whether the machine completed
            // the transfer the plan asked for.
            // The anchor is entry into SINGLE support -- the tick the plan
            // commits the whole machine to one foot. The other transition
            // (single -> double) does not discriminate: at the first one after
            // the push both the 0.20 that falls and the 0.30 that survives sit
            // on the edge for 21/21 ticks, because in double support the CoP
            // of each sole rides its inner edge by construction. It is the
            // commitment to one foot that the disturbed machine either
            // completes or does not.
            if self.transfer_t.is_none() {
                if let (Some(Support::Double), Support::Single { .. }) = (self.support_prev, rec.support) {
                    self.transfer_t = Some(t);
                    if fix.armed() && !self.fix_fired {
                        fire = Some(true);
                        self.fix_fired = true;
                    }
                    // 6-D centroidal momentum h = A_G * v; rows 0..3 are
                    // angular, and row 0 is the roll axis.
                    self.h_roll_at_xfer = h_roll();
                    if let Support::Single { stance, .. } = rec.support {
                        let u = rec.steps.xy(stance);
                        self.xfer_dcm_off = [rec.xi.x - u.x, rec.xi.y - u.y];
                        self.xfer_stance = stance as i32;
                    }
                }
            } else if self.transfer2_t.is_none() && t > self.transfer_t.unwrap() + 1e-9 {
                if let (Some(Support::Double), Support::Single { .. }) = (self.support_prev, rec.support) {
                    self.transfer2_t = Some(t);
                }
            }
            // Time-triggered corrective impulse (see `fix_at_s`).
            if fix.at_s >= 0.0
                && !self.fix_fired
                && fix.armed()
                && self.push_at_t.map(|tp0| t - tp0 >= fix.at_s).unwrap_or(false)
            {
                fire = Some(false);
                self.fix_fired = true;
            }
            if self.push_at_t.map(|tp0| t - tp0 <= 1.0).unwrap_or(false) {
                self.h_roll_max = self.h_roll_max.max(h_roll().abs());
            }
            if let Some(t2) = self.transfer2_t {
                if t - t2 <= self.setup.th.transfer_s {
                    self.xfer2_ticks += 1;
                    let on_edge2 = (0..2).any(|side| {
                        measured.cop[side][2] > 0.1 * self.setup.total_mass * G
                            && measured.cop[side][1].abs() >= self.setup.th.edge_frac * self.setup.sole_half.1
                    });
                    if on_edge2 {
                        self.xfer2_edge_ticks += 1;
                    }
                }
            }
            if let Some(t0) = self.transfer_t {
                if t - t0 <= self.setup.th.transfer_s {
                    self.xfer_ticks += 1;
                    let mut on_edge = false;
                    for side in 0..2 {
                        let fz = measured.cop[side][2];
                        if fz <= 0.1 * self.setup.total_mass * G {
                            continue;
                        }
                        if measured.cop[side][1].abs() >= self.setup.th.edge_frac * self.setup.sole_half.1 {
                            on_edge = true;
                        }
                        // A foot the schedule says is swinging, carrying load.
                        if !rec.support.is_stance(side) {
                            self.xfer_wrongfoot_ticks += 1;
                        }
                    }
                    if on_edge {
                        self.xfer_edge_ticks += 1;
                    } else if self.xfer_leave_t.is_none() {
                        self.xfer_leave_t = Some(t);
                    }
                }
            }
            let fz_total = measured.f_w[0][2] + measured.f_w[1][2];
            self.push_fz_total_min = self.push_fz_total_min.min(fz_total);
            self.push_fz_total_max = self.push_fz_total_max.max(fz_total);
            if fz_total < 0.5 * self.setup.total_mass * G {
                self.push_unloaded_ticks += 1;
            }
            if self.in_window {
                self.win_cop_max = self.win_cop_max.max(rec.cop_use);
                self.win_fz_total_min = self.win_fz_total_min.min(fz_total);
                self.win_fz_total_max = self.win_fz_total_max.max(fz_total);
                if fz_total < 0.5 * self.setup.total_mass * G {
                    self.win_unloaded_ticks += 1;
                }
            }
            for side in 0..2 {
                let fz = measured.f_w[side][2];
                if fz <= 1.0 {
                    continue; // an unloaded foot has no meaningful ratio
                }
                let tan = (measured.f_w[side][0].powi(2) + measured.f_w[side][1].powi(2)).sqrt();
                self.push_slip_wbc_max = self.push_slip_wbc_max.max(tan / (self.setup.friction_mu * fz));
                self.push_slip_plant_max = self.push_slip_plant_max.max(tan / (self.setup.mu_ground * fz));
                if self.in_window {
                    self.win_slip_plant_max = self.win_slip_plant_max.max(tan / (self.setup.mu_ground * fz));
                    if tan >= 0.98 * self.setup.mu_ground * fz {
                        self.win_slip_ticks += 1;
                    }
                }
                // AT the cone, not past it: MuJoCo's contact solver never
                // returns a force outside the cone, so `tan > mu*fz` is only
                // ever true by floating-point luck and counted 0 ticks on a
                // run that slid for 150 ms. Riding the boundary IS sliding.
                if tan >= 0.98 * self.setup.mu_ground * fz {
                    self.push_slip_ticks += 1;
                }
            }
        }
        fire
    }

    /// The plant's own contacts and joints, read every tick: the joint
    /// travel, the hits on the sparring target and the self-collisions
    /// (printing the first eight).
    pub fn observe_plant(&mut self, rig: &BipedRig, t: f64) {
        // Self-collision, watched EVERY tick rather than only at spawn.
        //
        // The spawn assert catches a robot that is braced against itself
        // before it moves; it says nothing about one that folds into itself
        // mid-step. That has already happened on this machine once -- a
        // stance-width sweep found 130 mm topples the robot because the FEET
        // touch -- and a contact the QP has no model for is indistinguishable
        // in the logs from a control failure.
        for ji in 0..rig.robot.joints.len() {
            let qj = rig.robot.joint_positions[ji];
            self.q_lo[ji] = self.q_lo[ji].min(qj);
            self.q_hi[ji] = self.q_hi[ji].max(qj);
        }
        {
            let all: Vec<(String, String, f64)> = rig
                .sim
                .contacts()
                .into_iter()
                .filter(|c| !c.body1.is_empty() && !c.body2.is_empty())
                .map(|c| (c.body1.clone(), c.body2.clone(), c.force_mag))
                .collect();
            let is_target = |a: &str, b: &str| a == "spar_target" || b == "spar_target";
            // Feet on staircase scenery are stance, not "the QP solving
            // against a contact it has no model for" -- measure_contacts now
            // counts them as ground. Anything ELSE against a stair (a shin,
            // a toe grinding a riser face) stays in the tally.
            let foot = |n: &str| n == rig.prof.foot_links[0] || n == rig.prof.foot_links[1];
            let is_stairs = |a: &str, b: &str| {
                (a.starts_with("stair") && foot(b)) || (b.starts_with("stair") && foot(a))
            };
            let f_hit: f64 = all
                .iter()
                .filter(|(a, b, _)| is_target(a, b))
                .map(|(_, _, f)| *f)
                .sum();
            self.hit_f_now = f_hit;
            if f_hit > 0.0 {
                self.hit_ticks += 1;
                self.hit_peak_f = self.hit_peak_f.max(f_hit);
                self.hit_impulse += f_hit * self.setup.dt;
                self.hit_first_t.get_or_insert(t);
                self.hit_last_t = t;
                self.ep_t0.get_or_insert(t);
                self.ep_imp += f_hit * self.setup.dt;
            } else if let Some(t0) = self.ep_t0.take() {
                self.hit_episodes.push((t - t0, self.ep_imp));
                self.ep_imp = 0.0;
            }
            let hits: Vec<(String, String, f64)> = all
                .into_iter()
                .filter(|(a, b, _)| !is_target(a, b) && !is_stairs(a, b))
                .collect();
            if !hits.is_empty() {
                self.n_selfcollide += 1;
                if self.n_selfcollide <= 8 {
                    let d: Vec<String> = hits
                        .iter()
                        .map(|(a, b, f)| format!("{a} <-> {b} ({f:.1} N)"))
                        .collect();
                    println!("  [self-collision] t={t:6.3}  {}", d.join(", "));
                }
                self.max_selfcollide_f =
                    self.max_selfcollide_f.max(hits.iter().map(|(_, _, f)| *f).fold(0.0, f64::max));
            }
        }
    }

    /// ankle_roll travel against the joint's own limit; returns the two
    /// angles (the trajectory CSV's columns).
    pub fn observe_ankle(&mut self, ankle: &AnkleRoll, rig: &BipedRig) -> [f64; 2] {
        // ankle_roll travel, against the joint's own limit.
        let ankle_roll = ankle.read(rig);
        for i in 0..2 {
            if ankle.lim[i].is_finite() && ankle.lim[i] > 1e-6 {
                let use_i = ankle_roll[i].abs() / ankle.lim[i];
                self.max_ankle_roll_use = self.max_ankle_roll_use.max(use_i);
                if self.push_at_t.is_some() {
                    self.push_ankle_max = self.push_ankle_max.max(use_i);
                }
            }
        }
        ankle_roll
    }

    /// Per-step accounting, keyed on the single-support phases (the swing
    /// foot's apex and touchdown speed from the plant).
    pub fn observe_steps(&mut self, rig: &BipedRig, t: f64, rec: &TickRecord, measured: &MeasuredContact, degraded: bool) {
        // Per-step accounting, keyed on the single-support phases.
        if matches!(rec.support, Support::Single { .. }) {
            let stance_side = match rec.support {
                Support::Single { stance, .. } => stance,
                Support::Double => unreachable!(),
            };
            if self.step_stats.last().map(|s| s.slice) != Some(rec.slice_idx) {
                self.step_stats.push(StepStat {
                    idx: rec.step_idx,
                    slice: rec.slice_idx,
                    t0: t,
                    stance: stance_side,
                    dcm_err_max: 0.0,
                    cop_use_max: 0.0,
                    fz_peak: 0.0,
                    degraded: 0,
                    td_vz: 0.0,
                    swing_apex: 0.0,
                });
            }
            let s = self.step_stats.last_mut().unwrap();
            s.dcm_err_max = s.dcm_err_max.max(rec.dcm_err);
            s.cop_use_max = s.cop_use_max.max(rec.cop_use);
            s.fz_peak = s.fz_peak.max(measured.f_w[0][2] + measured.f_w[1][2]);
            s.degraded += u32::from(degraded);
            if let Some(sw) = rec.support.swing() {
                let z = rig.sim.body_world_position(rig.prof.foot_links[sw]).unwrap()[2];
                s.swing_apex = s.swing_apex.max(z - rig.prof.sole_below_origin);
                s.td_vz = rig
                    .sim
                    .body_world_linear_velocity(rig.prof.foot_links[sw])
                    .map(|v| v[2])
                    .unwrap_or(0.0);
            }
        }
    }

    /// After the plant step: the travel in the body frame, the poses at the
    /// end of the ramp and of the walk, the lowest trunk and the tilt.
    /// Returns the trunk height and the tilt (the fall check).
    pub fn observe_pose(&mut self, rig: &BipedRig, t: f64, rec: &TickRecord) -> (f64, f64) {
        if !self.fell {
            let p = rig.sim.body_world_position(&rig.robot.root_link).unwrap();
            let y = rig
                .sim
                .body_world_orientation(&rig.robot.root_link)
                .unwrap()
                .euler_angles()
                .2;
            if let Some((pp, py)) = self.prev_pose {
                if t >= self.setup.t_ramp_end {
                    let (dx, dy) = (p[0] - pp[0], p[1] - pp[1]);
                    let (c, sn) = (py.cos(), py.sin());
                    self.travel[0] += dx * c + dy * sn;
                    self.travel[1] += -dx * sn + dy * c;
                    // Shortest-arc difference, so the accumulator never wraps.
                    let mut dy_aw = y - py;
                    while dy_aw > std::f64::consts::PI {
                        dy_aw -= std::f64::consts::TAU;
                    }
                    while dy_aw < -std::f64::consts::PI {
                        dy_aw += std::f64::consts::TAU;
                    }
                    self.travel[2] += dy_aw;
                }
            }
            self.prev_pose = Some((p, y));
            self.last_state = (p, y, t);
        }
        if self.ramp_end_state.is_none() && t >= self.setup.t_ramp_end {
            let p = rig.sim.body_world_position(&rig.robot.root_link).unwrap();
            let y = rig
                .sim
                .body_world_orientation(&rig.robot.root_link)
                .unwrap()
                .euler_angles()
                .2;
            self.ramp_end_state = Some((p, y, t));
        }
        if self.walk_end_state.is_none() && t >= self.setup.t_walk_end {
            let p = rig.sim.body_world_position(&rig.robot.root_link).unwrap();
            let y = rig
                .sim
                .body_world_orientation(&rig.robot.root_link)
                .unwrap()
                .euler_angles()
                .2;
            self.walk_end_state = Some((p, y, t));
        }
        let cur_z = rig.sim.body_world_position(&rig.robot.root_link).unwrap()[2];
        self.min_z = self.min_z.min(cur_z);
        let tilt = rec.roll.abs().max(rec.pitch.abs());
        self.max_tilt = self.max_tilt.max(tilt);
        if self.push_at_t.is_some() {
            self.push_tilt_max = self.push_tilt_max.max(tilt);
        }
        if self.in_window {
            self.win_tilt_max = self.win_tilt_max.max(tilt);
        }
        (cur_z, tilt)
    }
}

/// The report after the loop, in the order it always printed: per step, the
/// walk's result, the counters, the joint travel, the target hits, the
/// self-collisions, the push response, and the verdict.
pub fn report<S: misa_biped::wbc::WbcStack>(
    m: RunMetrics,
    walker: &walker::Walker<S>,
    tally: &DegradedTally,
    rig: &BipedRig,
    push: &PushScenario,
    spar_wall: f64,
    ref_any: bool,
) {
    let walker::WalkerStats { n_mom_cmp_ticks, mom_cmp_peak, n_push_z_drop, n_cp_shift, n_cp_retime, cp_retime_total, n_confirmed_admissions, ref_engagements, n_ds_capture, ds_capture_total, n_ds_feasible, n_ds_continuous, ds_feasible_total, n_push_replan, n_mpc_fs_cross, n_reanchor, preview_count, preview_failures, landing_retimes, landing_delay, n_replan, n_corrected, .. } = walker.stats().clone();
    let RunSetup { th: PushThresholds { recover_m: push_recover_m, recover_s: push_recover_s, window_s: push_window_s, edge_frac, transfer_s }, total_mass, sole_half: (sole_half_l, sole_half_w), friction_mu, mu_ground, dt, p0_body, yaw0_body, .. } = m.setup;
    let wcfg = walker.config();
    let gp = walker.gait_params();
    let omega = walker.omega();
    let walker::CommandSchedule { vx, vy, wz, .. } = *walker.command();
    let RunMetrics { step_stats, walk_end_state, ramp_end_state, last_state, travel, fell, min_z, max_tilt, max_dcm_err, max_cop_use, max_zmp_clamp, max_ankle_roll_use, n_open_loop, q_lo, q_hi, hit_ticks, hit_peak_f, hit_impulse, hit_first_t, hit_last_t, hit_episodes, n_selfcollide, max_selfcollide_f, push_at_t, push_at_step, push_dcm_err_max, push_tilt_max, push_degraded_at, push_cop_max, push_slip_wbc_max, push_slip_plant_max, push_slip_ticks, push_fz_total_min, push_fz_total_max, push_unloaded_ticks, win_dcm_err_max, win_tilt_max, win_cop_max, win_slip_plant_max, win_slip_ticks, win_fz_total_min, win_fz_total_max, win_unloaded_ticks, win_degraded_at, win_degraded_end, transfer_t, xfer_edge_ticks, xfer_ticks, xfer_leave_t, xfer_wrongfoot_ticks, xfer2_edge_ticks, xfer2_ticks, h_roll_max, h_roll_at_xfer, xfer_dcm_off, xfer_stance, push_ankle_max, push_exceeded, push_last_exceed, t_last, .. } = m;
    let _ = sole_half_w;

    println!("\n=== Per-step ===");
    println!("  step  t0     stance  dcm_err  cop_use  fz_peak  degraded  apex_mm  td_vz");
    for s in &step_stats {
        println!(
            "  {:4}  {:5.2}  {:>6}  {:7.4}  {:7.2}  {:7.1}  {:8}  {:7.1}  {:+.3}",
            s.idx,
            s.t0,
            if s.stance == 0 { "L" } else { "R" },
            s.dcm_err_max,
            s.cop_use_max,
            s.fz_peak,
            s.degraded,
            s.swing_apex * 1e3,
            s.td_vz
        );
    }

    println!(
        "COM REPLAN: {preview_count} solves / {preview_failures} failures; {landing_retimes} landing retimes ({landing_delay:.3}s net), {} confirmed / {} timeouts",
        walker.plan().landing().touchdowns, walker.plan().landing().timeouts
    );
    // The three-phase stair transfer was removed (never adopted, off by
    // default); the line stays so existing log parsers keep their layout.
    println!("STAIR TRANSFER: 0 started / 0 completed / 0 timeouts / 0 active");
    println!("\n=== Result (walk) ===");
    println!("  steps taken: {}", step_stats.len());
    {
        // Achieved against COMMANDED, on every axis that was commanded, and
        // measured over the walk rather than to the end of the log. A gait
        // that goes half as far as it was told looks perfectly stable while
        // doing it -- and a travel metric that only reads x says "100%" for a
        // sideways command that went nowhere, which is worse than no metric.
        let (pe, ye, te) = walk_end_state.unwrap_or(last_state);
        let (_, _, t0) = ramp_end_state.unwrap_or((p0_body, yaw0_body, gp.t_start));
        let _ = (pe, ye);
        let dur = (te - t0).max(1e-6);
        let ach = [travel[0] / dur, travel[1] / dur, travel[2] / dur];
        let cmd = [vx, vy, wz];
        let names = ["vx", "vy", "wz"];
        let units = ["m/s", "m/s", "rad/s"];
        let mut parts = Vec::new();
        for k in 0..3 {
            if cmd[k].abs() < 1e-9 && ach[k].abs() < 1e-3 {
                continue;
            }
            let pct = if cmd[k].abs() > 1e-9 {
                100.0 * ach[k] / cmd[k]
            } else {
                f64::NAN
            };
            // An uncommanded axis has no percentage -- it is cross-coupling,
            // and printing "(0%)" for it makes the pass criterion read it as a
            // failure.
            if pct.is_nan() {
                parts.push(format!("{}: {:+.4} {} (drift)", names[k], ach[k], units[k]));
            } else {
                parts.push(format!(
                    "{}: {:+.4} / {:+.4} {} ({:.0}%)",
                    names[k], ach[k], cmd[k], units[k], pct
                ));
            }
        }
        if parts.is_empty() {
            parts.push("no command".into());
        }
        println!(
            "  achieved after the ramp ({:.1} s): {}",
            dur,
            parts.join("   ")
        );
    }
    println!("  min trunk z = {min_z:.3}   max tilt = {max_tilt:.3} rad");
    println!("  max |xi - xi_ref| = {:.1} mm", max_dcm_err * 1e3);
    println!("  max CoP box use = {max_cop_use:.2}  (feet carrying >10% of weight)");
    println!("  max ZMP clamp = {:.1} mm", max_zmp_clamp * 1e3);
    println!(
        "  max ankle_roll use = {:.0}% of limit",
        max_ankle_roll_use * 100.0
    );
    println!("  degraded solves: {}", tally.n);
    tally.report();
    println!("  open-loop ticks: {n_open_loop}");
    println!(
        "  contact-set corrections: {n_corrected} ticks where the feet disagreed with the schedule"
    );
    // PHASE_BY_CONTACT is removed; the line keeps its off values.
    println!("  phase retimed by contact: 0 early / 0 late ticks, 0 ms total");
    if wcfg.contact.admit_confirmed || wcfg.contact.admit_scheduled {
        println!("  confirmed-landing admissions: {n_confirmed_admissions}");
    }
    // ADAPT_STEP / ADAPT_TIME / ADAPT_DS were removed (never adopted, off by
    // default); their summary lines stay, at the zeros they printed, so
    // existing log parsers keep their layout.
    println!("  footstep adaptations: 0 steps moved, largest 0.0 mm, {n_replan} live replans");
    if wcfg.contact_plan.on {
        println!("  contact plan: {n_cp_shift} shifts, {n_cp_retime} retimes ({:.0} ms total)", cp_retime_total * 1e3);
    }
    if wcfg.double_support.feasible {
        println!("  ds feasible: {n_ds_feasible} retimes ({:.0} ms total)", ds_feasible_total * 1e3);
        println!("  ds capture: {n_ds_capture} early ends ({:.0} ms total)", ds_capture_total * 1e3);
        // RESTEP (removed, never adopted) re-stepped nothing by default.
        println!("  re-steps: 0");
        // VX2_REANCHOR (removed) re-anchored nothing by default.
        println!("  plan re-anchors: 0 (0 mm total)");
        println!("  hip strategy (MOM_CMP): {n_mom_cmp_ticks} ticks, peak rate {mom_cmp_peak:.1} N m");
        if wcfg.double_support.feasible_continuous {
            println!("  ds continuous: {n_ds_continuous} plan writes");
        }
    }
    if wcfg.push_crouch.drop > 0.0 {
        println!("  push CoM drops: {n_push_z_drop}");
    }
    if ref_any {
        println!("  reference engagements: {ref_engagements}");
    }
    if walker.plan().landing().event_mode {
        println!(
            "  contact events: {} touchdowns by event, {} probes, {} timeouts",
            walker.plan().landing().events, walker.plan().landing().probes, walker.plan().landing().timeouts
        );
    }
    println!("  step-timing adaptations: 0 ticks retimed, 0 ms total, largest single shift 0 ms");
    println!("  reduced-step cuts: 0 ticks (unreachable -> end the step now)");
    // 20 mm: the ADAPT_TIME_WIDEN default.
    println!("  reactive widen: 0 steps (20 mm, unreachable -> land further out)");
    println!("  double-support retimes: 0 ticks, largest shift 0 ms");
    println!(
        "  step-timing solve: 0 solved / 0 declined \
         (DCM on the far side of the ZMP -- no positive time reaches the target), \
         0 clamped to the duration bounds"
    );
    {
        // Arms first and by name, because they are the ones nothing else
        // constrains; then the worst joint anywhere, so a leg that starts
        // wandering is not silently omitted by an arm-shaped filter.
        let pp = |ji: usize| (q_hi[ji] - q_lo[ji]).to_degrees();
        let mut arms: Vec<(f64, &str)> = (0..rig.robot.joints.len())
            .filter(|&ji| {
                let n = rig.robot.joints[ji].name.as_str();
                (n.contains("shoulder") || n.contains("elbow")) && q_hi[ji] > q_lo[ji]
            })
            .map(|ji| (pp(ji), rig.robot.joints[ji].name.as_str()))
            .collect();
        arms.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap());
        let worst = (0..rig.robot.joints.len())
            .filter(|&ji| q_hi[ji] > q_lo[ji])
            .map(|ji| (pp(ji), rig.robot.joints[ji].name.as_str()))
            .fold((0.0_f64, ""), |acc, x| if x.0 > acc.0 { x } else { acc });
        let arm_max = arms.first().map(|a| a.0).unwrap_or(0.0);
        println!(
            "  joint travel peak-to-peak: arms {arm_max:.1} deg ({}), worst overall \
             {:.1} deg ({})          -- an arm that swings without touching anything \
             is invisible to the self-collision count above",
            arms.first().map(|a| a.1).unwrap_or("none"),
            worst.0,
            worst.1
        );
        for (d, n) in arms.iter().take(3) {
            println!("    {n:32} {d:7.1} deg");
        }
    }
    if spar_wall > 0.0 {
        let span = hit_first_t.map(|t0| hit_last_t - t0).unwrap_or(0.0);
        println!(
            "  target hits: {hit_ticks} ticks, first contact at t={:.3}, span {:.3} s, \
             peak {hit_peak_f:.1} N, transferred impulse {hit_impulse:.4} N*s",
            hit_first_t.unwrap_or(f64::NAN),
            span
        );
        if !hit_episodes.is_empty() {
            let n = hit_episodes.len() as f64;
            let dmean = hit_episodes.iter().map(|e| e.0).sum::<f64>() / n;
            let dmax = hit_episodes.iter().map(|e| e.0).fold(0.0, f64::max);
            let imean = hit_episodes.iter().map(|e| e.1).sum::<f64>() / n;
            let imax = hit_episodes.iter().map(|e| e.1).fold(0.0, f64::max);
            println!(
                "  per punch ({} contacts): duration mean {dmean:.3} s / max {dmax:.3} s, \
                 impulse mean {imean:.3} / max {imax:.3} N*s",
                hit_episodes.len()
            );
        }
    }
    println!(
        "  self-collision ticks: {n_selfcollide}  (peak {max_selfcollide_f:.1} N)          -- any nonzero value means the QP was solving against a contact it has no model for"
    );
    if let Some(tp) = push_at_t {
        let PushScenario { imp: push_imp, deg: push_deg, dt: push_dt, at: push_at, support: push_support, .. } = &push;
        println!("\n=== Disturbance ===");
        println!(
            "  push: {push_imp:.3} N*s at {push_deg:.0} deg, {push_dt:.3} s, \
             t={tp:.3} step={push_at_step} ({push_support}, frac>={push_at:.2})"
        );
        // The ankle-strategy budget, from the CoP box this run is actually
        // using. Printed next to the impulse so a row that survived on
        // stepping alone is distinguishable from one the feet could have
        // absorbed standing still -- the two are different results.
        // How far the CoP can travel along the push direction before it leaves
        // the sole. For a rectangular box that is where the ray exits, i.e.
        // min over the axes of half_extent / |component| -- NOT the half
        // extent of whichever axis is dominant, which is only right for the
        // two axis-aligned cases and gets a 45 deg push wrong by 1.8x.
        let (ux, uy) = (push_deg.to_radians().cos(), push_deg.to_radians().sin());
        let d_cop = [
            if ux.abs() > 1e-9 {
                sole_half_l / ux.abs()
            } else {
                f64::INFINITY
            },
            if uy.abs() > 1e-9 {
                sole_half_w / uy.abs()
            } else {
                f64::INFINITY
            },
        ]
        .into_iter()
        .fold(f64::INFINITY, f64::min);
        let ankle_budget = total_mass * omega * d_cop;
        println!(
            "  ankle-strategy budget in that direction: {ankle_budget:.2} N*s \
             ({:.1}x exceeded)",
            push_imp / ankle_budget.max(1e-9)
        );
        // ---- the early window: everything below the push, before the fall --
        println!("  --- first {push_window_s:.2} s after the push (precursor window) ---");
        match transfer_t {
            Some(t0) => {
                println!(
                    "  weight transfer at t={t0:.3} ({:.3} s after the push), \
                     first {transfer_s:.2} s of it:",
                    t0 - tp
                );
                println!(
                    "    CoP on the sole edge (|y| >= {:.0}% of half-width) for \
                     {xfer_edge_ticks}/{xfer_ticks} ticks ({:.0}%)",
                    edge_frac * 100.0,
                    100.0 * xfer_edge_ticks as f64 / xfer_ticks.max(1) as f64,
                );
                match xfer_leave_t {
                    Some(tl) => println!(
                        "    left the edge {:.0} ms after the transfer",
                        (tl - t0) * 1e3
                    ),
                    None => println!(
                        "    NEVER left the edge inside the window -- the stance \
                         foot could not modulate its own pressure"
                    ),
                }
                println!(
                    "    load on a foot the schedule says is swinging: \
                     {xfer_wrongfoot_ticks} ticks"
                );
                if xfer2_ticks > 0 {
                    println!(
                        "    SECOND single support: CoP on the edge for \
                         {xfer2_edge_ticks}/{xfer2_ticks} ticks ({:.0}%)",
                        100.0 * xfer2_edge_ticks as f64 / xfer2_ticks as f64
                    );
                }
                println!(
                    "    centroidal roll momentum: {h_roll_at_xfer:+.4} kg*m^2/s at the \
                     transfer, peak |h_x| {h_roll_max:.4} in the first second"
                );
                let b_nom =
                    (walker.initial_feet().sole[0].y - walker.initial_feet().sole[1].y).abs() / (1.0 + (omega * gp.t_ss).exp());
                println!(
                    "    DCM offset from the committed stance foot: \
                     x {:+.1} mm, y {:+.1} mm (stance {}), nominal |b_y| = {:.1} mm",
                    xfer_dcm_off[0] * 1e3,
                    xfer_dcm_off[1] * 1e3,
                    if xfer_stance == 0 { "L" } else { "R" },
                    b_nom * 1e3,
                );
            }
            None => println!("  no support change after the push (run ended first)"),
        }
        println!("  win peak |xi - xi_ref| = {:.1} mm", win_dcm_err_max * 1e3);
        println!("  win peak tilt = {win_tilt_max:.3} rad");
        println!("  win peak CoP box use = {win_cop_max:.2}");
        println!(
            "  win peak friction use = {win_slip_plant_max:.2} of the plant cone, \
             sliding {win_slip_ticks} ticks ({:.0} ms)",
            win_slip_ticks as f64 * dt * 1e3
        );
        println!(
            "  win total vertical ground force: min {:.1} N ({:.2}x weight), \
             peak {win_fz_total_max:.1} N ({:.2}x weight)",
            if win_fz_total_min.is_finite() {
                win_fz_total_min
            } else {
                0.0
            },
            if win_fz_total_min.is_finite() {
                win_fz_total_min / (total_mass * G)
            } else {
                0.0
            },
            win_fz_total_max / (total_mass * G),
        );
        println!(
            "  win unloaded ticks: {win_unloaded_ticks} ({:.0} ms below half body weight)",
            win_unloaded_ticks as f64 * dt * 1e3
        );
        println!(
            "  win degraded solves: {}",
            win_degraded_end.saturating_sub(win_degraded_at.unwrap_or(win_degraded_end))
        );
        println!("  --- whole run after the push (contaminated by the fall) ---");
        println!(
            "  peak |xi - xi_ref| after the push = {:.1} mm",
            push_dcm_err_max * 1e3
        );
        println!("  peak tilt after the push = {push_tilt_max:.3} rad");
        println!(
            "  degraded solves after the push: {}",
            tally.n - push_degraded_at
        );
        // What was actually saturating when it let go. On this machine the DCM
        // error stays small right up to the limit and then the robot falls, so
        // the error is not the early warning -- these two are.
        println!("  peak CoP box use after the push = {push_cop_max:.2}");
        println!(
            "  peak friction use after the push = {push_slip_plant_max:.2} of the \
             PLANT cone (mu={mu_ground:.2}), {push_slip_wbc_max:.2} of what the WBC \
             assumed (mu={friction_mu:.2})"
        );
        println!(
            "  sliding ticks after the push: {push_slip_ticks} ({:.0} ms)  -- a loaded \
             foot riding the plant's friction cone",
            push_slip_ticks as f64 * dt * 1e3
        );
        println!(
            "  total vertical ground force after the push: min {:.1} N ({:.2}x weight), \
             peak {push_fz_total_max:.1} N ({:.2}x weight)",
            push_fz_total_min,
            push_fz_total_min / (total_mass * G),
            push_fz_total_max / (total_mass * G),
        );
        println!(
            "  unloaded ticks after the push: {push_unloaded_ticks} ({:.0} ms below half \
             body weight)  -- the machine is ballistic and the QP is planning \
             against contacts that are not carrying it",
            push_unloaded_ticks as f64 * dt * 1e3
        );
        println!(
            "  peak ankle_roll use after the push = {:.0}% of limit",
            push_ankle_max * 100.0
        );
        let settled_for = push_last_exceed
            .map(|te| t_last - te)
            .unwrap_or(f64::INFINITY);
        if fell {
            println!("  NOT RECOVERED: fell");
        } else if !push_exceeded {
            println!(
                "  ABSORBED: the DCM error never left the {:.0} mm band at all",
                push_recover_m * 1e3
            );
        } else if settled_for >= push_recover_s {
            let te = push_last_exceed.unwrap();
            println!(
                "  RECOVERED at t={te:.3} ({:.3} s, {} steps after the push) -- \
                 last out-of-band tick, then under {:.0} mm for {settled_for:.2} s \
                 to the end of the run",
                te - tp,
                walker.plan().gait().steps_taken(te.max(1e-9)).saturating_sub(push_at_step),
                push_recover_m * 1e3,
            );
        } else {
            println!(
                "  NOT RECOVERED: survived, but the DCM error was still outside \
                 the {:.0} mm band {settled_for:.2} s before the run ended \
                 (needs {push_recover_s:.2} s) -- lengthen T to decide this row",
                push_recover_m * 1e3
            );
        }
    }
    if wcfg.mpc_cross.y > 0.0 {
        println!("  mpc crossing gate: {n_mpc_fs_cross} single supports opened");
    }
    if wcfg.push_replan.trig > 0.0 {
        println!("  push re-plans: {n_push_replan}");
    }
    if wcfg.landing.reanchor > 0.0 {
        println!("  landing re-anchors: {n_reanchor}");
    }
    println!("  verdict: {}", if fell { "FELL" } else { "SURVIVED" });
}
