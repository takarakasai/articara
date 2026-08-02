//! Support schedule and swing-foot trajectory for a biped.
//!
//! The schedule is a plain list of time slices, built once, because for
//! stepping in place it IS known in advance and a plan the controller can
//! look ahead into is what the DCM reference needs ([`super::dcm`]). A
//! contact-driven correction belongs on top of this, not instead of it: the
//! nominal schedule is still what the reference was planned against.

use nalgebra as na;

/// 0 = left, 1 = right, matching [`super::rig::BipedRig::foot_mi`].
pub type Side = usize;
pub const LEFT: Side = 0;
pub const RIGHT: Side = 1;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Support {
    /// Both feet down.
    Double,
    /// One foot down. `stance` carries the load, `swing` is in the air.
    Single { stance: Side, swing: Side },
}

impl Support {
    pub fn swing(self) -> Option<Side> {
        match self {
            Support::Double => None,
            Support::Single { swing, .. } => Some(swing),
        }
    }
    pub fn is_stance(self, side: Side) -> bool {
        match self {
            Support::Double => true,
            Support::Single { stance, .. } => stance == side,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Slice {
    pub support: Support,
    pub t0: f64,
    pub t1: f64,
}

impl Slice {
    pub fn duration(&self) -> f64 {
        self.t1 - self.t0
    }
    /// Position within the slice, 0 at entry and 1 at exit.
    pub fn frac(&self, t: f64) -> f64 {
        let d = self.duration();
        if d <= 1e-12 {
            0.0
        } else {
            ((t - self.t0) / d).clamp(0.0, 1.0)
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct GaitParams {
    /// Double support before the first step. This is the weight shift, and it
    /// is deliberately longer than the inter-step double support: the CoM has
    /// to travel from the middle of the stance to over one foot, from rest.
    pub t_start: f64,
    /// Double support between steps.
    pub t_ds: f64,
    /// Single support per step.
    pub t_ss: f64,
    /// How many steps to take. The plan ends with a double-support segment
    /// that runs to `t_end`, so the robot comes back to rest centred.
    pub n_steps: usize,
    /// Which foot swings first.
    pub first_swing: Side,
    pub t_end: f64,
}

impl Default for GaitParams {
    fn default() -> Self {
        GaitParams {
            t_start: 2.0,
            t_ds: 0.20,
            t_ss: 0.45,
            n_steps: 20,
            first_swing: RIGHT,
            t_end: 25.0,
        }
    }
}

pub struct GaitPlan {
    pub slices: Vec<Slice>,
}

impl GaitPlan {
    pub fn new(p: &GaitParams) -> Self {
        let mut slices = Vec::with_capacity(2 * p.n_steps + 2);
        let mut t = 0.0;
        slices.push(Slice { support: Support::Double, t0: t, t1: t + p.t_start });
        t += p.t_start;
        let mut swing = p.first_swing;
        for _ in 0..p.n_steps {
            let stance = 1 - swing;
            slices.push(Slice {
                support: Support::Single { stance, swing },
                t0: t,
                t1: t + p.t_ss,
            });
            t += p.t_ss;
            slices.push(Slice { support: Support::Double, t0: t, t1: t + p.t_ds });
            t += p.t_ds;
            swing = 1 - swing;
        }
        // Run out to the end of the experiment on both feet, so the last
        // touchdown has somewhere to settle instead of the log simply
        // stopping mid-transient.
        let last = slices.last_mut().expect("t_start slice always exists");
        if p.t_end > last.t1 {
            last.t1 = p.t_end;
        }
        GaitPlan { slices }
    }

    /// The slice covering `t`, clamped to the ends.
    pub fn index_at(&self, t: f64) -> usize {
        match self.slices.iter().position(|s| t < s.t1) {
            Some(i) => i,
            None => self.slices.len() - 1,
        }
    }

    pub fn at(&self, t: f64) -> (&Slice, f64) {
        let i = self.index_at(t);
        (&self.slices[i], self.slices[i].frac(t))
    }

    pub fn support_at(&self, t: f64) -> Support {
        self.slices[self.index_at(t)].support
    }

    /// Number of single-support phases entered by time `t` -- the step count
    /// a summary line should report.
    pub fn steps_taken(&self, t: f64) -> usize {
        self.slices
            .iter()
            .filter(|s| matches!(s.support, Support::Single { .. }) && s.t0 < t)
            .count()
    }
}

/// Where each foot is planted, in world coordinates, at the SOLE.
///
/// For stepping in place this is captured once from the settled pose and
/// never updated. That is the point: the balance reference must be built from
/// the PLAN, not from where the feet currently measure. Reading the stance
/// foot every tick is what put the plant inside its own reference and cost a
/// fall with zero degraded solves -- the foot rolled 4.6 mm and the CoM
/// target jumped 37 mm in two ticks.
#[derive(Clone, Copy, Debug)]
pub struct Footsteps {
    pub sole: [na::Vector3<f64>; 2],
}

impl Footsteps {
    /// Sole centre of each foot in world coordinates, from an FK snapshot.
    pub fn from_fk(
        data: &misarta::data::Data<f64>,
        foot_mi: [usize; 2],
        sole_centre_x: f64,
        sole_below_origin: f64,
    ) -> Self {
        let mut sole = [na::Vector3::zeros(); 2];
        for side in 0..2 {
            let o = misarta::se3::translation(&data.oMi[foot_mi[side]]);
            let r = misarta::se3::rotation_matrix(&data.oMi[foot_mi[side]]);
            sole[side] = o + r * na::Vector3::new(sole_centre_x, 0.0, -sole_below_origin);
        }
        Footsteps { sole }
    }

    pub fn xy(&self, side: Side) -> na::Vector2<f64> {
        na::Vector2::new(self.sole[side].x, self.sole[side].y)
    }

    pub fn mid_xy(&self) -> na::Vector2<f64> {
        (self.xy(LEFT) + self.xy(RIGHT)) * 0.5
    }
}

/// Corrects the scheduled support set with what the feet are ACTUALLY doing.
///
/// The schedule is a plan, and the plant does not read it. Measured on
/// kyo46rs stepping in place, both directions of disagreement occur inside a
/// single 200 ms double-support phase:
///
/// - the swing foot lands 30 ms EARLY and carries 10.7 N while the QP still
///   has it out of the contact set, and
/// - a scheduled stance foot BOUNCES to `fz = 0` while the QP is solving for
///   it to carry 14-31 N.
///
/// Either way the QP is solving against a contact set that does not exist,
/// and the force distribution it computes is applied to a robot that cannot
/// realise it -- measured, the per-foot vertical loads came out completely
/// swapped from the plan, QP (0.0, 63.4) N against a plant doing (29.8, 30.2).
///
/// Hysteresis matters in both directions: a foot momentarily unloaded during
/// a weight transfer is normal and must not be dropped, and contact chatter on
/// touchdown must not add and remove the foot on alternate ticks.
#[derive(Clone, Debug)]
pub struct ContactCorrection {
    /// Load above which an unscheduled foot is admitted to the contact set.
    pub on_n: f64,
    /// Load below which a scheduled stance foot is dropped from it.
    pub off_n: f64,
    /// Consecutive ticks the condition must hold before acting.
    pub ticks: u32,
    on_count: [u32; 2],
    off_count: [u32; 2],
    state: [bool; 2],
}

impl ContactCorrection {
    /// Thresholds as a fraction of body weight: admit at 10%, drop at 2%.
    pub fn new(weight_n: f64, ticks: u32) -> Self {
        ContactCorrection {
            on_n: 0.10 * weight_n,
            off_n: 0.02 * weight_n,
            ticks,
            on_count: [0; 2],
            off_count: [0; 2],
            state: [true; 2],
        }
    }

    /// Feed this tick's measured vertical load per foot and the scheduled
    /// support; returns the support set to actually solve against, plus the
    /// sides whose contact was newly established (they need a fresh anchor).
    pub fn update(&mut self, nominal: Support, fz: [f64; 2]) -> (Vec<Side>, Vec<Side>) {
        let mut stance = Vec::with_capacity(2);
        let mut fresh = Vec::new();
        for side in 0..2 {
            let sched = nominal.is_stance(side);
            let loaded = fz[side] > self.on_n;
            let unloaded = fz[side] < self.off_n;
            self.on_count[side] = if loaded { self.on_count[side] + 1 } else { 0 };
            self.off_count[side] = if unloaded { self.off_count[side] + 1 } else { 0 };

            // A commanded LIFT is not negotiable. The correction exists to
            // notice contacts the plan did not predict, not to veto the plan:
            // keeping a scheduled swing foot in the contact set because it has
            // not unloaded yet means nothing ever unloads it -- the contact
            // constraint pins it and the swing task never runs. Measured, that
            // turned 20 steps into a shuffle, foot apex 0.2-1.2 mm.
            if nominal.swing() == Some(side) {
                let was = self.state[side];
                self.state[side] = false;
                let _ = was;
                continue;
            }
            let was = self.state[side];
            // Hysteresis on the MEASURED state, not on the schedule.
            //
            // Keying the branch off `sched` looks natural and is wrong: a foot
            // arrives at its scheduled double-support phase with `off_count`
            // already saturated from the swing, so "scheduled stance, keep
            // unless it has left" drops it on the first tick -- and the
            // re-admission path was on the `!sched` branch, so it could never
            // come back. Measured, that stranded the foot out of the contact
            // set for the whole rest of the run.
            let now = if was {
                !(self.off_count[side] >= self.ticks)
            } else {
                self.on_count[side] >= self.ticks
            };
            // The schedule still gets a vote at one moment: entering a phase
            // that calls a foot stance, with the foot already loaded, admits
            // it immediately rather than after the hysteresis.
            let now = now || (sched && loaded);
            self.state[side] = now;
            if now {
                stance.push(side);
                if !was {
                    fresh.push(side);
                }
            }
        }
        // Never hand the QP an empty contact set: with no contact the level-0
        // problem has no way to hold the robot up at all, and the fallback
        // would be commanding a posture PD in free flight. Keep the foot the
        // schedule believes in.
        if stance.is_empty() {
            let keep = match nominal {
                Support::Single { stance, .. } => stance,
                Support::Double => usize::from(fz[1] > fz[0]),
            };
            self.state[keep] = true;
            stance.push(keep);
        }
        (stance, fresh)
    }

    /// The side currently believed to be in contact, for logging.
    pub fn state(&self) -> [bool; 2] {
        self.state
    }
}

/// Swing-foot trajectory: smoothstep in xy, `sin^2` bump in z.
///
/// Both axes start and end with zero velocity, so the handover to stance
/// generates no impulsive torque on the leg. The vertical zero-velocity
/// property is the one that matters -- a cubic Bezier with nonzero touchdown
/// vz is what produced trunk bobbing on the quadruped.
///
/// This duplicates `quadruped_gait::swing_traj::swing_position`, which is
/// leg-count agnostic and would be reusable if it were `pub`. Twenty lines is
/// cheaper than a cross-crate change to reach it.
pub fn swing_position(
    lift_off: na::Vector3<f64>,
    touch_down: na::Vector3<f64>,
    swing_height: f64,
    frac: f64,
) -> na::Vector3<f64> {
    let t = frac.clamp(0.0, 1.0);
    // xy: smoothstep S(t) = 3t^2 - 2t^3, S'(0) = S'(1) = 0.
    let s = (3.0 - 2.0 * t) * t * t;
    let mut p = lift_off + (touch_down - lift_off) * s;
    // z bump: sin^2(pi t) h, zero derivative at both ends, peak at t = 0.5.
    p.z += (std::f64::consts::PI * t).sin().powi(2) * swing_height;
    p
}

/// Analytic derivative of [`swing_position`], in m/s, given the swing's real
/// duration. Feeding the task a velocity reference rather than damping to
/// zero is what stops the tracking error from growing with swing speed.
pub fn swing_velocity(
    lift_off: na::Vector3<f64>,
    touch_down: na::Vector3<f64>,
    swing_height: f64,
    frac: f64,
    duration_s: f64,
) -> na::Vector3<f64> {
    let t = frac.clamp(0.0, 1.0);
    let tt = duration_s.max(1e-6);
    let ds = 6.0 * t * (1.0 - t) / tt; // d/dt of smoothstep
    let mut v = (touch_down - lift_off) * ds;
    v.z += swing_height * std::f64::consts::PI * (2.0 * std::f64::consts::PI * t).sin() / tt;
    v
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn plan_alternates_and_covers_the_timeline() {
        let p = GaitParams { t_start: 1.0, t_ds: 0.2, t_ss: 0.4, n_steps: 3, first_swing: RIGHT, t_end: 10.0 };
        let plan = GaitPlan::new(&p);
        // start DS, then (SS, DS) x 3
        assert_eq!(plan.slices.len(), 7);
        assert_eq!(plan.slices[0].support, Support::Double);
        assert_eq!(plan.slices[1].support, Support::Single { stance: LEFT, swing: RIGHT });
        assert_eq!(plan.slices[3].support, Support::Single { stance: RIGHT, swing: LEFT });
        assert_eq!(plan.slices[5].support, Support::Single { stance: LEFT, swing: RIGHT });
        // contiguous, and the tail runs to t_end
        for w in plan.slices.windows(2) {
            assert!((w[0].t1 - w[1].t0).abs() < 1e-12);
        }
        assert_eq!(plan.slices.last().unwrap().t1, 10.0);
        assert_eq!(plan.steps_taken(10.0), 3);
    }

    #[test]
    fn no_steps_is_one_long_double_support() {
        let p = GaitParams { n_steps: 0, t_start: 1.0, t_end: 5.0, ..Default::default() };
        let plan = GaitPlan::new(&p);
        assert_eq!(plan.slices.len(), 1);
        assert_eq!(plan.support_at(4.0), Support::Double);
    }

    #[test]
    fn swing_endpoints_are_exact_and_boundary_velocity_is_zero() {
        let a = na::Vector3::new(0.0, 0.1, 0.0);
        let b = na::Vector3::new(0.2, 0.1, 0.0);
        assert!((swing_position(a, b, 0.04, 0.0) - a).norm() < 1e-15);
        assert!((swing_position(a, b, 0.04, 1.0) - b).norm() < 1e-15);
        // Peak clearance at mid-swing.
        assert!((swing_position(a, b, 0.04, 0.5).z - 0.04).abs() < 1e-15);
        // Touchdown vz is EXACTLY zero -- this is the property that matters.
        assert!(swing_velocity(a, b, 0.04, 1.0, 0.4).norm() < 1e-12);
        assert!(swing_velocity(a, b, 0.04, 0.0, 0.4).norm() < 1e-12);
    }
}
