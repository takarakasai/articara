//! Balance reference from the divergent component of motion.
//!
//! Under the linear inverted pendulum model the CoM splits into a stable and
//! an unstable part. The unstable one,
//!
//! ```text
//! xi = c + c_dot / omega        omega = sqrt(g / z_com)
//! ```
//!
//! obeys `xi_dot = omega (xi - p)` where `p` is the ZMP: first order, and
//! directly steerable by where the pressure is put. The CoM then follows
//! `c_dot = -omega (c - xi)`, which is stable on its own. So planning the
//! ZMP and tracking the DCM is the whole balance problem, and it needs no QP
//! of its own -- the output is a commanded CoM acceleration, which is exactly
//! what the P1 task already consumes.
//!
//! # Why this replaces the latched lateral target
//!
//! Single-leg stance used a ramp to a lateral CoM target that was frozen on
//! the first tick (`LATCH_STANCE`). Freezing was correct as far as it went --
//! reading the stance foot's position every tick put the plant inside its own
//! reference and cost a fall with ZERO degraded solves. But a frozen target
//! cannot walk. The fix is not to unfreeze it; it is to generate it from the
//! FOOTSTEP PLAN, which is world-frame data that no measurement feeds back
//! into. If the foot moves, that is a disturbance to reject, not a new goal.

use nalgebra as na;

use super::gait::{FootstepPlan, Footsteps, GaitPlan, Support};

pub const G: f64 = 9.81;

/// One segment of the planned centre of pressure: linear from `p0` to `p1`
/// over `duration`. Constant segments are the `p0 == p1` case, so single and
/// double support share one closed form.
#[derive(Clone, Copy, Debug)]
pub struct ZmpSeg {
    pub p0: na::Vector2<f64>,
    pub p1: na::Vector2<f64>,
    pub t0: f64,
    pub duration: f64,
}

impl ZmpSeg {
    pub fn at(&self, t: f64) -> na::Vector2<f64> {
        let f = if self.duration <= 1e-12 {
            0.0
        } else {
            ((t - self.t0) / self.duration).clamp(0.0, 1.0)
        };
        self.p0 + (self.p1 - self.p0) * f
    }
    /// `d(p)/dt`, constant within the segment.
    pub fn slope(&self) -> na::Vector2<f64> {
        if self.duration <= 1e-12 {
            na::Vector2::zeros()
        } else {
            (self.p1 - self.p0) / self.duration
        }
    }
}

/// A planned ZMP trajectory and the DCM trajectory that lands on it.
pub struct DcmPlan {
    pub omega: f64,
    pub segs: Vec<ZmpSeg>,
    /// DCM at the END of each segment, from the backward recursion.
    ///
    /// The END, not the start, and that is not a presentation choice. See
    /// [`DcmPlan::reference`]: evaluating forward from the start needs
    /// `exp(+omega t)`, and the DCM is the DIVERGENT component, so that
    /// exponential amplifies round-off instead of decaying it.
    pub xi_eos: Vec<na::Vector2<f64>>,
}

/// Instantaneous reference: where the DCM should be, how fast, and where the
/// pressure was planned to be while it does that.
#[derive(Clone, Copy, Debug)]
pub struct DcmRef {
    pub xi: na::Vector2<f64>,
    pub xi_dot: na::Vector2<f64>,
    pub p: na::Vector2<f64>,
}

impl DcmPlan {
    /// Build the ZMP plan from the support schedule and the footstep plan.
    ///
    /// Single support puts the pressure at the stance sole centre and holds
    /// it. Double support walks it linearly from the previous stance foot to
    /// the next one -- a step in the ZMP reference would be a step in the
    /// commanded CoM acceleration, and that lands straight in the contact.
    ///
    /// `lateral_scale` pulls the single-support target back toward mid-stance:
    /// 1.0 is the sole centre, 0.0 never leaves the middle. It is not a fudge
    /// factor, it is the lateral authority the machine is being asked for --
    /// full transfer means the CoM leans out by the half stance width, and on
    /// kyo46rs that alone consumes 76% of the ankle_roll travel with both feet
    /// still flat on the floor. Below 1.0 the plan no longer reaches true
    /// single support, so it is only meaningful while both feet are down.
    pub fn from_schedule(
        plan: &GaitPlan,
        steps: &Footsteps,
        z_com: f64,
        lateral_scale: f64,
    ) -> Self {
        let fp = FootstepPlan::constant_stride(plan, steps, 0.0);
        Self::from_footsteps(plan, &fp, z_com, lateral_scale)
    }

    /// The walking form: the pressure target comes from the footstep the
    /// stance foot is standing on IN THAT SLICE, not from a fixed pair.
    pub fn from_footsteps(
        plan: &GaitPlan,
        fp: &FootstepPlan,
        z_com: f64,
        lateral_scale: f64,
    ) -> Self {
        let omega = (G / z_com).sqrt();
        // Pass 1: where the pressure sits during each SINGLE support phase,
        // and the resting point (mid-stance) for double support at the ends.
        let target = |i: usize| -> Option<na::Vector2<f64>> {
            let st = fp.at_slice(i);
            match plan.slices[i].support {
                Support::Single { stance, .. } => {
                    let mid = st.mid_xy();
                    Some(mid + (st.xy(stance) - mid) * lateral_scale)
                }
                Support::Double => None,
            }
        };
        let n = plan.slices.len();
        let mut segs: Vec<ZmpSeg> = Vec::with_capacity(n);
        for (i, s) in plan.slices.iter().enumerate() {
            let (p0, p1) = match target(i) {
                Some(p) => (p, p),
                None => {
                    // Interpolate between the neighbouring single-support
                    // points, falling back to mid-stance at the ends. In a
                    // walk those neighbours are a stride apart, so this is
                    // also what carries the body forward through double
                    // support instead of parking it.
                    let prev = (0..i).rev().find_map(target)
                        .unwrap_or_else(|| fp.at_slice(i).mid_xy());
                    let next = (i + 1..n).find_map(target)
                        .unwrap_or_else(|| fp.at_slice(i).mid_xy());
                    (prev, next)
                }
            };
            segs.push(ZmpSeg { p0, p1, t0: s.t0, duration: s.duration() });
        }

        // Pass 2: backward recursion for the DCM.
        //
        // For a linear ZMP p(t) = p0 + v t on [0, T], the solution of
        // xi_dot = omega (xi - p) is
        //     xi(t) = p(t) + v/omega + (xi_0 - p0 - v/omega) e^{omega t}
        // so, going backwards from the segment's exit value,
        //     xi_0 = p0 + v/omega + (xi_T - p1 - v/omega) e^{-omega T}
        //
        // The terminal condition is rest at the final ZMP: xi = p, c_dot = 0.
        // Anything else asks the robot to still be moving when the plan runs
        // out, and the exponential means that error is the LARGEST one at the
        // start of the plan, not the smallest.
        let mut xi_eos = vec![na::Vector2::zeros(); segs.len()];
        let mut xi_end = segs.last().map(|s| s.p1).unwrap_or_else(na::Vector2::zeros);
        for i in (0..segs.len()).rev() {
            let s = &segs[i];
            let v_over_w = s.slope() / omega;
            xi_eos[i] = xi_end;
            xi_end = s.p0 + v_over_w + (xi_end - s.p1 - v_over_w) * (-omega * s.duration).exp();
        }

        DcmPlan { omega, segs, xi_eos }
    }

    pub fn index_at(&self, t: f64) -> usize {
        match self.segs.iter().position(|s| t < s.t0 + s.duration) {
            Some(i) => i,
            None => self.segs.len() - 1,
        }
    }

    /// Reference DCM, its rate, and the planned ZMP, at time `t`.
    ///
    /// Evaluated BACKWARD from the segment's end:
    ///
    /// ```text
    /// xi(t) = p(t) + v/omega + (xi_eos - p1 - v/omega) exp(-omega (T - t))
    /// ```
    ///
    /// which is the same solution as the forward form, but with a DECAYING
    /// exponential. That is not a nicety. The forward form,
    /// `xi_ini + ... exp(+omega t)`, pairs a coefficient that the backward
    /// recursion already shrank by `exp(-omega T)` with a factor that grows by
    /// `exp(+omega T)`, so the coefficient is a catastrophic cancellation and
    /// the exponential amplifies what survives.
    ///
    /// Measured, on a 10 s final double-support segment at omega = 5.585:
    /// `exp(omega T) = 1.8e24`, the cancelled coefficient carries ~9e-18 of
    /// round-off instead of its true ~1e-26, and the reference DCM came out
    /// at -0.39 m -- ten times the stance width, for a plan whose ZMP never
    /// leaves +-0.042 m. The robot tracked it, faithfully, into the floor.
    pub fn reference(&self, t: f64) -> DcmRef {
        let i = self.index_at(t);
        let s = &self.segs[i];
        let dt = (t - s.t0).clamp(0.0, s.duration);
        let v_over_w = s.slope() / self.omega;
        let p = s.at(t);
        let decay = (-self.omega * (s.duration - dt)).exp();
        let xi = p + v_over_w + (self.xi_eos[i] - s.p1 - v_over_w) * decay;
        DcmRef { xi, xi_dot: (xi - p) * self.omega, p }
    }
}

impl DcmPlan {
    /// The plan's DCM at the END of the segment covering `t`.
    pub fn eos_at(&self, t: f64) -> na::Vector2<f64> {
        self.xi_eos[self.index_at(t)]
    }

    /// Where the MEASURED DCM will be at the end of the current segment, if
    /// the ZMP follows the plan from here.
    ///
    /// This is the quantity footstep adaptation steers on: the next foot
    /// should land under where the DCM is actually going, not under where the
    /// plan assumed it would go.
    ///
    /// The `exp(+omega dt)` here is honest -- it is a forward PREDICTION over
    /// a shrinking horizon, not an evaluation of a stored coefficient
    /// (contrast [`DcmPlan::reference`]). It is noisy at the start of a step,
    /// where dt is a full single support and the factor is ~7, and exact at
    /// touchdown, where dt is zero. Applying the correction continuously is
    /// what makes that acceptable: it converges as the step runs out.
    pub fn predict_eos(&self, t: f64, xi: &na::Vector2<f64>) -> na::Vector2<f64> {
        let i = self.index_at(t);
        let s = &self.segs[i];
        let t_end = s.t0 + s.duration;
        let dt_left = (t_end - t).max(0.0);
        let v_over_w = s.slope() / self.omega;
        s.at(t_end) + v_over_w
            + (xi - s.at(t) - v_over_w) * (self.omega * dt_left).exp()
    }
}

/// Measured DCM from the measured CoM state.
pub fn dcm_of(com: &na::Vector3<f64>, com_vel: &na::Vector3<f64>, omega: f64) -> na::Vector2<f64> {
    na::Vector2::new(com.x + com_vel.x / omega, com.y + com_vel.y / omega)
}

/// The DCM tracking law, returning the ZMP the controller wants.
///
/// Solving `omega (xi - p) = xi_dot_ref - k (xi - xi_ref)` for `p` gives
///
/// ```text
/// p_cmd = xi - xi_dot_ref/omega + (k/omega) (xi - xi_ref)
/// ```
///
/// so `k_dcm = k/omega` is dimensionless and the error decays as
/// `exp(-k_dcm * omega * t)`. `k_dcm = 2` is one DCM time constant of settling
/// per half of `1/omega`, which is the usual starting point.
pub fn commanded_zmp(
    xi_meas: &na::Vector2<f64>,
    r: &DcmRef,
    omega: f64,
    k_dcm: f64,
) -> na::Vector2<f64> {
    xi_meas - r.xi_dot / omega + (xi_meas - r.xi) * k_dcm
}

/// Commanded CoM acceleration in the horizontal plane, `omega^2 (c - p)`.
pub fn com_accel_xy(
    com: &na::Vector3<f64>,
    p_cmd: &na::Vector2<f64>,
    omega: f64,
) -> na::Vector2<f64> {
    na::Vector2::new(com.x - p_cmd.x, com.y - p_cmd.y) * (omega * omega)
}

/// Clamp the commanded ZMP into the current support polygon.
///
/// The QP's `patch_contact` already refuses to produce a CoP outside the
/// sole, so an out-of-box command is not dangerous -- it is DISHONEST. The
/// solver saturates, the CoM tracks something other than what was asked, and
/// the logs show a healthy `Optimal` status the whole way down. Clamping here
/// makes the saturation a number the summary can report instead of a silent
/// difference between the reference and the plant.
///
/// The polygon is approximated by the axis-aligned hull of the stance soles'
/// CoP boxes, which is exact while the feet are parallel and level -- the
/// only case stepping in place produces.
pub struct SupportBox {
    pub lo: na::Vector2<f64>,
    pub hi: na::Vector2<f64>,
}

impl SupportBox {
    pub fn from_stance(
        steps: &Footsteps,
        support: Support,
        cop_half: (f64, f64),
        margin: f64,
    ) -> Self {
        let mut lo = na::Vector2::new(f64::INFINITY, f64::INFINITY);
        let mut hi = na::Vector2::new(f64::NEG_INFINITY, f64::NEG_INFINITY);
        let half = na::Vector2::new(cop_half.0 * margin, cop_half.1 * margin);
        for side in 0..2 {
            if !support.is_stance(side) {
                continue;
            }
            let c = steps.xy(side);
            lo = na::Vector2::new(lo.x.min(c.x - half.x), lo.y.min(c.y - half.y));
            hi = na::Vector2::new(hi.x.max(c.x + half.x), hi.y.max(c.y + half.y));
        }
        SupportBox { lo, hi }
    }

    /// Returns the clamped point and how far it had to move (0 = inside).
    pub fn clamp(&self, p: &na::Vector2<f64>) -> (na::Vector2<f64>, f64) {
        let c = na::Vector2::new(p.x.clamp(self.lo.x, self.hi.x), p.y.clamp(self.lo.y, self.hi.y));
        (c, (c - p).norm())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::biped::gait::{GaitParams, GaitPlan, LEFT, RIGHT};

    fn steps() -> Footsteps {
        Footsteps {
            sole: [
                na::Vector3::new(0.0, 0.0706, 0.0),
                na::Vector3::new(0.0, -0.0706, 0.0),
            ],
        }
    }

    #[test]
    fn dcm_reference_is_continuous_across_segment_boundaries() {
        let p = GaitParams { t_start: 1.0, t_ds: 0.2, t_ss: 0.4, n_steps: 4, first_swing: RIGHT, t_end: 6.0 };
        let plan = DcmPlan::from_schedule(&GaitPlan::new(&p), &steps(), 0.315, 1.0);
        for s in plan.segs.iter().skip(1) {
            let before = plan.reference(s.t0 - 1e-7).xi;
            let after = plan.reference(s.t0 + 1e-7).xi;
            assert!((before - after).norm() < 1e-5, "jump at t={}: {before:?} vs {after:?}", s.t0);
        }
    }

    #[test]
    fn closed_form_satisfies_the_dcm_ode_pointwise() {
        // `xi_dot = omega (xi - p)` is the whole model, so check it as a
        // residual against a central difference of the closed form.
        //
        // NOT by forward-integrating: the DCM is the DIVERGENT component, the
        // eigenvalue is +omega, and at omega = 5.6 rad/s a 3 s march
        // multiplies any truncation error by e^16.7 ~ 2e7. An integration
        // test here fails on a correct plan, which is exactly what the first
        // version of this test did.
        let p = GaitParams { t_start: 1.0, t_ds: 0.2, t_ss: 0.4, n_steps: 3, first_swing: RIGHT, t_end: 4.0 };
        let plan = DcmPlan::from_schedule(&GaitPlan::new(&p), &steps(), 0.315, 1.0);
        let h = 1e-6;
        let mut worst: f64 = 0.0;
        let mut t = 0.05;
        while t < 3.5 {
            // Skip segment boundaries, where the central difference straddles
            // a slope change in p and is not a derivative of anything.
            let near_edge = plan
                .segs
                .iter()
                .any(|s| (t - s.t0).abs() < 1e-3 || (t - (s.t0 + s.duration)).abs() < 1e-3);
            if !near_edge {
                let fd = (plan.reference(t + h).xi - plan.reference(t - h).xi) / (2.0 * h);
                let r = plan.reference(t);
                worst = worst.max((fd - (r.xi - r.p) * plan.omega).norm());
                // The published xi_dot must agree with the same derivative.
                worst = worst.max((fd - r.xi_dot).norm());
            }
            t += 1e-3;
        }
        assert!(worst < 1e-5, "worst DCM ODE residual {worst:.3e}");
    }

    #[test]
    fn plan_ends_at_rest_over_the_final_support() {
        let p = GaitParams { t_start: 1.0, t_ds: 0.2, t_ss: 0.4, n_steps: 2, first_swing: RIGHT, t_end: 5.0 };
        let plan = DcmPlan::from_schedule(&GaitPlan::new(&p), &steps(), 0.315, 1.0);
        let end = plan.reference(5.0);
        // At rest means xi == p, i.e. c_dot == 0.
        assert!((end.xi - end.p).norm() < 1e-6);
        // ...and the final ZMP is mid-stance, not over one foot.
        assert!((end.p - steps().mid_xy()).norm() < 1e-9);
    }

    #[test]
    fn single_support_zmp_sits_on_the_stance_sole() {
        let p = GaitParams { t_start: 1.0, t_ds: 0.2, t_ss: 0.4, n_steps: 2, first_swing: RIGHT, t_end: 5.0 };
        let gait = GaitPlan::new(&p);
        let plan = DcmPlan::from_schedule(&gait, &steps(), 0.315, 1.0);
        // First SS: right swings, so the left foot carries the pressure.
        let mid_ss = 1.0 + 0.2;
        assert!(matches!(gait.support_at(mid_ss), Support::Single { stance: LEFT, .. }));
        assert!((plan.reference(mid_ss).p - steps().xy(LEFT)).norm() < 1e-12);
    }

    #[test]
    fn a_long_trailing_segment_does_not_blow_the_reference_up() {
        // The regression that cost a 22 s run. `t_end` far past the last step
        // leaves one double-support segment ten seconds long; at
        // omega = 5.585 that is exp(55.9) = 1.8e24 of forward amplification
        // on a coefficient that is pure cancellation. Bound the reference by
        // something physical: the DCM of a plan whose ZMP stays inside the
        // stance can never leave the stance by more than a few centimetres.
        let p = GaitParams { t_start: 2.0, t_ds: 0.2, t_ss: 0.45, n_steps: 20, first_swing: RIGHT, t_end: 25.0 };
        let plan = DcmPlan::from_schedule(&GaitPlan::new(&p), &steps(), 0.315, 1.0);
        let mut t = 0.0;
        while t <= 25.0 {
            let r = plan.reference(t);
            assert!(
                r.xi.norm() < 0.15,
                "reference DCM {:?} at t={t:.2} is outside anything the plan can produce",
                r.xi
            );
            t += 0.005;
        }
        // ...and it must still END at rest over the final ZMP.
        let end = plan.reference(25.0);
        assert!((end.xi - end.p).norm() < 1e-9);
    }

    #[test]
    fn support_box_shrinks_to_one_foot_in_single_support() {
        let s = steps();
        let ds = SupportBox::from_stance(&s, Support::Double, (0.049, 0.019), 1.0);
        let ss = SupportBox::from_stance(&s, Support::Single { stance: LEFT, swing: RIGHT }, (0.049, 0.019), 1.0);
        assert!((ds.hi.y - ds.lo.y) > (ss.hi.y - ss.lo.y));
        // A command over the right foot is outside the left-foot-only box.
        let (c, moved) = ss.clamp(&s.xy(RIGHT));
        assert!(moved > 0.0);
        assert!(c.y >= ss.lo.y - 1e-12);
    }
}
