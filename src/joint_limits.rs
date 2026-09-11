//! Joint-limit checking: which joints sit at, or past, the bounds their model
//! declares — and the colours a viewer paints them with.
//!
//! A pose that violates its own limits is easy to produce and easy to miss.
//! Sliders clamp, so dragging one only ever *reaches* a bound, but nothing else
//! does: an IK drag, a Zenoh feed carrying angles read off a real robot, a
//! script, a simulation step, or a model whose home pose was authored outside
//! its own declaration. The result renders as a perfectly plausible robot that
//! no real one could hold.
//!
//! This module only decides *what* is out of bounds. Where that shows up —
//! tinted links in the viewport, a coloured slider, a status-bar count — is the
//! GUI's business.

use std::collections::HashMap;

use crate::robot::RobotModel;

/// How close to a bound counts as "reached". Absolute, in the joint's own units
/// (rad for revolute, m for prismatic): tight enough that only a joint actually
/// resting on its stop trips it, loose enough to survive the rounding of a
/// slider step or a round-trip through a serialized pose.
pub const AT_LIMIT_EPS: f64 = 1e-3;

/// Where a joint's angle sits relative to its declared range.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum LimitState {
    /// Resting on a bound (within [`AT_LIMIT_EPS`]). Reachable by ordinary
    /// dragging, so it's a note, not an error.
    AtLimit,
    /// Past a bound. Whatever produced this pose ignored the model's limits.
    Beyond,
}

/// One joint outside its comfort zone.
#[derive(Clone, Debug, PartialEq)]
pub struct LimitViolation {
    /// Index into `model.joints` / `model.joint_positions`.
    pub joint: usize,
    pub name: String,
    /// The link this joint drives — what a viewport highlights.
    pub link: String,
    pub state: LimitState,
    pub position: f64,
    /// The bound that was hit, and how far past it (0 when merely at it).
    pub bound: f64,
    pub overshoot: f64,
}

impl LimitViolation {
    /// One-line summary for a tooltip or status bar.
    pub fn describe(&self) -> String {
        match self.state {
            LimitState::AtLimit => {
                format!("{}: at limit ({:+.3})", self.name, self.bound)
            }
            LimitState::Beyond => format!(
                "{}: {:+.3} is {:.3} past {:+.3}",
                self.name, self.position, self.overshoot, self.bound
            ),
        }
    }
}

/// Check every joint that declares a usable range.
///
/// Joints skipped: `fixed` ones (nothing moves) and any whose `lower >= upper`,
/// which is how an unlimited or continuous joint is spelled — there is no bound
/// to violate. Returns only the joints that are at or past a bound, in joint
/// order.
pub fn check(model: &RobotModel) -> Vec<LimitViolation> {
    let mut out = Vec::new();
    for (i, joint) in model.joints.iter().enumerate() {
        if joint.joint_type == "fixed" || joint.lower >= joint.upper {
            continue;
        }
        let Some(&q) = model.joint_positions.get(i) else {
            continue;
        };
        // Pick the bound this joint is closest to failing.
        let (bound, signed_over) = if q - joint.upper >= joint.lower - q {
            (joint.upper, q - joint.upper)
        } else {
            (joint.lower, joint.lower - q)
        };
        let state = if signed_over > AT_LIMIT_EPS {
            LimitState::Beyond
        } else if signed_over >= -AT_LIMIT_EPS {
            LimitState::AtLimit
        } else {
            continue;
        };
        out.push(LimitViolation {
            joint: i,
            name: joint.name.clone(),
            link: joint.child_link.clone(),
            state,
            position: q,
            bound,
            overshoot: signed_over.max(0.0),
        });
    }
    out
}

/// Colour for a joint at a bound — amber: reachable by ordinary dragging.
pub const AT_LIMIT_COLOR: [f32; 4] = [0.95, 0.65, 0.15, 1.0];
/// Colour for a joint past a bound — red: something ignored the model.
pub const BEYOND_COLOR: [f32; 4] = [0.95, 0.20, 0.20, 1.0];

impl LimitState {
    pub fn color(self) -> [f32; 4] {
        match self {
            Self::AtLimit => AT_LIMIT_COLOR,
            Self::Beyond => BEYOND_COLOR,
        }
    }
}

/// Per-link tints for the viewport: each violated joint paints the link it
/// drives. A link driven past its bound stays red even if another pass would
/// call it merely at-limit — the worse state wins.
pub fn link_tints(violations: &[LimitViolation]) -> HashMap<String, [f32; 4]> {
    let mut tints: HashMap<String, [f32; 4]> = HashMap::new();
    for v in violations {
        let entry = tints.entry(v.link.clone()).or_insert(v.state.color());
        if v.state == LimitState::Beyond {
            *entry = BEYOND_COLOR;
        }
    }
    tints
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::PathBuf;

    /// The real Go2: 12 revolute joints with the asymmetric ranges a URDF
    /// actually declares, rather than a synthetic ±1 that hides sign mistakes.
    fn go2() -> RobotModel {
        let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("models/unitree_go2/go2.misa");
        RobotModel::from_misa(&path).expect("load go2.misa")
    }

    /// Park every joint in the middle of its own range.
    fn centred(model: &mut RobotModel) {
        for (i, j) in model.joints.iter().enumerate() {
            if j.lower < j.upper {
                model.joint_positions[i] = 0.5 * (j.lower + j.upper);
            }
        }
    }

    /// Index of a joint with a usable range, plus its bounds.
    fn a_limited_joint(model: &RobotModel) -> (usize, f64, f64) {
        let (i, j) = model
            .joints
            .iter()
            .enumerate()
            .find(|(_, j)| j.joint_type != "fixed" && j.lower < j.upper)
            .expect("go2 has limited joints");
        (i, j.lower, j.upper)
    }

    #[test]
    fn a_pose_inside_every_range_reports_nothing() {
        let mut m = go2();
        centred(&mut m);
        assert!(check(&m).is_empty(), "{:?}", check(&m));
    }

    /// Resting on a bound is reachable by dragging a slider, so it reads as a
    /// note; going past it means something ignored the model.
    #[test]
    fn a_bound_is_reached_before_it_is_exceeded() {
        let mut m = go2();
        centred(&mut m);
        let (i, lower, upper) = a_limited_joint(&m);
        let state = |m: &RobotModel| check(m).first().map(|v| v.state);

        m.joint_positions[i] = upper;
        assert_eq!(state(&m), Some(LimitState::AtLimit));
        m.joint_positions[i] = lower;
        assert_eq!(state(&m), Some(LimitState::AtLimit));
        m.joint_positions[i] = upper - AT_LIMIT_EPS / 2.0;
        assert_eq!(
            state(&m),
            Some(LimitState::AtLimit),
            "just short still counts as resting on it"
        );
        m.joint_positions[i] = upper + 0.25;
        assert_eq!(state(&m), Some(LimitState::Beyond));
        m.joint_positions[i] = lower - 0.25;
        assert_eq!(state(&m), Some(LimitState::Beyond));
    }

    #[test]
    fn the_violation_names_the_bound_it_broke() {
        let mut m = go2();
        centred(&mut m);
        let (i, lower, upper) = a_limited_joint(&m);

        m.joint_positions[i] = upper + 0.25;
        let v = check(&m).remove(0);
        assert_eq!(v.joint, i);
        assert_eq!(v.bound, upper);
        assert!((v.overshoot - 0.25).abs() < 1e-12);
        assert_eq!(v.link, m.joints[i].child_link, "the link a viewport tints");
        assert!(v.describe().contains("past"), "{}", v.describe());

        m.joint_positions[i] = lower - 0.25;
        let v = check(&m).remove(0);
        assert_eq!(v.bound, lower);
        assert!((v.overshoot - 0.25).abs() < 1e-12);
    }

    /// A joint with no usable range has no bound to violate: `lower >= upper`
    /// is how continuous / unlimited joints are spelled, and `fixed` doesn't
    /// move at all.
    #[test]
    fn joints_without_a_range_are_skipped() {
        let mut m = go2();
        centred(&mut m);
        let (i, ..) = a_limited_joint(&m);
        m.joint_positions[i] = 100.0;

        let mut unlimited = m.clone();
        unlimited.joints[i].lower = 0.0;
        unlimited.joints[i].upper = 0.0;
        assert!(check(&unlimited).is_empty(), "continuous joint");

        let mut inverted = m.clone();
        inverted.joints[i].lower = 1.0;
        inverted.joints[i].upper = -1.0;
        assert!(check(&inverted).is_empty(), "no usable range");

        let mut fixed = m.clone();
        fixed.joints[i].joint_type = "fixed".into();
        assert!(check(&fixed).is_empty(), "fixed joint");
    }

    /// The Go2's zero pose is outside its own calf range — which is exactly the
    /// kind of thing this is for, and a reminder that "all joints at 0" is not
    /// a safe default pose.
    #[test]
    fn the_go2_zero_pose_violates_its_own_calf_limits() {
        let mut m = go2();
        for q in m.joint_positions.iter_mut() {
            *q = 0.0;
        }
        let calves: Vec<_> = check(&m)
            .into_iter()
            .filter(|v| v.name.contains("calf"))
            .collect();
        assert_eq!(calves.len(), 4, "one per leg");
        assert!(calves.iter().all(|v| v.state == LimitState::Beyond));
    }

    /// Two joints on one link: the worse state decides the colour, whichever
    /// order they come in.
    #[test]
    fn the_worse_state_wins_a_shared_link() {
        let at = LimitViolation {
            joint: 0,
            name: "a".into(),
            link: "shared".into(),
            state: LimitState::AtLimit,
            position: 1.0,
            bound: 1.0,
            overshoot: 0.0,
        };
        let beyond = LimitViolation {
            state: LimitState::Beyond,
            ..at.clone()
        };
        for pair in [
            vec![at.clone(), beyond.clone()],
            vec![beyond.clone(), at.clone()],
        ] {
            assert_eq!(link_tints(&pair)["shared"], BEYOND_COLOR);
        }
        assert_eq!(link_tints(&[at])["shared"], AT_LIMIT_COLOR);
    }
}
