//! Motion editing on top of named poses and sequences.
//!
//! A [`Sequence`] is a chain of steps (target pose, duration, curve). This
//! module gives it a time axis: a [`Trajectory`] from a start
//! configuration, keyframe editing in absolute time, feasibility checks for
//! running the motion on a real robot (joint limits, velocity and torque
//! against each joint's declared `velocity` / `effort`), and CSV export of
//! the sampled joint trajectory.
//!
//! The data stays the plain pose / sequence model, so everything edited
//! here saves to `.misa`, exports as glTF animation and replays in MuJoCo
//! unchanged.

use std::collections::{BTreeMap, HashMap};

use misarta::trajectory::{InterpolationKind, Keyframe, KeyframeAnimation, PoseTransition};

use crate::robot::{NamedPose, RobotModel, Sequence, SequenceStep};

/// Joint-space trajectory of a sequence played from a start configuration.
pub struct Trajectory {
    /// `(start time, transition)` per step, in step order.
    segments: Vec<(f64, PoseTransition<f64>)>,
    q_start: Vec<f64>,
    duration: f64,
}

impl Trajectory {
    /// Build from `seq` played from `q_start` (full joint vector in
    /// `model.joints` order). Each step moves from the previous target to
    /// its pose; joints a pose doesn't name keep their previous value.
    pub fn from_sequence(model: &RobotModel, seq: &Sequence, q_start: &[f64]) -> Result<Self, String> {
        let mut segments = Vec::with_capacity(seq.steps.len());
        let mut t = 0.0;
        let mut q = q_start.to_vec();
        for step in &seq.steps {
            let pose = model
                .poses
                .iter()
                .find(|p| p.name == step.pose_name)
                .ok_or_else(|| format!("sequence '{}': unknown pose '{}'", seq.name, step.pose_name))?;
            let q_end = pose.to_vector(model, &q);
            let duration = step.duration.max(0.0);
            segments.push((t, PoseTransition::new(q.clone(), q_end.clone(), duration, step.kind)));
            t += duration;
            q = q_end;
        }
        Ok(Self { segments, q_start: q_start.to_vec(), duration: t })
    }

    pub fn duration(&self) -> f64 {
        self.duration
    }

    /// Absolute time of each step's end (= its keyframe), in step order.
    pub fn keyframe_times(&self) -> Vec<f64> {
        self.segments.iter().map(|(t0, tr)| t0 + tr.duration).collect()
    }

    /// `(q, q̇, q̈)` at time `t`, clamped to `[0, duration]`.
    pub fn sample(&self, t: f64) -> (Vec<f64>, Vec<f64>, Vec<f64>) {
        let t = t.clamp(0.0, self.duration);
        // Last segment whose start is ≤ t (zero-length ones are skipped).
        let seg = self
            .segments
            .iter()
            .rev()
            .find(|(t0, tr)| *t0 <= t && tr.duration > 0.0)
            .or_else(|| self.segments.last());
        match seg {
            Some((t0, tr)) => {
                let local = t - t0;
                (tr.evaluate(local), tr.evaluate_velocity(local), tr.evaluate_acceleration(local))
            }
            None => {
                let zero = vec![0.0; self.q_start.len()];
                (self.q_start.clone(), zero.clone(), zero)
            }
        }
    }
}

/// Playback of `seq` for a controller (MuJoCo): holds `q_start` for
/// `settle` seconds (lets a legged robot land and settle before moving),
/// then follows the same keyframes as [`Trajectory`], shifted by `settle`.
pub fn keyframe_animation(
    model: &RobotModel,
    seq: &Sequence,
    q_start: &[f64],
    settle: f64,
) -> Result<KeyframeAnimation<f64>, String> {
    let settle = settle.max(0.0);
    let mut keys = vec![Keyframe::new(0.0, q_start.to_vec(), InterpolationKind::Linear)];
    if settle > 0.0 {
        keys.push(Keyframe::new(settle, q_start.to_vec(), InterpolationKind::Linear));
    }
    let mut t = settle;
    let mut q = q_start.to_vec();
    for step in &seq.steps {
        let pose = model
            .poses
            .iter()
            .find(|p| p.name == step.pose_name)
            .ok_or_else(|| format!("sequence '{}': unknown pose '{}'", seq.name, step.pose_name))?;
        q = pose.to_vector(model, &q);
        t += step.duration.max(0.0);
        keys.push(Keyframe::new(t, q.clone(), step.kind));
    }
    Ok(KeyframeAnimation::new(keys))
}

// ─── Keyframe editing (absolute time) ───────────────────────────────────

/// Absolute keyframe times of `seq` (cumulative step durations).
pub fn keyframe_times(seq: &Sequence) -> Vec<f64> {
    seq.steps
        .iter()
        .scan(0.0, |t, s| {
            *t += s.duration;
            Some(*t)
        })
        .collect()
}

/// Insert a keyframe at absolute time `time` holding `angles`: a new pose
/// (named `<sequence>_k<n>`, unique and a valid `.misa` identifier) is added to the model and a step
/// targeting it is spliced in, keeping every existing keyframe's absolute
/// time. Past the end the sequence is extended. Returns the step index.
pub fn insert_keyframe(
    model: &mut RobotModel,
    seq_idx: usize,
    time: f64,
    angles: BTreeMap<String, f64>,
    kind: InterpolationKind,
) -> usize {
    let seq_name = model.sequences[seq_idx].name.clone();
    let mut n = model.sequences[seq_idx].steps.len();
    let pose_name = loop {
        let (name, _) = misarta::native::sanitize_identifier(&format!("{seq_name}_k{n}"));
        if !model.poses.iter().any(|p| p.name == name) {
            break name;
        }
        n += 1;
    };
    model.poses.push(NamedPose { name: pose_name.clone(), angles, duration: 1.0, kind });

    let seq = &mut model.sequences[seq_idx];
    let times = keyframe_times(seq);
    let time = time.max(0.0);
    let at = times.iter().position(|&t| t > time).unwrap_or(times.len());
    let prev_t = if at == 0 { 0.0 } else { times[at - 1] };
    seq.steps.insert(at, SequenceStep { pose_name, duration: time - prev_t, kind });
    // The following step now starts at `time`; shorten it to keep its
    // keyframe in place.
    if let Some(next) = seq.steps.get_mut(at + 1) {
        next.duration = (times[at] - time).max(0.0);
    }
    at
}

/// Move keyframe `k` to absolute time `time`, clamped between its
/// neighbours so the order never changes; later keyframes keep their times.
/// Returns the time actually applied.
pub fn move_keyframe(seq: &mut Sequence, k: usize, time: f64) -> f64 {
    let times = keyframe_times(seq);
    if k >= times.len() {
        return 0.0;
    }
    let lo = if k == 0 { 0.0 } else { times[k - 1] };
    let hi = times.get(k + 1).copied().unwrap_or(f64::INFINITY);
    let t = time.clamp(lo, hi);
    seq.steps[k].duration = t - lo;
    if let Some(next) = seq.steps.get_mut(k + 1) {
        next.duration = hi - t;
    }
    t
}

/// Remove keyframe `k`; the next keyframe keeps its absolute time. The
/// pose itself stays in the model (other sequences may use it).
pub fn delete_keyframe(seq: &mut Sequence, k: usize) {
    if k >= seq.steps.len() {
        return;
    }
    let removed = seq.steps.remove(k);
    if let Some(next) = seq.steps.get_mut(k) {
        next.duration += removed.duration;
    }
}

// ─── Sampling, checks, export ───────────────────────────────────────────

/// Movable joints (revolute / continuous / prismatic) in `model.joints` order.
pub fn movable_joints(model: &RobotModel) -> Vec<usize> {
    model
        .joints
        .iter()
        .enumerate()
        .filter(|(_, j)| matches!(j.joint_type.as_str(), "revolute" | "continuous" | "prismatic"))
        .map(|(i, _)| i)
        .collect()
}

/// A trajectory sampled at a fixed step, per movable joint.
pub struct Series {
    pub times: Vec<f64>,
    /// Joint indices (into `model.joints`) of the columns below.
    pub joints: Vec<usize>,
    /// `q[c][k]`: position of column `c` at `times[k]`.
    pub q: Vec<Vec<f64>>,
    pub qd: Vec<Vec<f64>>,
    /// Required torque / force from fixed-base inverse dynamics
    /// `τ = M(q)·q̈ + h(q, q̇)`.
    pub tau: Vec<Vec<f64>>,
}

impl Series {
    /// Build from recorded samples (e.g. a MuJoCo trace): `q[k]`, `qd[k]`,
    /// `tau[k]` are full joint vectors in `model.joints` order at
    /// `times[k]`; only movable joints are kept.
    pub fn from_samples(
        model: &RobotModel,
        times: Vec<f64>,
        q: &[Vec<f64>],
        qd: &[Vec<f64>],
        tau: &[Vec<f64>],
    ) -> Self {
        let joints = movable_joints(model);
        let column = |rows: &[Vec<f64>], ji: usize| rows.iter().map(|r| r.get(ji).copied().unwrap_or(0.0)).collect();
        Self {
            q: joints.iter().map(|&ji| column(q, ji)).collect(),
            qd: joints.iter().map(|&ji| column(qd, ji)).collect(),
            tau: joints.iter().map(|&ji| column(tau, ji)).collect(),
            times,
            joints,
        }
    }
}

/// Largest `|q_actual − q_plan|` per movable joint over the actual
/// samples' time span: `(joint index, error, time of the error)`, worst
/// first.
pub fn tracking_error(plan: &Trajectory, actual: &Series) -> Vec<(usize, f64, f64)> {
    let mut out: Vec<(usize, f64, f64)> = actual
        .joints
        .iter()
        .enumerate()
        .map(|(c, &ji)| {
            let mut worst = (0.0, 0.0);
            for (k, &t) in actual.times.iter().enumerate() {
                let err = (actual.q[c][k] - plan.sample(t).0[ji]).abs();
                if err > worst.0 {
                    worst = (err, t);
                }
            }
            (ji, worst.0, worst.1)
        })
        .collect();
    out.sort_by(|a, b| b.1.total_cmp(&a.1));
    out
}

/// Fixed-base inverse dynamics `τ = M(q)·q̈ + h(q, q̇)` at the model's
/// current joint positions; `qd` / `qdd` are full joint vectors, `order`
/// from [`crate::rbd::dynamics::topological_joint_order`]. Returns a full
/// joint vector (0 for fixed joints).
pub fn inverse_dynamics(work: &RobotModel, order: &[usize], qd: &[f64], qdd: &[f64]) -> Vec<f64> {
    let mc = work.mc();
    let velocities: HashMap<usize, f64> = order.iter().map(|&ji| (ji, qd[ji])).collect();
    let (m, idx) = crate::rbd::dynamics::crba(work, order, mc);
    let h = crate::rbd::dynamics::rnea_bias_with_mc(work, order, &velocities, mc);
    let qdd_sub = nalgebra::DVector::from_iterator(order.len(), order.iter().map(|&ji| qdd[ji]));
    let tau = m * qdd_sub + h;
    (0..work.joints.len()).map(|ji| idx.get(ji).copied().flatten().map_or(0.0, |i| tau[i])).collect()
}

/// Sample `traj` every `dt` seconds (always including the end), with
/// mimic joints enforced and torques from inverse dynamics. The base is
/// held fixed: torques are exact for arms and for legs in the air, not
/// for legs carrying the body, whose load depends on contacts.
pub fn sample_series(model: &RobotModel, traj: &Trajectory, dt: f64) -> Series {
    let joints = movable_joints(model);
    let n = ((traj.duration() / dt.max(1e-4)).ceil() as usize).max(1);
    let times: Vec<f64> = (0..=n).map(|k| traj.duration() * k as f64 / n as f64).collect();

    let mut work = model.clone();
    if work.misarta_cache.is_none() {
        work.rebuild_misarta_model();
    }
    let order = crate::rbd::dynamics::topological_joint_order(&work);
    let cols = joints.len();
    let mut series = Series {
        times: times.clone(),
        joints: joints.clone(),
        q: vec![Vec::with_capacity(times.len()); cols],
        qd: vec![Vec::with_capacity(times.len()); cols],
        tau: vec![Vec::with_capacity(times.len()); cols],
    };
    for &t in &times {
        let (q, qd, qdd) = traj.sample(t);
        work.joint_positions = q;
        work.enforce_mimic();
        let tau = inverse_dynamics(&work, &order, &qd, &qdd);
        for (c, &ji) in joints.iter().enumerate() {
            series.q[c].push(work.joint_positions[ji]);
            series.qd[c].push(qd[ji]);
            series.tau[c].push(tau[ji]);
        }
    }
    series
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum IssueKind {
    /// Outside `[lower, upper]`.
    Position,
    /// `|q̇|` above the joint's `velocity`.
    Velocity,
    /// `|τ|` above the joint's `effort`.
    Torque,
}

impl IssueKind {
    pub fn label(self) -> &'static str {
        match self {
            IssueKind::Position => "limit",
            IssueKind::Velocity => "velocity",
            IssueKind::Torque => "torque",
        }
    }
}

/// One contiguous interval where a joint breaks a limit.
#[derive(Clone, Debug)]
pub struct Issue {
    pub joint: usize,
    pub kind: IssueKind,
    pub t_start: f64,
    pub t_end: f64,
    /// Worst value reached (position, |velocity| or |torque|).
    pub peak: f64,
    /// The bound it broke.
    pub limit: f64,
}

/// Every interval where a joint leaves its range or exceeds its declared
/// `velocity` / `effort` (0 = undeclared, not checked).
pub fn check(model: &RobotModel, series: &Series) -> Vec<Issue> {
    let mut issues = Vec::new();
    for (c, &ji) in series.joints.iter().enumerate() {
        let joint = &model.joints[ji];
        let has_range = joint.joint_type != "continuous" && joint.upper > joint.lower;
        // (kind, value per sample, how far past the bound, bound)
        let mut scan = |kind: IssueKind, over: &dyn Fn(usize) -> Option<(f64, f64)>| {
            let mut open: Option<Issue> = None;
            for k in 0..series.times.len() {
                match (over(k), open.as_mut()) {
                    (Some((value, limit)), Some(issue)) => {
                        issue.t_end = series.times[k];
                        if value.abs() > issue.peak.abs() {
                            issue.peak = value;
                        }
                        issue.limit = limit;
                    }
                    (Some((value, limit)), None) => {
                        open = Some(Issue {
                            joint: ji,
                            kind,
                            t_start: series.times[k],
                            t_end: series.times[k],
                            peak: value,
                            limit,
                        })
                    }
                    (None, _) => issues.extend(open.take()),
                }
            }
            issues.extend(open);
        };
        if has_range {
            let tol = 1e-9;
            scan(IssueKind::Position, &|k| {
                let q = series.q[c][k];
                if q < joint.lower - tol {
                    Some((q, joint.lower))
                } else if q > joint.upper + tol {
                    Some((q, joint.upper))
                } else {
                    None
                }
            });
        }
        if joint.velocity > 0.0 {
            scan(IssueKind::Velocity, &|k| {
                let v = series.qd[c][k];
                (v.abs() > joint.velocity).then_some((v, joint.velocity))
            });
        }
        if joint.effort > 0.0 {
            scan(IssueKind::Torque, &|k| {
                let tau = series.tau[c][k];
                (tau.abs() > joint.effort).then_some((tau, joint.effort))
            });
        }
    }
    issues.sort_by(|a, b| a.t_start.total_cmp(&b.t_start));
    issues
}

/// Joint trajectory as CSV sampled at `rate_hz`: `time` then one column per
/// movable joint (rad / m), header row with joint names.
pub fn trajectory_csv(model: &RobotModel, traj: &Trajectory, rate_hz: f64) -> String {
    let joints = movable_joints(model);
    let mut out = String::from("time");
    for &ji in &joints {
        out.push(',');
        out.push_str(&model.joints[ji].name);
    }
    out.push('\n');
    let dt = 1.0 / rate_hz.max(1e-3);
    let n = (traj.duration() / dt).round() as usize;
    let mut work = model.clone();
    if work.misarta_cache.is_none() {
        work.rebuild_misarta_model();
    }
    for k in 0..=n {
        let t = (k as f64 * dt).min(traj.duration());
        work.joint_positions = traj.sample(t).0;
        work.enforce_mimic();
        out.push_str(&format!("{t:.6}"));
        for &ji in &joints {
            out.push_str(&format!(",{:.6}", work.joint_positions[ji]));
        }
        out.push('\n');
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra as na;

    /// base → (revolute about Y, ±1 rad, 2 rad/s, 1 N·m) → arm with a
    /// 1 kg mass 0.5 m out along X.
    fn arm() -> RobotModel {
        let mut model = RobotModel::new_empty("arm");
        model
            .add_child(
                "base_link",
                "arm",
                "shoulder",
                "revolute",
                na::Isometry3::identity(),
                na::Vector3::y(),
                crate::robot::GeomData::Sphere { radius: 0.01 },
                [1.0; 4],
                -1.0,
                1.0,
            )
            .unwrap();
        let j = model.joint_map["shoulder"];
        model.joints[j].velocity = 2.0;
        model.joints[j].effort = 1.0;
        let li = model.link_map["arm"];
        let inertial = &mut model.links[li].inertial;
        inertial.mass = 1.0;
        inertial.origin = na::Isometry3::translation(0.5, 0.0, 0.0);
        inertial.ixx = 1e-6;
        inertial.iyy = 1e-6;
        inertial.izz = 1e-6;
        model.rebuild_misarta_model();
        model
    }

    fn pose(name: &str, q: f64) -> NamedPose {
        NamedPose {
            name: name.into(),
            angles: [("shoulder".to_string(), q)].into_iter().collect(),
            duration: 1.0,
            kind: InterpolationKind::Linear,
        }
    }

    fn seq(steps: &[(&str, f64)]) -> Sequence {
        Sequence {
            name: "s".into(),
            steps: steps
                .iter()
                .map(|(p, d)| SequenceStep { pose_name: p.to_string(), duration: *d, kind: InterpolationKind::Linear })
                .collect(),
        }
    }

    #[test]
    fn trajectory_samples_linear_steps() {
        let mut model = arm();
        model.poses = vec![pose("a", 0.5), pose("b", -0.5)];
        let s = seq(&[("a", 1.0), ("b", 2.0)]);
        let traj = Trajectory::from_sequence(&model, &s, &[0.0]).unwrap();
        assert_eq!(traj.duration(), 3.0);
        assert_eq!(traj.keyframe_times(), [1.0, 3.0]);
        assert!((traj.sample(0.5).0[0] - 0.25).abs() < 1e-12);
        assert!((traj.sample(2.0).0[0] - 0.0).abs() < 1e-12);
        assert!((traj.sample(2.0).1[0] + 0.5).abs() < 1e-12);
        assert_eq!(traj.sample(10.0).0[0], -0.5);
    }

    #[test]
    fn keyframe_edits_keep_other_times() {
        let mut model = arm();
        model.poses = vec![pose("a", 0.5), pose("b", -0.5)];
        model.sequences = vec![seq(&[("a", 1.0), ("b", 2.0)])];

        let k = insert_keyframe(&mut model, 0, 2.0, pose("x", 0.1).angles, InterpolationKind::CubicSmooth);
        assert_eq!(k, 1);
        assert_eq!(keyframe_times(&model.sequences[0]), [1.0, 2.0, 3.0]);
        assert_eq!(model.sequences[0].steps[1].pose_name, "s_k2");
        assert!(model.poses.iter().any(|p| p.name == "s_k2"));
        // Names survive a `.misa` save / load unchanged.
        assert!(model.poses.iter().all(|p| misarta::native::is_valid_identifier(&p.name)));

        // Past the end extends the sequence.
        insert_keyframe(&mut model, 0, 4.5, BTreeMap::new(), InterpolationKind::Linear);
        assert_eq!(keyframe_times(&model.sequences[0]), [1.0, 2.0, 3.0, 4.5]);

        let s = &mut model.sequences[0];
        assert_eq!(move_keyframe(s, 1, 2.5), 2.5);
        assert_eq!(keyframe_times(s), [1.0, 2.5, 3.0, 4.5]);
        // Clamped between neighbours.
        assert_eq!(move_keyframe(s, 1, 9.0), 3.0);
        assert_eq!(move_keyframe(s, 0, -1.0), 0.0);
        assert_eq!(keyframe_times(s), [0.0, 3.0, 3.0, 4.5]);

        delete_keyframe(s, 1);
        assert_eq!(keyframe_times(s), [0.0, 3.0, 4.5]);
    }

    #[test]
    fn checks_flag_limit_velocity_and_torque() {
        let mut model = arm();
        // 0 → 1.2 rad in 0.5 s: past the +1 limit, 2.4 rad/s > 2.
        model.poses = vec![pose("up", 1.2)];
        let s = seq(&[("up", 0.5)]);
        let traj = Trajectory::from_sequence(&model, &s, &[0.0]).unwrap();
        let series = sample_series(&model, &traj, 0.01);
        let issues = check(&model, &series);
        let kinds: Vec<_> = issues.iter().map(|i| i.kind).collect();
        assert!(kinds.contains(&IssueKind::Position), "{issues:?}");
        assert!(kinds.contains(&IssueKind::Velocity), "{issues:?}");
        let pos = issues.iter().find(|i| i.kind == IssueKind::Position).unwrap();
        assert!((pos.peak - 1.2).abs() < 1e-9 && pos.limit == 1.0);
        // Entered at 1/1.2 of the way: t ≈ 0.4167 s.
        assert!((pos.t_start - 0.42).abs() < 0.011, "{}", pos.t_start);

        // Gravity on 1 kg at 0.5 m needs ≈ 4.9 N·m at q = 0 (> 1 N·m).
        let torque = issues.iter().find(|i| i.kind == IssueKind::Torque).unwrap();
        assert!(torque.peak.abs() > 4.0, "{torque:?}");
        let tau0 = series.tau[0][0].abs();
        assert!((tau0 - 9.80665 * 0.5).abs() < 0.05, "{tau0}");
    }

    #[test]
    fn animation_matches_trajectory_after_settle() {
        let mut model = arm();
        model.poses = vec![pose("a", 0.5), pose("b", -0.5)];
        let mut s = seq(&[("a", 1.0), ("b", 2.0)]);
        s.steps[1].kind = InterpolationKind::QuinticSmooth;
        let traj = Trajectory::from_sequence(&model, &s, &[0.1]).unwrap();
        let anim = keyframe_animation(&model, &s, &[0.1], 0.75).unwrap();
        assert!((anim.duration() - 3.75).abs() < 1e-12);
        assert!((anim.evaluate(0.5)[0] - 0.1).abs() < 1e-12, "holds the start while settling");
        for t in [0.0, 0.3, 1.0, 1.7, 2.4, 3.0] {
            let (a, b) = (anim.evaluate(t + 0.75)[0], traj.sample(t).0[0]);
            assert!((a - b).abs() < 1e-9, "t={t}: {a} vs {b}");
        }
    }

    #[test]
    fn tracking_error_against_recorded_series() {
        let mut model = arm();
        model.poses = vec![pose("a", 1.0)];
        let traj = Trajectory::from_sequence(&model, &seq(&[("a", 1.0)]), &[0.0]).unwrap();
        // "Recorded" motion lagging the plan by 0.1 rad at the end.
        let times = vec![0.0, 0.5, 1.0];
        let q = vec![vec![0.0], vec![0.45], vec![0.9]];
        let zeros = vec![vec![0.0]; 3];
        let actual = Series::from_samples(&model, times, &q, &zeros, &zeros);
        let err = tracking_error(&traj, &actual);
        assert_eq!(err.len(), 1);
        assert!((err[0].1 - 0.1).abs() < 1e-9 && err[0].2 == 1.0, "{err:?}");
    }

    #[test]
    fn csv_has_header_and_rows() {
        let mut model = arm();
        model.poses = vec![pose("a", 0.5)];
        let traj = Trajectory::from_sequence(&model, &seq(&[("a", 1.0)]), &[0.0]).unwrap();
        let csv = trajectory_csv(&model, &traj, 10.0);
        let lines: Vec<_> = csv.lines().collect();
        assert_eq!(lines[0], "time,shoulder");
        assert_eq!(lines.len(), 12);
        assert_eq!(lines[6], "0.500000,0.250000");
        assert_eq!(lines[11], "1.000000,0.500000");
    }
}
