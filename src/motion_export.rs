//! Hand a motion to a real controller: the joint trajectory at the
//! controller's rate, with target velocities and an optional torque
//! feedforward, plus the per-joint settings it was tuned with.
//!
//! ```text
//! # articara-hardware-trajectory v1
//! # sequence: jump
//! # rate_hz: 500
//! # motion: 1.000 .. 2.000 s (start pose held before, end pose after)
//! # feedforward: plan inverse dynamics (fixed base)
//! # units: revolute rad, rad/s, N·m; prismatic m, m/s, N
//! # joint: FL_hip_joint kp=50 kv=5 effort=1.5 velocity=33.5 lower=-0.8 upper=0.8
//! time,q:FL_hip_joint,…,qd:FL_hip_joint,…,tau_ff:FL_hip_joint,…
//! ```
//!
//! The `q:` columns use the run-CSV convention, so the file also loads as
//! a run in the motion editor (e.g. to overlay a hardware log of it).

use crate::motion::{self, Trajectory};
use crate::motion_run::RunRecord;
use crate::robot::RobotModel;

/// Where the torque feedforward column comes from.
#[derive(Clone, Copy, Debug)]
pub enum Feedforward<'a> {
    /// No feedforward column: pure position (PD) tracking.
    None,
    /// `τ = M·q̈ + h` with the base fixed — right for arms and for legs in
    /// the air, wrong for legs carrying the body.
    PlanInverseDynamics,
    /// The torques a recorded run actually applied (e.g. a MuJoCo run on
    /// the ground, contacts included), interpolated in time.
    Recorded(&'a RunRecord),
}

#[derive(Clone, Debug)]
pub struct HardwareExport {
    pub rate_hz: f64,
    /// Start pose held this long before the motion (s).
    pub hold_before: f64,
    /// End pose held this long after it (s).
    pub hold_after: f64,
    /// Include target velocities (`qd:` columns).
    pub velocity: bool,
}

impl Default for HardwareExport {
    fn default() -> Self {
        Self { rate_hz: 500.0, hold_before: 1.0, hold_after: 1.0, velocity: true }
    }
}

/// Linear interpolation of `col` over `times` at `t` (clamped to the ends).
fn interp(times: &[f64], col: &[f64], t: f64) -> f64 {
    if times.is_empty() {
        return 0.0;
    }
    let k = times.partition_point(|&x| x <= t);
    if k == 0 {
        return col[0];
    }
    if k >= times.len() {
        return col[times.len() - 1];
    }
    let (t0, t1) = (times[k - 1], times[k]);
    let u = if t1 > t0 { (t - t0) / (t1 - t0) } else { 0.0 };
    col[k - 1] + (col[k] - col[k - 1]) * u
}

/// The motion as a controller-rate CSV (see the module docs).
pub fn hardware_csv(
    model: &RobotModel,
    sequence: &str,
    traj: &Trajectory,
    opts: &HardwareExport,
    feedforward: Feedforward<'_>,
) -> String {
    let joints = motion::movable_joints(model);
    let dt = 1.0 / opts.rate_hz.max(1.0);
    let duration = traj.duration();
    let total = opts.hold_before.max(0.0) + duration + opts.hold_after.max(0.0);
    let n = (total / dt).round() as usize;

    let mut out = String::from("# articara-hardware-trajectory v1\n");
    out.push_str(&format!("# sequence: {sequence}\n"));
    out.push_str(&format!("# rate_hz: {}\n", opts.rate_hz));
    out.push_str(&format!(
        "# motion: {:.3} .. {:.3} s (start pose held before, end pose after)\n",
        opts.hold_before,
        opts.hold_before + duration
    ));
    out.push_str(&format!(
        "# feedforward: {}\n",
        match feedforward {
            Feedforward::None => "none".to_string(),
            Feedforward::PlanInverseDynamics => "plan inverse dynamics (fixed base)".to_string(),
            Feedforward::Recorded(run) => format!("recorded torque of '{}' ({})", run.name, run.source),
        }
    ));
    out.push_str("# units: revolute rad, rad/s, N·m; prismatic m, m/s, N\n");
    for &ji in &joints {
        let j = &model.joints[ji];
        out.push_str(&format!(
            "# joint: {} kp={} kv={} effort={} velocity={} lower={} upper={}\n",
            j.name, j.actuator_kp, j.actuator_kv, j.effort, j.velocity, j.lower, j.upper
        ));
    }
    let mut header = vec!["time".to_string()];
    let ff = !matches!(feedforward, Feedforward::None);
    for (prefix, on) in [("q", true), ("qd", opts.velocity), ("tau_ff", ff)] {
        if on {
            header.extend(joints.iter().map(|&ji| format!("{prefix}:{}", model.joints[ji].name)));
        }
    }
    out.push_str(&header.join(","));
    out.push('\n');

    let mut work = model.clone();
    if work.misarta_cache.is_none() {
        work.rebuild_misarta_model();
    }
    let order = crate::rbd::dynamics::topological_joint_order(&work);
    let recorded_cols: Vec<Option<usize>> = match feedforward {
        Feedforward::Recorded(run) => joints.iter().map(|&ji| run.column(&model.joints[ji].name)).collect(),
        _ => Vec::new(),
    };
    for k in 0..=n {
        let t = k as f64 * dt;
        let tp = t - opts.hold_before;
        let moving = (0.0..=duration).contains(&tp);
        let (q, mut qd, mut qdd) = traj.sample(tp);
        if !moving {
            // Holding a pose: no velocity, no acceleration.
            qd.iter_mut().for_each(|v| *v = 0.0);
            qdd.iter_mut().for_each(|v| *v = 0.0);
        }
        work.joint_positions = q;
        work.enforce_mimic();
        out.push_str(&format!("{t:.6}"));
        for &ji in &joints {
            out.push_str(&format!(",{:.6}", work.joint_positions[ji]));
        }
        if opts.velocity {
            for &ji in &joints {
                out.push_str(&format!(",{:.6}", qd[ji]));
            }
        }
        match feedforward {
            Feedforward::None => {}
            Feedforward::PlanInverseDynamics => {
                let tau = motion::inverse_dynamics(&work, &order, &qd, &qdd);
                for &ji in &joints {
                    out.push_str(&format!(",{:.6}", tau[ji]));
                }
            }
            Feedforward::Recorded(run) => {
                for col in &recorded_cols {
                    let v = match (col, &run.tau) {
                        (Some(c), Some(tau)) => interp(&run.times, &tau[*c], tp),
                        _ => 0.0,
                    };
                    out.push_str(&format!(",{v:.6}"));
                }
            }
        }
        out.push('\n');
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::robot::{NamedPose, Sequence, SequenceStep};
    use misarta::trajectory::InterpolationKind;
    use nalgebra as na;

    /// base → shoulder (revolute about Y) → 1 kg at 0.5 m along X.
    fn arm() -> RobotModel {
        let mut m = RobotModel::new_empty("arm");
        m.add_child(
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
        let li = m.link_map["arm"];
        let inertial = &mut m.links[li].inertial;
        inertial.mass = 1.0;
        inertial.origin = na::Isometry3::translation(0.5, 0.0, 0.0);
        inertial.ixx = 1e-6;
        inertial.iyy = 1e-6;
        inertial.izz = 1e-6;
        m.poses = vec![NamedPose {
            name: "up".into(),
            angles: [("shoulder".to_string(), 0.4)].into_iter().collect(),
            duration: 1.0,
            kind: InterpolationKind::Linear,
        }];
        m.sequences = vec![Sequence {
            name: "lift".into(),
            steps: vec![SequenceStep { pose_name: "up".into(), duration: 1.0, kind: InterpolationKind::Linear }],
        }];
        m.rebuild_misarta_model();
        m
    }

    fn rows(csv: &str) -> (Vec<String>, Vec<Vec<f64>>) {
        let mut lines = csv.lines().filter(|l| !l.starts_with('#'));
        let header = lines.next().unwrap().split(',').map(str::to_string).collect();
        let rows = lines.map(|l| l.split(',').map(|c| c.parse().unwrap()).collect()).collect();
        (header, rows)
    }

    #[test]
    fn holds_motion_and_gravity_feedforward() {
        let m = arm();
        let traj = Trajectory::from_sequence(&m, &m.sequences[0], &[0.0]).unwrap();
        let opts = HardwareExport { rate_hz: 10.0, hold_before: 0.5, hold_after: 0.5, velocity: true };
        let csv = hardware_csv(&m, "lift", &traj, &opts, Feedforward::PlanInverseDynamics);
        assert!(csv.contains("# motion: 0.500 .. 1.500 s"));
        assert!(csv.contains("# joint: shoulder kp=50 kv=5"));
        let (header, rows) = rows(&csv);
        assert_eq!(header, ["time", "q:shoulder", "qd:shoulder", "tau_ff:shoulder"]);
        assert_eq!(rows.len(), 21); // 0 .. 2.0 s at 10 Hz
        // Hold before: start pose, no velocity, gravity-holding torque
        // (1 kg at 0.5 m ≈ 4.9 N·m).
        assert_eq!((rows[2][1], rows[2][2]), (0.0, 0.0));
        assert!((rows[2][3].abs() - 9.80665 * 0.5).abs() < 0.05, "{}", rows[2][3]);
        // Mid motion: halfway, 0.4 rad/s.
        assert!((rows[10][1] - 0.2).abs() < 1e-6 && (rows[10][2] - 0.4).abs() < 1e-6);
        // Hold after: end pose, velocity back to 0.
        assert!((rows[20][1] - 0.4).abs() < 1e-6 && rows[20][2] == 0.0);
    }

    #[test]
    fn recorded_feedforward_is_interpolated_and_file_loads_as_run() {
        let m = arm();
        let traj = Trajectory::from_sequence(&m, &m.sequences[0], &[0.0]).unwrap();
        let run = RunRecord::from_rows(
            &m,
            "sim",
            "lift",
            "MuJoCo, on ground",
            vec![0.0, 1.0],
            &[vec![0.0], vec![0.4]],
            None,
            Some(&[vec![1.0], vec![3.0]]),
            None,
            None,
            None,
        );
        let opts = HardwareExport { rate_hz: 4.0, hold_before: 0.0, hold_after: 0.0, velocity: false };
        let csv = hardware_csv(&m, "lift", &traj, &opts, Feedforward::Recorded(&run));
        assert!(csv.contains("# feedforward: recorded torque of 'sim' (MuJoCo, on ground)"));
        let (header, rows) = rows(&csv);
        assert_eq!(header, ["time", "q:shoulder", "tau_ff:shoulder"]);
        let tau: Vec<f64> = rows.iter().map(|r| r[2]).collect();
        assert_eq!(tau, [1.0, 1.5, 2.0, 2.5, 3.0]);

        // The export reads back as a run (q only; tau_ff is skipped).
        let back = RunRecord::from_csv(&csv).unwrap();
        assert_eq!(back.joints, ["shoulder"]);
        assert_eq!(back.q[0], [0.0, 0.1, 0.2, 0.3, 0.4]);
        assert!(back.tau.is_none());
    }
}
