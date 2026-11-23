//! Recorded runs of a motion, kept to compare against the plan and each
//! other.
//!
//! A [`RunRecord`] is what one execution of a sequence produced: joint
//! positions (and, when known, velocities, torques, base height and tilt)
//! over plan time, plus a snapshot of the plan that was run. Columns are
//! keyed by joint name, so a record survives model edits and can come from
//! anywhere that writes the same CSV — a MuJoCo run today, a hardware log
//! later:
//!
//! ```text
//! # articara-run v1
//! # name: run 3
//! # sequence: jump
//! # source: MuJoCo, on ground, settle 1.00 s
//! # plan: {"steps":[…],"poses":[…],"start":{…}}
//! time,q:FL_hip,…,qd:FL_hip,…,tau:FL_hip,…,base_z,tilt
//! ```
//!
//! Only `time` and the `q:` columns are required; a bare joint name is
//! read as its position, so a minimal hardware log is just
//! `time,<joint>,<joint>,…`.

use std::collections::BTreeMap;

use serde_json::{Value, json};

use crate::motion::{self, Issue, IssueKind, Series, Trajectory};
use crate::robot::{NamedPose, RobotModel, Sequence, SequenceStep};

/// The plan a run executed: enough to restore it into the model.
#[derive(Clone, Debug)]
pub struct PlanSnapshot {
    pub sequence: Sequence,
    /// The poses the sequence's steps reference.
    pub poses: Vec<NamedPose>,
    /// Start configuration by joint name.
    pub start: BTreeMap<String, f64>,
}

impl PlanSnapshot {
    /// Snapshot sequence `seq` of `model` started from `start_q`.
    pub fn capture(model: &RobotModel, seq: &Sequence, start_q: &[f64]) -> Self {
        let poses = model
            .poses
            .iter()
            .filter(|p| seq.steps.iter().any(|s| s.pose_name == p.name))
            .cloned()
            .collect();
        let start = model
            .joints
            .iter()
            .zip(start_q)
            .filter(|(j, _)| j.joint_type != "fixed")
            .map(|(j, q)| (j.name.clone(), *q))
            .collect();
        Self { sequence: seq.clone(), poses, start }
    }

    /// Put this plan back: the sequence of the same name is replaced (or
    /// added) and its poses overwritten (or added). Returns the sequence
    /// index and the start configuration as a full joint vector.
    pub fn restore(&self, model: &mut RobotModel) -> (usize, Vec<f64>) {
        for pose in &self.poses {
            match model.poses.iter_mut().find(|p| p.name == pose.name) {
                Some(p) => *p = pose.clone(),
                None => model.poses.push(pose.clone()),
            }
        }
        let idx = match model.sequences.iter().position(|s| s.name == self.sequence.name) {
            Some(i) => {
                model.sequences[i] = self.sequence.clone();
                i
            }
            None => {
                model.sequences.push(self.sequence.clone());
                model.sequences.len() - 1
            }
        };
        let start = model
            .joints
            .iter()
            .zip(&model.joint_positions)
            .map(|(j, q)| self.start.get(&j.name).copied().unwrap_or(*q))
            .collect();
        (idx, start)
    }

    /// The planned trajectory this snapshot describes, on `model`'s
    /// kinematics (the model itself is left untouched).
    pub fn trajectory(&self, model: &RobotModel) -> Result<Trajectory, String> {
        let mut scratch = model.clone();
        let (idx, start) = self.restore(&mut scratch);
        Trajectory::from_sequence(&scratch, &scratch.sequences[idx], &start)
    }

    fn to_json(&self) -> Value {
        json!({
            "sequence": self.sequence.name,
            "steps": self.sequence.steps.iter().map(|s| json!({
                "pose": s.pose_name, "duration": s.duration, "kind": s.kind,
            })).collect::<Vec<_>>(),
            "poses": self.poses.iter().map(|p| json!({
                "name": p.name, "angles": p.angles, "duration": p.duration, "kind": p.kind,
            })).collect::<Vec<_>>(),
            "start": self.start,
        })
    }

    fn from_json(v: &Value) -> Option<Self> {
        let kind = |v: &Value| serde_json::from_value(v.clone()).ok();
        let steps = v["steps"]
            .as_array()?
            .iter()
            .map(|s| {
                Some(SequenceStep {
                    pose_name: s["pose"].as_str()?.to_string(),
                    duration: s["duration"].as_f64()?,
                    kind: kind(&s["kind"])?,
                })
            })
            .collect::<Option<Vec<_>>>()?;
        let poses = v["poses"]
            .as_array()?
            .iter()
            .map(|p| {
                Some(NamedPose {
                    name: p["name"].as_str()?.to_string(),
                    angles: serde_json::from_value(p["angles"].clone()).ok()?,
                    duration: p["duration"].as_f64()?,
                    kind: kind(&p["kind"])?,
                })
            })
            .collect::<Option<Vec<_>>>()?;
        Some(Self {
            sequence: Sequence { name: v["sequence"].as_str()?.to_string(), steps },
            poses,
            start: serde_json::from_value(v["start"].clone()).ok()?,
        })
    }
}

/// One execution of a sequence. Signals are `[column][sample]`, columns
/// aligned with `joints`.
#[derive(Clone, Debug)]
pub struct RunRecord {
    pub name: String,
    pub sequence: String,
    /// Where and how it ran, e.g. "MuJoCo, on ground, settle 1.00 s".
    pub source: String,
    pub joints: Vec<String>,
    /// Plan time (s): 0 = the sequence starts.
    pub times: Vec<f64>,
    pub q: Vec<Vec<f64>>,
    pub qd: Option<Vec<Vec<f64>>>,
    pub tau: Option<Vec<Vec<f64>>>,
    /// Root link height (m).
    pub base_z: Option<Vec<f64>>,
    /// Angle between the root link's z axis and world up (rad).
    pub tilt: Option<Vec<f64>>,
    pub plan: Option<PlanSnapshot>,
}

/// Rows of full joint vectors (`model.joints` order) → movable-joint columns.
fn columns(model: &RobotModel, rows: &[Vec<f64>]) -> Vec<Vec<f64>> {
    motion::movable_joints(model)
        .into_iter()
        .map(|ji| rows.iter().map(|r| r.get(ji).copied().unwrap_or(f64::NAN)).collect())
        .collect()
}

impl RunRecord {
    /// Build from per-sample full joint vectors (e.g. a MuJoCo trace).
    #[allow(clippy::too_many_arguments)]
    pub fn from_rows(
        model: &RobotModel,
        name: impl Into<String>,
        sequence: impl Into<String>,
        source: impl Into<String>,
        times: Vec<f64>,
        q: &[Vec<f64>],
        qd: Option<&[Vec<f64>]>,
        tau: Option<&[Vec<f64>]>,
        base_z: Option<Vec<f64>>,
        tilt: Option<Vec<f64>>,
        plan: Option<PlanSnapshot>,
    ) -> Self {
        Self {
            name: name.into(),
            sequence: sequence.into(),
            source: source.into(),
            joints: motion::movable_joints(model).into_iter().map(|ji| model.joints[ji].name.clone()).collect(),
            times,
            q: columns(model, q),
            qd: qd.map(|r| columns(model, r)),
            tau: tau.map(|r| columns(model, r)),
            base_z,
            tilt,
            plan,
        }
    }

    /// Column of joint `name`, if recorded.
    pub fn column(&self, name: &str) -> Option<usize> {
        self.joints.iter().position(|j| j == name)
    }

    /// The record as a [`Series`] over `model`'s movable joints. Joints it
    /// didn't record read NaN; missing velocity / torque read 0 (so they
    /// raise no issues).
    pub fn series(&self, model: &RobotModel) -> Series {
        let n = self.times.len();
        let joints = motion::movable_joints(model);
        let pick = |sig: Option<&Vec<Vec<f64>>>, ji: usize, missing: f64| -> Vec<f64> {
            match (sig, self.column(&model.joints[ji].name)) {
                (Some(cols), Some(c)) => cols[c].clone(),
                _ => vec![missing; n],
            }
        };
        Series {
            times: self.times.clone(),
            q: joints.iter().map(|&ji| pick(Some(&self.q), ji, f64::NAN)).collect(),
            qd: joints.iter().map(|&ji| pick(self.qd.as_ref(), ji, 0.0)).collect(),
            tau: joints.iter().map(|&ji| pick(self.tau.as_ref(), ji, 0.0)).collect(),
            joints,
        }
    }

    pub fn to_csv(&self) -> String {
        let mut out = String::from("# articara-run v1\n");
        for (k, v) in [("name", &self.name), ("sequence", &self.sequence), ("source", &self.source)] {
            out.push_str(&format!("# {k}: {}\n", v.replace('\n', " ")));
        }
        if let Some(plan) = &self.plan {
            out.push_str(&format!("# plan: {}\n", plan.to_json()));
        }
        let mut header = vec!["time".to_string()];
        let mut cols: Vec<&Vec<f64>> = Vec::new();
        for (prefix, sig) in [("q", Some(&self.q)), ("qd", self.qd.as_ref()), ("tau", self.tau.as_ref())] {
            if let Some(sig) = sig {
                for (name, col) in self.joints.iter().zip(sig) {
                    header.push(format!("{prefix}:{name}"));
                    cols.push(col);
                }
            }
        }
        for (name, sig) in [("base_z", self.base_z.as_ref()), ("tilt", self.tilt.as_ref())] {
            if let Some(sig) = sig {
                header.push(name.to_string());
                cols.push(sig);
            }
        }
        out.push_str(&header.join(","));
        out.push('\n');
        for (k, t) in self.times.iter().enumerate() {
            out.push_str(&format!("{t:.6}"));
            for col in &cols {
                out.push_str(&format!(",{}", col[k]));
            }
            out.push('\n');
        }
        out
    }

    pub fn from_csv(text: &str) -> Result<Self, String> {
        let mut meta: BTreeMap<String, String> = BTreeMap::new();
        let mut lines = text.lines().filter(|l| !l.trim().is_empty());
        let header = loop {
            let line = lines.next().ok_or("run CSV: no header row")?;
            match line.strip_prefix('#') {
                Some(m) => {
                    if let Some((k, v)) = m.split_once(':') {
                        meta.insert(k.trim().to_string(), v.trim().to_string());
                    }
                }
                None => break line,
            }
        };
        let names: Vec<&str> = header.split(',').map(str::trim).collect();
        let time_col = names.iter().position(|n| *n == "time").ok_or("run CSV: no 'time' column")?;

        enum Col {
            Q(usize),
            Qd(usize),
            Tau(usize),
            BaseZ,
            Tilt,
            Skip,
        }
        let mut joints: Vec<String> = Vec::new();
        let mut joint_of = |name: &str| match joints.iter().position(|j| j == name) {
            Some(i) => i,
            None => {
                joints.push(name.to_string());
                joints.len() - 1
            }
        };
        let kinds: Vec<Col> = names
            .iter()
            .enumerate()
            .map(|(i, n)| {
                if i == time_col {
                    Col::Skip
                } else if let Some(j) = n.strip_prefix("q:") {
                    Col::Q(joint_of(j))
                } else if let Some(j) = n.strip_prefix("qd:") {
                    Col::Qd(joint_of(j))
                } else if let Some(j) = n.strip_prefix("tau:") {
                    Col::Tau(joint_of(j))
                } else if *n == "base_z" {
                    Col::BaseZ
                } else if *n == "tilt" {
                    Col::Tilt
                } else {
                    // Bare joint name: a position column.
                    Col::Q(joint_of(n))
                }
            })
            .collect();
        let nj = joints.len();
        let has = |f: &dyn Fn(&Col) -> bool| kinds.iter().any(f);
        let (has_qd, has_tau) = (has(&|c| matches!(c, Col::Qd(_))), has(&|c| matches!(c, Col::Tau(_))));
        let (has_z, has_tilt) = (has(&|c| matches!(c, Col::BaseZ)), has(&|c| matches!(c, Col::Tilt)));
        if !has(&|c| matches!(c, Col::Q(_))) {
            return Err("run CSV: no joint position columns".into());
        }

        let mut times = Vec::new();
        let (mut q, mut qd, mut tau) = (vec![Vec::new(); nj], vec![Vec::new(); nj], vec![Vec::new(); nj]);
        let (mut base_z, mut tilt) = (Vec::new(), Vec::new());
        for (row, line) in lines.enumerate() {
            let cells: Vec<&str> = line.split(',').collect();
            if cells.len() != names.len() {
                return Err(format!("run CSV: row {} has {} cells, header has {}", row + 1, cells.len(), names.len()));
            }
            let num = |i: usize| cells[i].trim().parse::<f64>().map_err(|e| format!("run CSV: row {}: '{}': {e}", row + 1, cells[i]));
            times.push(num(time_col)?);
            // Joints with no qd / tau column in an otherwise full file read NaN.
            for c in 0..nj {
                qd[c].push(f64::NAN);
                tau[c].push(f64::NAN);
                q[c].push(f64::NAN);
            }
            for (i, kind) in kinds.iter().enumerate() {
                match kind {
                    Col::Q(c) => *q[*c].last_mut().unwrap() = num(i)?,
                    Col::Qd(c) => *qd[*c].last_mut().unwrap() = num(i)?,
                    Col::Tau(c) => *tau[*c].last_mut().unwrap() = num(i)?,
                    Col::BaseZ => base_z.push(num(i)?),
                    Col::Tilt => tilt.push(num(i)?),
                    Col::Skip => {}
                }
            }
        }
        let plan = meta.get("plan").and_then(|p| serde_json::from_str(p).ok()).and_then(|v| PlanSnapshot::from_json(&v));
        Ok(Self {
            name: meta.get("name").cloned().unwrap_or_else(|| "imported".into()),
            sequence: meta.get("sequence").cloned().unwrap_or_default(),
            source: meta.get("source").cloned().unwrap_or_else(|| "imported CSV".into()),
            joints,
            times,
            q,
            qd: has_qd.then_some(qd),
            tau: has_tau.then_some(tau),
            base_z: has_z.then_some(base_z),
            tilt: has_tilt.then_some(tilt),
            plan,
        })
    }
}

/// Headline numbers of a run, for comparing runs side by side.
#[derive(Clone, Debug)]
pub struct RunSummary {
    pub issues: Vec<Issue>,
    /// Largest `|τ| / effort` and its joint (only joints declaring effort).
    pub worst_torque: Option<(usize, f64)>,
    /// Largest `|q̇| / velocity` and its joint.
    pub worst_velocity: Option<(usize, f64)>,
    /// `(joint, max |q − q_plan|, time)`, worst first.
    pub tracking: Vec<(usize, f64, f64)>,
    pub max_tilt: Option<f64>,
    pub min_base_z: Option<f64>,
}

impl RunSummary {
    pub fn new(model: &RobotModel, run: &RunRecord, plan: Option<&Trajectory>) -> Self {
        let series = run.series(model);
        let worst = |sig: &Vec<Vec<f64>>, limit: &dyn Fn(usize) -> f64| {
            series
                .joints
                .iter()
                .enumerate()
                .filter(|(_, ji)| limit(**ji) > 0.0)
                .map(|(c, &ji)| (ji, sig[c].iter().filter(|v| v.is_finite()).fold(0.0, |m, v| f64::max(m, v.abs())) / limit(ji)))
                .max_by(|a, b| a.1.total_cmp(&b.1))
        };
        let finite = |s: &Option<Vec<f64>>| s.as_ref().map(|v| v.iter().copied().filter(|x| x.is_finite()).collect::<Vec<_>>());
        Self {
            issues: motion::check(model, &series)
                .into_iter()
                // NaN (unrecorded) samples never raise issues; skip kinds the run lacks.
                .filter(|i| match i.kind {
                    IssueKind::Velocity => run.qd.is_some(),
                    IssueKind::Torque => run.tau.is_some(),
                    IssueKind::Position => true,
                })
                .collect(),
            worst_torque: run.tau.as_ref().and_then(|_| worst(&series.tau, &|ji| model.joints[ji].effort)),
            worst_velocity: run.qd.as_ref().and_then(|_| worst(&series.qd, &|ji| model.joints[ji].velocity)),
            tracking: plan
                .map(|p| motion::tracking_error(p, &series).into_iter().filter(|t| t.1.is_finite()).collect())
                .unwrap_or_default(),
            max_tilt: finite(&run.tilt).and_then(|v| v.into_iter().reduce(f64::max)),
            min_base_z: finite(&run.base_z).and_then(|v| v.into_iter().reduce(f64::min)),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use misarta::trajectory::InterpolationKind;
    use nalgebra as na;

    fn model() -> RobotModel {
        let mut m = RobotModel::new_empty("bot");
        for (parent, child, joint) in [("base_link", "a", "j1"), ("a", "b", "j2")] {
            m.add_child(
                parent,
                child,
                joint,
                "revolute",
                na::Isometry3::identity(),
                na::Vector3::z(),
                crate::robot::GeomData::Sphere { radius: 0.01 },
                [1.0; 4],
                -1.0,
                1.0,
            )
            .unwrap();
        }
        for j in &mut m.joints {
            j.effort = 2.0;
            j.velocity = 4.0;
        }
        m.poses = vec![NamedPose {
            name: "up".into(),
            angles: [("j1".to_string(), 0.5)].into_iter().collect(),
            duration: 1.0,
            kind: InterpolationKind::CubicSmooth,
        }];
        m.sequences = vec![Sequence {
            name: "s".into(),
            steps: vec![SequenceStep { pose_name: "up".into(), duration: 1.0, kind: InterpolationKind::QuinticSmooth }],
        }];
        m.rebuild_misarta_model();
        m
    }

    fn sample_run(m: &RobotModel) -> RunRecord {
        let times = vec![0.0, 0.5, 1.0];
        let q = vec![vec![0.0, 0.0], vec![0.3, 0.1], vec![0.5, 1.2]];
        let qd = vec![vec![0.0, 0.0], vec![1.0, 5.0], vec![0.0, 0.0]];
        let tau = vec![vec![0.5, 0.0], vec![-3.0, 0.2], vec![0.1, 0.0]];
        let plan = PlanSnapshot::capture(m, &m.sequences[0], &[0.0, 0.0]);
        RunRecord::from_rows(
            m,
            "run 1",
            "s",
            "MuJoCo, on ground, settle 1.00 s",
            times,
            &q,
            Some(&qd),
            Some(&tau),
            Some(vec![0.3, 0.29, 0.31]),
            Some(vec![0.0, 0.1, 0.05]),
            Some(plan),
        )
    }

    #[test]
    fn csv_roundtrip_keeps_signals_and_plan() {
        let m = model();
        let run = sample_run(&m);
        let back = RunRecord::from_csv(&run.to_csv()).unwrap();
        assert_eq!((back.name.as_str(), back.sequence.as_str()), ("run 1", "s"));
        assert_eq!(back.source, run.source);
        assert_eq!(back.joints, ["j1", "j2"]);
        assert_eq!(back.times, run.times);
        assert_eq!(back.q, run.q);
        assert_eq!(back.qd, run.qd);
        assert_eq!(back.tau, run.tau);
        assert_eq!(back.base_z, run.base_z);
        assert_eq!(back.tilt, run.tilt);
        let plan = back.plan.unwrap();
        assert_eq!(plan.sequence.steps[0].kind, InterpolationKind::QuinticSmooth);
        assert_eq!(plan.poses[0].angles["j1"], 0.5);
        assert_eq!(plan.start["j2"], 0.0);
    }

    #[test]
    fn minimal_hardware_log_reads_bare_joint_names() {
        let csv = "time,j2,j1\n0,0.1,0.2\n0.01,0.15,0.25\n";
        let run = RunRecord::from_csv(csv).unwrap();
        assert_eq!(run.joints, ["j2", "j1"]);
        assert_eq!(run.q[1], [0.2, 0.25]);
        assert!(run.qd.is_none() && run.tau.is_none() && run.plan.is_none());
        // Series maps by name onto the model's joint order.
        let s = run.series(&model());
        assert_eq!(s.q[0], [0.2, 0.25]);
        assert_eq!(s.q[1], [0.1, 0.15]);
    }

    #[test]
    fn summary_reports_ratios_issues_and_tracking() {
        let m = model();
        let run = sample_run(&m);
        let plan = Trajectory::from_sequence(&m, &m.sequences[0], &[0.0, 0.0]).unwrap();
        let s = RunSummary::new(&m, &run, Some(&plan));
        let j1 = m.joint_map["j1"];
        let j2 = m.joint_map["j2"];
        assert_eq!(s.worst_torque, Some((j1, 1.5)));
        assert_eq!(s.worst_velocity, Some((j2, 1.25)));
        let kinds: Vec<_> = s.issues.iter().map(|i| (i.joint, i.kind)).collect();
        assert!(kinds.contains(&(j1, IssueKind::Torque)), "{kinds:?}");
        assert!(kinds.contains(&(j2, IssueKind::Velocity)), "{kinds:?}");
        assert!(kinds.contains(&(j2, IssueKind::Position)), "{kinds:?}");
        // j2 should stay at 0 but reaches 1.2 at the end.
        assert_eq!(s.tracking[0].0, j2);
        assert!((s.tracking[0].1 - 1.2).abs() < 1e-9);
        assert_eq!(s.max_tilt, Some(0.1));
        assert_eq!(s.min_base_z, Some(0.29));

        // Without qd / tau there are no velocity / torque numbers or issues.
        let bare = RunRecord { qd: None, tau: None, ..run };
        let s = RunSummary::new(&m, &bare, None);
        assert!(s.worst_torque.is_none() && s.worst_velocity.is_none() && s.tracking.is_empty());
        assert!(s.issues.iter().all(|i| i.kind == IssueKind::Position));
    }

    #[test]
    fn plan_restore_brings_back_sequence_and_poses() {
        let mut m = model();
        let snap = PlanSnapshot::capture(&m, &m.sequences[0], &[0.2, -0.1]);
        // Edit after the run.
        m.sequences[0].steps[0].duration = 3.0;
        m.poses[0].angles.insert("j1".into(), 0.9);
        let (idx, start) = snap.restore(&mut m);
        assert_eq!(idx, 0);
        assert_eq!(m.sequences[0].steps[0].duration, 1.0);
        assert_eq!(m.poses[0].angles["j1"], 0.5);
        assert_eq!(start, [0.2, -0.1]);
    }
}
