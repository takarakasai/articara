//! Tune namiashi's `jump` for the real robot: search crouch depth and
//! timing for the highest jump that stays within the motors' limits.
//!
//! Every candidate is a sequence `crouch → extend → tuck → land` built from
//! the fixture's own poses (scaled), run headless in MuJoCo on the ground
//! with the motors' torque limits enforced (`articara::motion_sim`). A
//! candidate is *feasible* when its plan breaks no joint range / velocity
//! limit and the run passes the motion editor's criteria (no fall, tracking
//! within bounds, no limit issues during the motion). Among feasible ones
//! the highest jump wins — measured as the lowest point of the robot's
//! geometry above the ground (from the recorded joint angles and base
//! height), not as base height, since the start pose on straight legs is
//! the tallest of all. The winner is then swept over friction, gains,
//! motor torque and pushes (scaled to the robot's weight).
//!
//! ```bash
//! cargo xtask run --features mujoco --example jump_tune -- [--write]
//! ```
//!
//! `--write` adds the winner to the fixture as sequence `jump_hw` (poses
//! `jump_hw_*`), leaving the original `jump` untouched.

use std::collections::BTreeMap;
use std::path::PathBuf;

use articara::motion::{self, Trajectory};
use articara::motion_run::{Criteria, RunSummary};
use articara::motion_sim::{RunConditions, SweepAxis, one_at_a_time, simulate};
use articara::robot::{NamedPose, RobotModel, Sequence, SequenceStep};
use misarta::trajectory::InterpolationKind;

const LEGS: [&str; 4] = ["FL", "FR", "RL", "RR"];

/// Leg pose with thigh / calf angles (hips and arm at 0).
fn leg_pose(name: &str, thigh: f64, calf: f64) -> NamedPose {
    let mut angles = BTreeMap::new();
    for leg in LEGS {
        angles.insert(format!("{leg}_hip_joint"), 0.0);
        angles.insert(format!("{leg}_thigh_joint"), thigh);
        angles.insert(format!("{leg}_calf_joint"), calf);
    }
    angles.insert("arm_pitch_joint".into(), 0.0);
    NamedPose { name: name.into(), angles, duration: 1.0, kind: InterpolationKind::QuinticSmooth }
}

#[derive(Clone, Debug)]
struct Candidate {
    /// Crouch depth as a fraction of the original `constrain_2`
    /// (thigh 1.3, calf −2.6).
    crouch: f64,
    t_crouch: f64,
    /// Pause in the crouch before pushing off (lets the body settle).
    t_hold: f64,
    t_extend: f64,
    extend_kind: InterpolationKind,
    /// Extension target (same scale; the original `extend` is ≈ 0.23).
    extend: f64,
    /// Tuck depth (same scale as `crouch`).
    tuck: f64,
}

/// Seconds for the largest joint move between two poses at `frac` of each
/// joint's velocity limit, for a curve whose peak speed is `peak` × mean.
fn safe_duration(model: &RobotModel, a: &NamedPose, b: &NamedPose, frac: f64, peak: f64) -> f64 {
    a.angles
        .iter()
        .filter_map(|(j, qa)| {
            let qb = b.angles.get(j)?;
            let v = model.joints[*model.joint_map.get(j)?].velocity;
            (v > 0.0).then(|| (qb - qa).abs() * peak / (v * frac))
        })
        .fold(0.0, f64::max)
}

fn peak_factor(kind: InterpolationKind) -> f64 {
    match kind {
        InterpolationKind::Linear => 1.0,
        InterpolationKind::CubicSmooth => 1.5,
        InterpolationKind::QuinticSmooth => 1.875,
    }
}

/// The candidate as poses + sequence named `prefix…`.
fn build(model: &RobotModel, c: &Candidate, prefix: &str) -> (Vec<NamedPose>, Sequence) {
    // `.misa` pose names are identifiers: `_`, not `/`.
    let crouch = leg_pose(&format!("{prefix}_crouch"), 1.3 * c.crouch, -2.6 * c.crouch);
    let extend = leg_pose(&format!("{prefix}_extend"), 1.3 * c.extend, -2.6 * c.extend);
    let tuck = leg_pose(&format!("{prefix}_tuck"), 1.3 * c.tuck, -2.6 * c.tuck);
    // Land in a moderate crouch (knees bent to absorb the impact).
    let land = leg_pose(&format!("{prefix}_land"), 0.5, -1.0);
    // Tuck as fast as the motors allow (80 % of their velocity limit).
    let t_tuck = safe_duration(model, &extend, &tuck, 0.8, peak_factor(InterpolationKind::QuinticSmooth)).max(0.05);
    let step = |pose: &NamedPose, duration: f64, kind| SequenceStep { pose_name: pose.name.clone(), duration, kind };
    let mut steps = vec![step(&crouch, c.t_crouch, InterpolationKind::QuinticSmooth)];
    if c.t_hold > 0.0 {
        steps.push(step(&crouch, c.t_hold, InterpolationKind::Linear));
    }
    steps.push(step(&extend, c.t_extend, c.extend_kind));
    steps.push(step(&tuck, t_tuck, InterpolationKind::QuinticSmooth));
    steps.push(step(&land, 0.4, InterpolationKind::QuinticSmooth));
    (vec![crouch, extend, tuck, land], Sequence { name: prefix.into(), steps })
}

/// Lowest world z of the collision geometry — what actually touches the
/// ground in MuJoCo (visual meshes can sit above the foot spheres and
/// would read as clearance while standing).
fn collision_min_z(model: &RobotModel) -> f64 {
    use articara::robot::GeomData;
    use nalgebra::Point3;
    let tf = model.compute_transforms();
    let mut min = f64::INFINITY;
    for link in &model.links {
        let Some(world) = tf.get(&link.name) else { continue };
        for c in &link.collisions {
            let pose = world * c.origin;
            let pts: Vec<Point3<f32>> = match &c.geometry {
                GeomData::Sphere { radius } => {
                    min = min.min((pose.translation.vector.z - radius) as f64);
                    continue;
                }
                GeomData::Box { hx, hy, hz } => (0..8)
                    .map(|i| {
                        let s = |b: usize| if i >> b & 1 == 1 { 1.0 } else { -1.0 };
                        Point3::new(s(0) * hx, s(1) * hy, s(2) * hz)
                    })
                    .collect(),
                GeomData::Capsule { radius, half_length } => {
                    // Lowest point: the lower cap's centre minus the radius.
                    for z in [*half_length, -half_length] {
                        min = min.min(((pose * Point3::new(0.0, 0.0, z)).z - radius) as f64);
                    }
                    continue;
                }
                GeomData::Cylinder { radius, half_length } => (0..32)
                    .map(|i| {
                        let a = (i % 16) as f32 / 16.0 * std::f32::consts::TAU;
                        let z = if i < 16 { *half_length } else { -half_length };
                        Point3::new(radius * a.cos(), radius * a.sin(), z)
                    })
                    .collect(),
                GeomData::Mesh { mesh, .. } => {
                    mesh.vertices.iter().map(|v| Point3::new(v.x as f32, v.y as f32, v.z as f32)).collect()
                }
            };
            for p in pts {
                min = min.min((pose * p).z as f64);
            }
        }
    }
    min
}

struct Score {
    /// Peak clearance of the robot's lowest point above the ground (m).
    rise: f64,
    /// Time with clearance above 3 mm (s).
    flight: f64,
    plan_issues: usize,
    verdict: Vec<String>,
    summary: RunSummary,
}

fn evaluate(model: &RobotModel, seq: &Sequence, start: &[f64], criteria: &Criteria, cond: &RunConditions) -> Result<Score, String> {
    let plan = Trajectory::from_sequence(model, seq, start)?;
    let plan_issues = motion::check(model, &motion::sample_series(model, &plan, 0.005)).len();
    let run = simulate(model, seq, start, cond, "c")?;
    let summary = RunSummary::new(model, &run, Some(&plan));
    let z = run.base_z.as_ref().ok_or("no base height")?;
    // Lowest point of the geometry at each sample (base assumed upright —
    // the criteria reject tilted runs anyway), relative to t = 0 when the
    // feet stand on the ground.
    let mut work = model.clone();
    let cols: Vec<Option<usize>> = model.joints.iter().map(|j| run.column(&j.name)).collect();
    let mut clearance = Vec::new();
    for k in (0..run.times.len()).step_by(5) {
        work.joint_positions = cols.iter().map(|c| c.map_or(0.0, |c| run.q[c][k])).collect();
        work.base_transform = nalgebra::Isometry3::translation(0.0, 0.0, z[k]);
        clearance.push((run.times[k], collision_min_z(&work)));
    }
    let ground = clearance.first().map_or(0.0, |c| c.1);
    let rise = clearance.iter().map(|c| c.1 - ground).fold(0.0, f64::max);
    let dt = clearance.get(1).map_or(0.0, |c| c.0) - clearance.first().map_or(0.0, |c| c.0);
    let flight = clearance.iter().filter(|c| c.1 - ground > 0.003).count() as f64 * dt;
    Ok(Score { rise, flight, plan_issues, verdict: summary.verdict(criteria), summary })
}

fn kind_name(k: InterpolationKind) -> &'static str {
    match k {
        InterpolationKind::Linear => "lin",
        InterpolationKind::CubicSmooth => "cub",
        InterpolationKind::QuinticSmooth => "qui",
    }
}

fn main() -> Result<(), String> {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/namiashi/namiashi.misa");
    let model = RobotModel::from_misa(&path)?;
    let start = model.joint_positions.clone();
    let criteria = Criteria::default();
    let nominal = RunConditions::default();

    // Baseline: the original sequence.
    let jump = model.sequences.iter().find(|s| s.name == "jump").ok_or("no 'jump'")?.clone();
    let base = evaluate(&model, &jump, &start, &criteria, &nominal)?;
    println!(
        "original jump: clearance {:.1} mm, flight {:.0} ms, plan issues {}, verdict {:?}",
        base.rise * 1e3,
        base.flight * 1e3,
        base.plan_issues,
        base.verdict
    );

    let mut results = Vec::new();
    for crouch in [0.55, 0.7, 0.85] {
        for t_hold in [0.0, 0.3] {
            for extend in [0.23, 0.12, 0.05] {
                for t_extend in [0.1, 0.15, 0.2, 0.3] {
                    for extend_kind in [InterpolationKind::Linear, InterpolationKind::CubicSmooth] {
                        for tuck in [0.5, 0.75] {
                            let c = Candidate { crouch, t_crouch: 0.6, t_hold, t_extend, extend_kind, extend, tuck };
                            let (poses, seq) = build(&model, &c, "cand");
                            let mut m = model.clone();
                            m.poses.extend(poses);
                            match evaluate(&m, &seq, &start, &criteria, &nominal) {
                                Ok(s) => results.push((c, s)),
                                Err(e) => eprintln!("{c:?}: {e}"),
                            }
                        }
                    }
                }
            }
        }
    }
    let feasible = |s: &Score| s.plan_issues == 0 && s.verdict.is_empty();
    let n_ok = results.iter().filter(|(_, s)| feasible(s)).count();
    println!("{} candidates, {} feasible", results.len(), n_ok);
    results.sort_by(|a, b| (feasible(&b.1), b.1.rise).partial_cmp(&(feasible(&a.1), a.1.rise)).unwrap());
    println!("  crouch  hold   ext  t_ext kind tuck | clear mm  air ms  plan  track°  tilt°  τ/max  verdict");
    for (c, s) in results.iter().take(12) {
        println!(
            "  {:5.2} {:5.2} {:5.2} {:6.2} {:>4} {:4.2} | {:8.1} {:7.0} {:5} {:7.1} {:6.1} {:6.2}  {}",
            c.crouch,
            c.t_hold,
            c.extend,
            c.t_extend,
            kind_name(c.extend_kind),
            c.tuck,
            s.rise * 1e3,
            s.flight * 1e3,
            s.plan_issues,
            s.summary.tracking.first().map_or(0.0, |t| t.1.to_degrees()),
            s.summary.max_tilt.unwrap_or(0.0).to_degrees(),
            s.summary.worst_torque.map_or(0.0, |t| t.1),
            if s.verdict.is_empty() { "ok".to_string() } else { s.verdict.join("; ") }
        );
    }
    // For reference: what a looser tracking bound (saturating during the
    // push-off is normal on hardware) would allow.
    let loose = Criteria { max_tracking: 20f64.to_radians(), ..criteria.clone() };
    if let Some((c, s)) = results
        .iter()
        .filter(|(_, s)| s.plan_issues == 0 && s.summary.verdict(&loose).is_empty())
        .max_by(|a, b| a.1.rise.total_cmp(&b.1.rise))
    {
        println!(
            "with tracking ≤ 20°: clearance {:.1} mm, tracking {:.1}° ({c:?})",
            s.rise * 1e3,
            s.summary.tracking.first().map_or(0.0, |t| t.1.to_degrees())
        );
    }
    // Best infeasible jump too, to show what the limits cost.
    if let Some((c, s)) = results.iter().filter(|(_, s)| !feasible(s)).max_by(|a, b| a.1.rise.total_cmp(&b.1.rise)) {
        println!(
            "highest infeasible: clearance {:.1} mm ({c:?}): plan issues {}, {:?}",
            s.rise * 1e3,
            s.plan_issues,
            s.verdict
        );
    }

    // Robustness decides among the best few: sweep the 8 highest feasible
    // candidates and keep the one passing most conditions (then highest).
    let weight = model.links.iter().map(|l| l.inertial.mass).sum::<f64>() * 9.80665;
    let axes = [
        SweepAxis::GroundFriction(vec![0.4, 1.0]),
        SweepAxis::KpScale(vec![0.7, 1.4]),
        SweepAxis::KvScale(vec![0.5, 2.0]),
        SweepAxis::DampingScale(vec![0.0, 3.0]),
        SweepAxis::EffortScale(vec![0.8, 0.9]),
        // 0.2 and 0.4 × body weight for 0.1 s, mid-crouch.
        SweepAxis::Push {
            magnitudes: vec![0.2 * weight, 0.4 * weight],
            direction: [1.0, 0.0, 0.0],
            at: 0.3,
            duration: 0.1,
        },
    ];
    let conds = one_at_a_time(&nominal, &axes);
    println!("sweeping the top candidates over {} conditions:", conds.len());
    let mut ranked = Vec::new();
    for (c, s) in results.iter().filter(|(_, s)| feasible(s)).take(8) {
        let (poses, seq) = build(&model, c, "jump_hw");
        let mut m = model.clone();
        m.poses.extend(poses);
        let mut lines = Vec::new();
        let mut pass = 0;
        for cond in &conds {
            let r = evaluate(&m, &seq, &start, &criteria, cond)?;
            pass += r.verdict.is_empty() as usize;
            lines.push(format!(
                "    {:<45} clearance {:5.1} mm  {}",
                cond.label(),
                r.rise * 1e3,
                if r.verdict.is_empty() { "✔".to_string() } else { format!("✖ {}", r.verdict.join("; ")) }
            ));
        }
        println!("  {pass:2}/{} pass, clearance {:5.1} mm: {c:?}", conds.len(), s.rise * 1e3);
        ranked.push((pass, s.rise, c.clone(), lines));
    }
    ranked.sort_by(|a, b| (b.0, b.1).partial_cmp(&(a.0, a.1)).unwrap());
    let Some((pass, rise, best, lines)) = ranked.into_iter().next() else {
        return Err("no feasible candidate".into());
    };
    println!("chosen ({pass}/{} pass, clearance {:.1} mm): {best:?}", conds.len(), rise * 1e3);
    for l in lines {
        println!("{l}");
    }
    let (poses, seq) = build(&model, &best, "jump_hw");
    for st in &seq.steps {
        println!("  step {:<16} {:.3} s {:?}", st.pose_name, st.duration, st.kind);
    }

    if std::env::args().any(|a| a == "--write") {
        // Insert as text so the rest of the fixture stays byte-identical
        // (a full `save_as_misa` would reformat and reorder the file).
        let text = std::fs::read_to_string(&path).map_err(|e| e.to_string())?;
        if text.contains("\"jump_hw") {
            return Err("the fixture already has jump_hw; remove it first".into());
        }
        let kind = |k: InterpolationKind| format!("{k:?}");
        let mut pose_block = String::new();
        for p in &poses {
            pose_block.push_str(&format!("[[pose]]\nname = \"{}\"\nduration = {:?}\nkind = \"{}\"\n\n[pose.angles]\n", p.name, p.duration, kind(p.kind)));
            for (j, q) in &p.angles {
                pose_block.push_str(&format!("{j} = {q:?}\n"));
            }
            pose_block.push('\n');
        }
        let mut seq_block = format!("[[sequence]]\nname = \"{}\"\n\n", seq.name);
        for st in &seq.steps {
            seq_block.push_str(&format!(
                "[[sequence.steps]]\npose_name = \"{}\"\nduration = {:?}\nkind = \"{}\"\n\n",
                st.pose_name, st.duration, kind(st.kind)
            ));
        }
        let at_seq = text.find("[[sequence]]").ok_or("no [[sequence]] in the fixture")?;
        let at_home = text.find("[home]").ok_or("no [home] in the fixture")?;
        let out = format!("{}{pose_block}{}{seq_block}{}", &text[..at_seq], &text[at_seq..at_home], &text[at_home..]);
        // Must still load, with the new sequence.
        std::fs::write(&path, &out).map_err(|e| e.to_string())?;
        let check = RobotModel::from_misa(&path)?;
        if !check.sequences.iter().any(|s| s.name == "jump_hw") {
            std::fs::write(&path, &text).map_err(|e| e.to_string())?;
            return Err("written fixture lost jump_hw; restored".into());
        }
        println!("wrote sequence 'jump_hw' to {}", path.display());
    }
    Ok(())
}
