//! Headless MuJoCo runs of a motion: run a sequence under stated
//! conditions as fast as the CPU allows and get a [`RunRecord`] back — the
//! same record the motion editor keeps for interactive runs, so batch runs
//! (condition sweeps) compare in the same table.

use crate::mjcf::{GroundPlaneCfg, MjcfExportOptions};
use crate::motion::{self, Trajectory};
use crate::motion_run::{PlanSnapshot, RunRecord};
use crate::mujoco_sim::MujocoSim;
use crate::robot::{RobotModel, Sequence};

/// How the robot is held.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Support {
    /// Free base on a flat ground plane at z = 0 (the base is lifted so
    /// the lowest geometry touches it).
    Ground,
    /// Base welded in place: a hanging test for limb motions.
    Fixed,
}

/// A push on a link during the motion.
#[derive(Clone, Debug, PartialEq)]
pub struct Push {
    /// Link to push; `None` = the root link.
    pub link: Option<String>,
    /// Plan time at which it starts (s).
    pub at: f64,
    /// World-frame force (N).
    pub force: [f64; 3],
    pub duration: f64,
}

/// Conditions of one run. The defaults describe a nominal robot on a
/// rubber-on-lab-floor ground with its motors' torque limits enforced.
#[derive(Clone, Debug, PartialEq)]
pub struct RunConditions {
    pub support: Support,
    /// Start pose held before the sequence (s): lets a legged robot land.
    pub settle: f64,
    /// Recording continues this long after the sequence ends (s).
    pub tail: f64,
    /// Sliding friction coefficient of every contact.
    pub ground_friction: f64,
    /// Scale on every joint's PD gains (`actuator_kp` / `actuator_kv`).
    pub kp_scale: f64,
    pub kv_scale: f64,
    /// Scale on every joint's passive damping.
    pub damping_scale: f64,
    /// Scale on every joint's torque limit (`effort`), e.g. 0.8 for weak or
    /// hot motors. Only meaningful with `enforce_limits`.
    pub effort_scale: f64,
    /// Clamp commands to the motors' torque limits, as real motors do.
    pub enforce_limits: bool,
    pub push: Option<Push>,
    /// Physics step (s); `None` = MuJoCo's default.
    pub timestep: Option<f64>,
}

impl Default for RunConditions {
    fn default() -> Self {
        Self {
            support: Support::Ground,
            settle: 1.0,
            tail: 0.5,
            ground_friction: 0.7,
            kp_scale: 1.0,
            kv_scale: 1.0,
            damping_scale: 1.0,
            effort_scale: 1.0,
            enforce_limits: true,
            push: None,
            timestep: None,
        }
    }
}

impl RunConditions {
    /// Short description listing only what differs from the defaults,
    /// e.g. `"MuJoCo, on ground, μ 0.4, kp×0.8"`.
    pub fn label(&self) -> String {
        let d = Self::default();
        let mut parts = vec![
            "MuJoCo".to_string(),
            match self.support {
                Support::Ground => "on ground".into(),
                Support::Fixed => "fixed base".into(),
            },
        ];
        let mut scale = |name: &str, v: f64, dv: f64| {
            if (v - dv).abs() > 1e-9 {
                parts.push(format!("{name}×{v:.2}"));
            }
        };
        scale("kp", self.kp_scale, d.kp_scale);
        scale("kv", self.kv_scale, d.kv_scale);
        scale("damping", self.damping_scale, d.damping_scale);
        scale("effort", self.effort_scale, d.effort_scale);
        if self.support == Support::Ground && (self.ground_friction - d.ground_friction).abs() > 1e-9 {
            parts.push(format!("μ {:.2}", self.ground_friction));
        }
        if !self.enforce_limits {
            parts.push("no torque limits".into());
        }
        if let Some(p) = &self.push {
            let f = (p.force[0].powi(2) + p.force[1].powi(2) + p.force[2].powi(2)).sqrt();
            parts.push(format!("push {f:.0} N @ {:.2}s", p.at));
        }
        if let Some(dt) = self.timestep {
            parts.push(format!("dt {:.1} ms", dt * 1e3));
        }
        parts.join(", ")
    }
}

/// Run `seq` from `start_q` under `cond` and record it (plan time: 0 = the
/// sequence starts). The model is not modified.
pub fn simulate(
    model: &RobotModel,
    seq: &Sequence,
    start_q: &[f64],
    cond: &RunConditions,
    name: impl Into<String>,
) -> Result<RunRecord, String> {
    let duration = Trajectory::from_sequence(model, seq, start_q)?.duration();
    let anim = motion::keyframe_animation(model, seq, start_q, cond.settle)?;

    let mut work = model.clone();
    for j in &mut work.joints {
        j.actuator_kp *= cond.kp_scale;
        j.actuator_kv *= cond.kv_scale;
        j.joint_damping *= cond.damping_scale;
        j.effort *= cond.effort_scale;
    }
    work.joint_positions = start_q.to_vec();
    work.rebuild_misarta_model();

    let opts = MjcfExportOptions {
        base_pos: None,
        ground_plane: (cond.support == Support::Ground).then_some(GroundPlaneCfg {
            z: 0.0,
            half_size: 5.0,
            roll: 0.0,
            pitch: 0.0,
        }),
        base_locked_axes: [cond.support == Support::Fixed; 6],
        bake_actuator_limits: cond.enforce_limits,
        bake_joint_position_limits: cond.enforce_limits,
        default_friction: [cond.ground_friction, 0.005, 0.0001],
        timestep: cond.timestep,
        ..Default::default()
    };
    let mut sim = MujocoSim::new(&work, opts)?;
    let dt = sim.timestep();
    let end = cond.settle + duration + cond.tail;
    let frames = (end / dt).ceil() as usize;
    sim.set_trace_max(frames + 10);
    sim.start_sequence(anim, seq.name.clone());

    let push_link = cond.push.as_ref().map(|p| p.link.clone().unwrap_or_else(|| work.root_link.clone()));
    let mut pushed = false;
    for k in 0..frames {
        if let (Some(p), Some(link), false) = (&cond.push, &push_link, pushed) {
            if k as f64 * dt >= cond.settle + p.at {
                sim.apply_external_force(link, p.force, [0.0; 3], p.duration);
                pushed = true;
            }
        }
        sim.step_n_frames(&mut work, 1, cond.enforce_limits);
    }

    let (mut times, mut q, mut qd, mut tau, mut base_z, mut tilt) =
        (Vec::new(), Vec::new(), Vec::new(), Vec::new(), Vec::new(), Vec::new());
    for f in sim.trace().filter(|f| f.time >= cond.settle) {
        times.push(f.time - cond.settle);
        q.push(f.q.clone());
        qd.push(f.qvel.clone());
        tau.push(f.tau.clone());
        base_z.push(f.base_pos.map_or(f64::NAN, |p| p[2]));
        tilt.push(f.base_quat.map_or(f64::NAN, |[_, x, y, _]| (1.0 - 2.0 * (x * x + y * y)).clamp(-1.0, 1.0).acos()));
    }
    if times.is_empty() {
        return Err("MuJoCo run recorded no samples".into());
    }
    Ok(RunRecord::from_rows(
        model,
        name,
        seq.name.clone(),
        cond.label(),
        times,
        &q,
        Some(&qd),
        Some(&tau),
        Some(base_z),
        Some(tilt),
        Some(PlanSnapshot::capture(model, seq, start_q)),
    ))
}
