//! What to do with the QP's answer, including when it did not produce one.
//!
//! The policy, its tally and the gravity + posture fallback are
//! `misa_biped::command` (moved there 2026-10-31); these wrappers keep the
//! names and signatures the examples call, printing to stdout as before.
//! The plant write ([`write_to_plant`]) is MuJoCo's and stays here.

use misa_wbc::SolveStatus;
use nalgebra as na;

use super::rig::{BipedRig, CtrlMode};

/// `misa_biped::command`'s lines to stdout, as these used to `println!`.
struct Stdout;

impl misa_biped::sink::LogSink for Stdout {
    fn line(&mut self, s: &str) {
        println!("{s}");
    }
    fn err_line(&mut self, s: &str) {
        eprintln!("{s}");
    }
}

/// Running tally of degraded solves (`misa_biped::command::DegradedTally`;
/// its fields through `Deref`).
#[derive(Default)]
pub struct DegradedTally(misa_biped::command::DegradedTally);

impl std::ops::Deref for DegradedTally {
    type Target = misa_biped::command::DegradedTally;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DegradedTally {
    pub fn new() -> Self {
        Self(misa_biped::command::DegradedTally::new())
    }

    pub fn observe(&mut self, status: &SolveStatus, t: f64, tick: usize, nc: usize, level_names: &[&str]) {
        self.0.observe(status, t, tick, nc, level_names, &mut Stdout);
    }

    /// Final tally by NAME rather than index.
    pub fn report(&self) {
        self.0.report(&mut Stdout);
    }
}

/// How a degraded solve is turned into something safe to command
/// (`misa_biped::command::CommandPolicy`; its fields through `Deref`).
pub struct CommandPolicy(misa_biped::command::CommandPolicy);

impl std::ops::Deref for CommandPolicy {
    type Target = misa_biped::command::CommandPolicy;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl std::ops::DerefMut for CommandPolicy {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl CommandPolicy {
    pub fn new(n_joints: usize, fallback_max_level: usize, hold_bridge: u32, hold_last: bool, blend_ticks: u32) -> Self {
        Self(misa_biped::command::CommandPolicy::new(n_joints, fallback_max_level, hold_bridge, hold_last, blend_ticks))
    }

    /// See `misa_biped::command::CommandPolicy::apply`.
    pub fn apply(
        &mut self,
        status: &SolveStatus,
        robot_taus: &mut [f64],
        t: f64,
        dt: f64,
        nc: usize,
        fallback: &dyn Fn(&mut [f64]),
    ) -> bool {
        self.0.apply(status, robot_taus, t, dt, nc, fallback, &mut Stdout)
    }
}

/// The degraded-solve fallback (`misa_biped::command::gravity_plus_posture`)
/// on the rig's actuated rows, seed posture and torque box. `q_joint` is the
/// joint angles the PD reads (harness order): the plant's
/// `rig.robot.joint_positions` as it always was, or the state estimate's
/// (kyo46rs_walk's `JOINT_Q_EST`).
pub fn gravity_plus_posture(
    rig: &BipedRig,
    q_joint: &[f64],
    tau_gravity: &na::DVector<f64>,
    v: &[f64],
    hold_kp: f64,
    hold_kd: f64,
    out: &mut [f64],
) {
    misa_biped::command::gravity_plus_posture(
        &rig.actuated(), &rig.q_seed, &rig.torque_max, q_joint, tau_gravity, v, hold_kp, hold_kd, out,
    );
}

/// Hand the command to the plant in whichever form the control mode wants.
/// `q_joint` is the joint angles Hybrid integrates from (harness order; the
/// plant's `rig.robot.joint_positions`, or the estimate's under
/// kyo46rs_walk's `JOINT_Q_EST`); the other modes do not read it.
pub fn write_to_plant(
    rig: &mut BipedRig,
    mode: CtrlMode,
    q_joint: &[f64],
    robot_taus: &[f64],
    qddot: &na::DVector<f64>,
    v: &[f64],
    dt: f64,
) {
    match mode {
        CtrlMode::Velocity | CtrlMode::Servo => {
            for (ji, vi) in rig.actuated() {
                let qd_des = v[vi] + qddot[vi] * dt;
                if mode == CtrlMode::Servo {
                    // MuJoCo's own <velocity> servo: ctrl IS the target.
                    rig.sim.set_actuator_ctrl(ji, qd_des);
                } else {
                    rig.sim.set_velocity_target(ji, qd_des);
                }
            }
        }
        CtrlMode::Hybrid => {
            // Integrate the QP's own qddot one control period forward from
            // the MEASURED state, so the reference never drifts away from
            // where the robot actually is, and hand the plant the triple it
            // wants. tau goes in as feedforward: the PD is there to cover
            // what changes between ticks, not to reproduce the dynamics.
            for (ji, vi) in rig.actuated() {
                let qdd = qddot[vi];
                let q_now = q_joint[ji];
                let qd_now = v[vi];
                rig.sim
                    .set_position_target(ji, q_now + qd_now * dt + 0.5 * qdd * dt * dt);
                rig.sim.set_position_target_velocity(ji, qd_now + qdd * dt);
                rig.sim.set_torque_feedforward(ji, robot_taus[ji]);
            }
        }
        CtrlMode::Torque => {
            rig.sim.set_wbc_torques(robot_taus);
        }
    }
}
