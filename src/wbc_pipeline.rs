//! Glue layer between MuJoCo state, the gait controller, and the
//! Hierarchical WBC solver in `quadruped_gait::wbc`.
//!
//! On every tick the host calls [`WbcPipeline::solve`] with:
//! - the current `RobotModel` + `MujocoSim` (for `q`, `q̇`, body pose),
//! - the gait controller's last [`quadruped_gait::ControllerOutput`]
//!   (foot-body targets + stance flags) and joint mappings,
//! - the SRBD MPC's predicted GRFs,
//! - velocity / yaw commands and observations.
//!
//! It returns a per-`RobotModel`-joint torque vector that the host
//! hands to [`crate::mujoco_sim::MujocoSim::set_wbc_torques`].
//!
//! ## Frame convention
//!
//! The misarta model's floating-base is left at neutral (origin +
//! identity orientation) every tick — `MisartaCache::build_q` doesn't
//! sync the actual body pose. This means the "world frame" inside the
//! WBC's M / h / J quantities **is the body frame at neutral
//! orientation**. We treat all WBC inputs and outputs in this frame:
//!
//! - GRF references are passed in as-is (the SRBD MPC outputs them in
//!   world frame, but for upright bodies that ≈ body frame; small
//!   yaw-rotation error accepted in this iteration).
//! - Base-acceleration target is the desired body velocity error
//!   (`v_cmd_world − v_obs_world`) scaled by Kp — we don't try to
//!   un-rotate it.
//! - Foot-position references from `out.legs[i].foot_body` are already
//!   in body frame and used directly.
//!
//! Future work: wire the actual base pose (xpos / xquat from MuJoCo or
//! IMU + leg odom) into the misarta `q` so the WBC sees real
//! tilt / yaw. That removes the upright-body assumption.

use nalgebra as na;

use quadruped_gait::wbc::{self, WbcDims, WbcInputs};
use quadruped_gait::{ControllerOutput, KinematicsConfig, foot_jacobian_body, forward_leg_kinematics};

use crate::mujoco_sim::MujocoSim;
use crate::rbd::model::RobotModel;

/// Stateful wrapper around a single sim-tick WBC solve. Carries the
/// previous tick's foot-body targets so the swing-leg Cartesian PD's
/// "desired velocity" term can be finite-differenced (the gait
/// controller doesn't currently expose a swing-trajectory time
/// derivative).
#[derive(Debug, Clone)]
pub struct WbcPipeline {
    /// Foot link names, in canonical FL/FR/RL/RR slot order.
    pub foot_links: [String; 4],
    /// Per-leg [`misarta`] joint index for the foot frame, resolved
    /// once at construction so each tick doesn't repeat the lookup.
    foot_misarta_idx: [Option<usize>; 4],

    /// Cartesian PD gains for swing legs (Newtons-per-metre style;
    /// applied as Cartesian acceleration, so units are 1/s² and 1/s).
    pub swing_kp: f64,
    pub swing_kd: f64,
    /// Body linear / angular velocity-tracking gains driving the
    /// `base_accel` reference (units 1/s — applied to a velocity
    /// error).
    pub base_kp_lin: f64,
    pub base_kp_ang: f64,
    /// Friction coefficient for the contact pyramid (per foot).
    pub friction_mu: f64,

    /// Previous tick's body-frame foot-body targets, used to
    /// finite-difference the swing reference velocity. Initialised to
    /// the nominal stance pose so the first tick doesn't see a huge
    /// fictitious velocity.
    last_foot_body_des: [na::Vector3<f64>; 4],
}

impl WbcPipeline {
    pub fn new(robot: &RobotModel, foot_links: [String; 4]) -> Self {
        let mc = robot.mc();
        let mut foot_misarta_idx = [None; 4];
        for (slot, link) in foot_links.iter().enumerate() {
            let ji = robot.joints.iter().position(|j| &j.child_link == link);
            foot_misarta_idx[slot] = ji.and_then(|j| mc.a2m.get(j).copied().flatten());
        }
        Self {
            foot_links,
            foot_misarta_idx,
            swing_kp: 80.0,
            swing_kd: 8.0,
            base_kp_lin: 30.0,
            base_kp_ang: 30.0,
            friction_mu: 0.5,
            last_foot_body_des: [na::Vector3::zeros(); 4],
        }
    }

    /// One tick of the WBC pipeline.
    ///
    /// Returns a per-`robot.joints` torque vector. Entries for fixed
    /// joints stay at 0; entries for movable joints carry the WBC
    /// solution. Call [`MujocoSim::set_wbc_torques`] with this result
    /// (or `clear_wbc_torques` when the pipeline is disabled).
    ///
    /// `gait_out`'s `foot_body` is the per-foot target in body frame
    /// at the current gait sub-fraction; `joint_indices` /
    /// `joint_signs` come from
    /// [`crate::gait::GaitController::joint_indices`] /
    /// [`crate::gait::GaitController::joint_signs`].
    #[allow(clippy::too_many_arguments)]
    pub fn solve(
        &mut self,
        robot: &RobotModel,
        mj_sim: &MujocoSim,
        gait_out: &ControllerOutput,
        kin: &KinematicsConfig,
        joint_indices: [[usize; 3]; 4],
        joint_signs: [[f64; 3]; 4],
        v_cmd_world: &na::Vector3<f64>,
        wz_cmd: f64,
        v_obs_world: &na::Vector3<f64>,
        omega_obs_world: &na::Vector3<f64>,
        f_grf_world_des: &[na::Vector3<f64>; 4],
        contact_flag: [bool; 4],
        dt: f64,
    ) -> Vec<f64> {
        let mc = robot.mc();
        let nv = mc.model.nv;
        let na_count = joint_indices.iter().map(|leg| leg.len()).sum::<usize>(); // 12 for quadruped
        debug_assert_eq!(nv, 6 + na_count, "WBC assumes 6-DoF floating base");

        let dims = WbcDims {
            nv,
            nc: 4,
            na: na_count,
        };

        // ── Build q (neutral base + joint angles from RobotModel) ──
        let q = mc.build_q(robot);
        // ── Build v (base velocity = world velocity since base is at
        // identity in misarta; joint velocities from MuJoCo) ──────
        let mut v = vec![0.0_f64; nv];
        v[0] = v_obs_world.x;
        v[1] = v_obs_world.y;
        v[2] = v_obs_world.z;
        v[3] = omega_obs_world.x;
        v[4] = omega_obs_world.y;
        v[5] = omega_obs_world.z;
        for ji in 0..robot.joints.len() {
            let Some(mi) = mc.a2m.get(ji).and_then(|&m| m) else {
                continue;
            };
            if mc.model.joints[mi].joint_type.nv() != 1 {
                continue;
            }
            let vi = mc.model.v_idx[mi];
            if let Some((_, qd)) = mj_sim.joint_q_qd(&robot.joints[ji].name) {
                v[vi] = qd;
            }
        }

        // ── M, h ────────────────────────────────────────────────────
        let mass = misarta::crba::crba(&mc.model, &q);
        let h = misarta::rnea::nonlinear_effects(&mc.model, &q, &v);

        // ── Per-foot J_linear (3×nv) and dJ·v (3) ──────────────────
        let mut j_contact = na::DMatrix::zeros(12, nv);
        let mut dj_v = na::DVector::zeros(12);
        for slot in 0..4 {
            let Some(mi) = self.foot_misarta_idx[slot] else {
                continue;
            };
            let j_full = misarta::jacobian::compute_joint_jacobian(&mc.model, &q, mi);
            let dj_dt = misarta::jacobian::compute_joint_jacobian_time_derivative(
                &mc.model, &q, &v, mi,
            );
            let v_dvec = na::DVector::from_column_slice(&v);
            let dj_v_full = dj_dt * v_dvec;
            // Linear part is rows 0..3 (top half of the 6-row spatial Jacobian).
            for r in 0..3 {
                for c in 0..nv {
                    j_contact[(3 * slot + r, c)] = j_full[(r, c)];
                }
                dj_v[3 * slot + r] = dj_v_full[r];
            }
        }

        // ── a_base_des: P-control on body linear + angular velocity ─
        // We don't have an MPC base-acceleration reference (the SRBD
        // MPC outputs GRFs, not body accel), so approximate the
        // tracking goal as a velocity error scaled by Kp.
        let a_base_lin = self.base_kp_lin * (v_cmd_world - v_obs_world);
        let omega_cmd_world = na::Vector3::new(0.0, 0.0, wz_cmd);
        let a_base_ang = self.base_kp_ang * (omega_cmd_world - omega_obs_world);
        let a_base_des = na::DVector::from_iterator(
            6,
            [
                a_base_lin.x,
                a_base_lin.y,
                a_base_lin.z,
                a_base_ang.x,
                a_base_ang.y,
                a_base_ang.z,
            ],
        );

        // ── a_swing_des per foot (Cartesian PD in body frame) ──────
        // Body frame ≈ world frame here because misarta has base at
        // identity. Only the swing legs use these entries.
        let mut a_swing_des = na::DVector::zeros(12);
        for slot in 0..4 {
            let p_des = gait_out.legs[slot].foot_body;
            // Measured foot position in body frame via FK on actual q.
            let leg_kin = kin.legs()[slot];
            // Pull current IK-convention angles from MuJoCo joint state.
            let mut q_leg = [0.0_f64; 3];
            let mut qd_leg = [0.0_f64; 3];
            for k in 0..3 {
                let ji = joint_indices[slot][k];
                let sign = joint_signs[slot][k];
                if let Some((q_urdf, qd_urdf)) =
                    mj_sim.joint_q_qd(&robot.joints[ji].name)
                {
                    q_leg[k] = sign * q_urdf;
                    qd_leg[k] = sign * qd_urdf;
                }
            }
            let p_meas = forward_leg_kinematics(leg_kin, q_leg[0], q_leg[1], q_leg[2]);
            let j_leg = foot_jacobian_body(leg_kin, q_leg[0], q_leg[1], q_leg[2]);
            let qd_vec = na::Vector3::new(qd_leg[0], qd_leg[1], qd_leg[2]);
            let v_meas = j_leg * qd_vec;
            // Desired velocity from finite-differencing the gait's
            // foot-body target. Skipped on the very first tick (no
            // history yet) — small initial error gets soaked up by the
            // P-term alone.
            let v_des = if dt > 1e-6 {
                (p_des - self.last_foot_body_des[slot]) / dt
            } else {
                na::Vector3::zeros()
            };
            let a = self.swing_kp * (p_des - p_meas) + self.swing_kd * (v_des - v_meas);
            for k in 0..3 {
                a_swing_des[3 * slot + k] = a[k];
            }
            self.last_foot_body_des[slot] = p_des;
        }

        // ── f_GRF_des: stack MPC GRFs (treat world ≈ body frame) ───
        let mut f_grf_des = na::DVector::zeros(12);
        for slot in 0..4 {
            for k in 0..3 {
                f_grf_des[3 * slot + k] = f_grf_world_des[slot][k];
            }
        }

        // ── Per-actuator torque limits (na) ────────────────────────
        let mut torque_max = na::DVector::from_element(na_count, 100.0);
        for slot in 0..4 {
            for k in 0..3 {
                let ji = joint_indices[slot][k];
                let limit = robot.joints[ji].effort.max(1.0); // 1 N·m floor
                // The WBC's τ vector is in misarta v-index order; map
                // (slot, k) → actuator_idx via the misarta v_idx of the
                // articara joint.
                if let Some(mi) = mc.a2m.get(ji).and_then(|&m| m) {
                    let vi = mc.model.v_idx[mi];
                    if vi >= 6 {
                        let actuator_idx = vi - 6;
                        if actuator_idx < na_count {
                            torque_max[actuator_idx] = limit;
                        }
                    }
                }
            }
        }

        // ── Solve ──────────────────────────────────────────────────
        let inputs = WbcInputs {
            dims,
            mass: &mass,
            nle: &h,
            j_contact: &j_contact,
            dj_v: &dj_v,
            contact_flag,
            friction_mu: self.friction_mu,
            torque_max: &torque_max,
            a_base_des: &a_base_des,
            a_swing_des: &a_swing_des,
            f_grf_des: &f_grf_des,
        };
        let sol = wbc::solve(&inputs);

        // ── Map sol.tau (misarta actuator order) → robot.joints order ─
        let mut robot_taus = vec![0.0_f64; robot.joints.len()];
        for ji in 0..robot.joints.len() {
            let Some(mi) = mc.a2m.get(ji).and_then(|&m| m) else {
                continue;
            };
            if mc.model.joints[mi].joint_type.nv() != 1 {
                continue;
            }
            let vi = mc.model.v_idx[mi];
            if vi < 6 {
                // Floating-base DoF — not actuated.
                continue;
            }
            let actuator_idx = vi - 6;
            if actuator_idx < sol.tau.len() {
                robot_taus[ji] = sol.tau[actuator_idx];
            }
        }
        robot_taus
    }
}
