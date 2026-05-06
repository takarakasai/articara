//! Host wrapper that wires [`LinearKalmanEstimator`] into the
//! application — feeds IMU + joint encoder + per-foot contact data
//! from [`MujocoSim`] / [`RobotModel`] and runs `update` each tick.
//!
//! Mirrors [`crate::wbc_pipeline::WbcPipeline`]'s shape: holds a
//! floating-base misarta `Model` (so foot world positions are
//! well-defined under any joint configuration) and re-exports the
//! decoded body / foot pose for downstream consumers.
//!
//! The host is expected to feed:
//! - IMU world-frame quaternion (typically from
//!   [`crate::attitude_estimator::MadgwickAhrs`]),
//! - IMU body-frame linear acceleration (from `MujocoSim`'s IMU
//!   sensor or a real IMU),
//! - per-foot contact flag (from [`MujocoSim::contact_force_per_foot`]
//!   thresholded; or a real contact sensor).
//!
//! The world-frame foot offsets and velocities are computed inside
//! the pipeline using misarta FK / Jacobian — the host only supplies
//! joint q / q̇ via the standard `RobotModel`+`MujocoSim` pair.

use nalgebra as na;

use misarta::joint::JointType;
use misarta::model::Model;

use super::linear_kalman::{LinearKalmanEstimator, LinearKalmanInputs, LinearKalmanOutput};

#[cfg(feature = "mujoco")]
use crate::mujoco_sim::MujocoSim;
use crate::rbd::model::RobotModel;
use crate::wbc_pipeline::build_floating_base_model;

/// Stateful host wrapper around [`LinearKalmanEstimator`].
#[derive(Clone, Debug)]
pub struct LkfPipeline {
    /// Foot link names in canonical FL/FR/RL/RR slot order.
    pub foot_links: [String; 4],
    /// Floating-base misarta model (built once from `RobotModel`).
    model: Model<f64>,
    /// articara joint index → misarta joint index mapping.
    a2m: Vec<Option<usize>>,
    /// Foot link → misarta joint index.
    foot_misarta_idx: [Option<usize>; 4],
    /// Underlying KF.
    pub kf: LinearKalmanEstimator,
    /// Threshold used by [`Self::update`] to derive a per-foot
    /// contact flag from the supplied force vector. Bumping this above
    /// 5 N filters out micro-contact transients during fast trotting.
    pub contact_force_threshold_n: f64,
}

impl LkfPipeline {
    pub fn new(robot: &RobotModel, foot_links: [String; 4]) -> Self {
        let (model, a2m, link_to_idx) = build_floating_base_model(robot);
        let mut foot_misarta_idx = [None; 4];
        for (slot, link) in foot_links.iter().enumerate() {
            foot_misarta_idx[slot] = link_to_idx.get(link).copied();
        }
        Self {
            foot_links,
            model,
            a2m,
            foot_misarta_idx,
            kf: LinearKalmanEstimator::new(),
            contact_force_threshold_n: 5.0,
        }
    }

    /// Run one estimator tick.
    ///
    /// `body_quat_world` should come from an attitude estimator
    /// (Madgwick) — it sets the misarta floating base's orientation
    /// for the FK / Jacobian calculations. The KF itself estimates
    /// body **position + linear velocity** only; orientation is
    /// passthrough.
    ///
    /// `accel_world` is the IMU's local linear acceleration rotated
    /// into world frame and with gravity subtracted. The host should
    /// compute this once and feed it here.
    ///
    /// `contact_force_z_per_foot` is the per-foot z-contact force
    /// (from [`MujocoSim::contact_force_per_foot`]). Used to derive
    /// per-foot stance flags via `contact_force_threshold_n`.
    pub fn update(
        &mut self,
        robot: &RobotModel,
        body_quat_world: na::UnitQuaternion<f64>,
        accel_world: na::Vector3<f64>,
        joint_q: &[f64],
        joint_qd: &[f64],
        contact_force_z_per_foot: [f64; 4],
        dt: f64,
    ) -> LinearKalmanOutput {
        debug_assert_eq!(joint_q.len(), robot.joints.len());
        debug_assert_eq!(joint_qd.len(), robot.joints.len());
        let nv = self.model.nv;

        // ── Build misarta q with body at origin (KF estimates pos) ─
        // Setting body_pos = (0,0,0) makes `forwardKinematics` return
        // foot positions as **body→foot vectors in world frame**,
        // which is exactly the LKF's `foot_pos_world_offset` input.
        let mut q = self.model.neutral_q();
        q[0] = 0.0;
        q[1] = 0.0;
        q[2] = 0.0;
        q[3] = body_quat_world.i;
        q[4] = body_quat_world.j;
        q[5] = body_quat_world.k;
        q[6] = body_quat_world.w;
        for ji in 0..robot.joints.len() {
            let Some(mi) = self.a2m.get(ji).and_then(|&m| m) else {
                continue;
            };
            if self.model.joints[mi].joint_type.nq() == 1 {
                let qi = self.model.q_idx[mi];
                q[qi] = joint_q[ji];
            }
        }

        // ── Build misarta v with body at rest (KF estimates vel) ──
        // Same reasoning: with v[0..6] = 0 the Jacobian × v gives
        // foot velocities purely from joint motion (= rotated body-
        // relative foot velocity), which is what we want as the
        // observation for stance feet (≈ 0).
        let mut v = vec![0.0_f64; nv];
        for ji in 0..robot.joints.len() {
            let Some(mi) = self.a2m.get(ji).and_then(|&m| m) else {
                continue;
            };
            if self.model.joints[mi].joint_type.nv() != 1 {
                continue;
            }
            let vi = self.model.v_idx[mi];
            v[vi] = joint_qd[ji];
        }

        // ── Per-foot world-frame position offset + velocity ───────
        let mut foot_offset = [na::Vector3::zeros(); 4];
        let mut foot_vel = [na::Vector3::zeros(); 4];
        let v_dvec = na::DVector::from_column_slice(&v);
        for slot in 0..4 {
            let Some(mi) = self.foot_misarta_idx[slot] else {
                continue;
            };
            // FK: forward_kinematics fills `data.oMi[i]` with the
            // world placement of joint `i`. With body at origin, the
            // translation is the body→foot vector in world frame.
            let fk = misarta::fk::forward_kinematics(&self.model, &q);
            let t = misarta::se3::translation(&fk.oMi[mi]);
            foot_offset[slot] = na::Vector3::new(t[0], t[1], t[2]);
            // Foot velocity (world frame) = J_world · v.
            // Use the linear part (rows 3..6 in misarta's
            // [angular; linear] spatial Jacobian).
            let j_full = misarta::jacobian::compute_joint_jacobian(&self.model, &q, mi);
            let jv = &j_full * &v_dvec;
            foot_vel[slot] = na::Vector3::new(jv[3], jv[4], jv[5]);
        }

        // ── Contact flag from force threshold ────────────────────
        let contact_flag: [bool; 4] = std::array::from_fn(|i| {
            contact_force_z_per_foot[i] > self.contact_force_threshold_n
        });

        let inputs = LinearKalmanInputs {
            dt,
            accel_world,
            foot_pos_world_offset: &foot_offset,
            foot_vel_world: &foot_vel,
            contact_flag,
        };
        self.kf.update(&inputs)
    }

    /// Convenience: run [`Self::update`] with inputs sourced directly
    /// from a [`MujocoSim`] (per-foot ground-z force from
    /// `contact_force_per_foot`, joint q / q̇ from `RobotModel`'s
    /// per-joint state, IMU accel and quaternion from the host's
    /// preferred path).
    ///
    /// Available behind the `mujoco` feature only.
    #[cfg(feature = "mujoco")]
    #[allow(clippy::too_many_arguments)]
    pub fn update_from_mujoco(
        &mut self,
        robot: &RobotModel,
        mj_sim: &MujocoSim,
        body_quat_world: na::UnitQuaternion<f64>,
        accel_world: na::Vector3<f64>,
        dt: f64,
    ) -> LinearKalmanOutput {
        let mut joint_q = vec![0.0_f64; robot.joints.len()];
        let mut joint_qd = vec![0.0_f64; robot.joints.len()];
        for (ji, joint) in robot.joints.iter().enumerate() {
            if let Some((q, qd)) = mj_sim.joint_q_qd(&joint.name) {
                joint_q[ji] = q;
                joint_qd[ji] = qd;
            }
        }
        let foot_links: [&str; 4] = [
            self.foot_links[0].as_str(),
            self.foot_links[1].as_str(),
            self.foot_links[2].as_str(),
            self.foot_links[3].as_str(),
        ];
        let force_z = mj_sim.contact_force_per_foot(&foot_links);
        self.update(
            robot,
            body_quat_world,
            accel_world,
            &joint_q,
            &joint_qd,
            force_z,
            dt,
        )
    }
}

// suppress unused-import warning when nothing pulls JointType in
#[allow(dead_code)]
fn _joint_type_used(_: JointType<f64>) {}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::PathBuf;

    fn namiashi_urdf() -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("tests")
            .join("fixtures")
            .join("namiashi")
            .join("urdf")
            .join("namiashi.urdf")
    }

    /// Smoke test: build the pipeline from a real URDF and run one
    /// update with synthetic inputs. Verifies the misarta floating-
    /// base model construction + FK/Jacobian wiring without needing
    /// MuJoCo.
    #[test]
    fn smoke_pipeline_construction_and_one_tick() {
        let path = namiashi_urdf();
        if !path.exists() {
            eprintln!("namiashi fixture missing — skipping smoke test");
            return;
        }
        let robot = RobotModel::from_urdf(&path).unwrap();
        let foot_links = [
            "FL_foot".to_string(),
            "FR_foot".to_string(),
            "RL_foot".to_string(),
            "RR_foot".to_string(),
        ];
        let mut pipeline = LkfPipeline::new(&robot, foot_links);
        // Initialise with body slightly above ground.
        pipeline.kf.reset(
            na::Vector3::new(0.0, 0.0, 0.30),
            &[
                na::Vector3::new(0.18, 0.10, 0.0),
                na::Vector3::new(0.18, -0.10, 0.0),
                na::Vector3::new(-0.18, 0.10, 0.0),
                na::Vector3::new(-0.18, -0.10, 0.0),
            ],
        );
        let q_zero = vec![0.0_f64; robot.joints.len()];
        let qd_zero = vec![0.0_f64; robot.joints.len()];
        let out = pipeline.update(
            &robot,
            na::UnitQuaternion::identity(),
            na::Vector3::zeros(),
            &q_zero,
            &qd_zero,
            [50.0; 4], // all four feet loaded
            0.002,
        );
        // Body z should remain physical (not NaN / huge).
        assert!(out.body_pos_world.z.is_finite());
        assert!(out.body_pos_world.z.abs() < 10.0);
    }
}
