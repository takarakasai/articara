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
//! ## Floating-base model
//!
//! The shared [`crate::rbd::model::MisartaCache`] builds a **fixed-base**
//! misarta model (the trunk attaches directly to universe), which is
//! what the existing IK / gravity-comp / dynamics paths expect. The
//! WBC, however, needs a floating-base topology so the
//! `floating_base_eom` task and the foot world-frame Jacobians make
//! physical sense — a fixed-base trunk in misarta means the feet are
//! pinned by the kinematic tree alone, not by ground contact.
//!
//! [`WbcPipeline::new`] therefore builds its **own** misarta model
//! with a `JointType::FreeFlyer` between universe and the trunk,
//! preserving the rest of the kinematic tree. The base remains at
//! identity orientation each tick (we don't sync the actual MuJoCo
//! body pose into `q[3..7]`), which is fine on flat ground but
//! introduces a small error if the body tilts. Future work: feed
//! the real base pose from MuJoCo / IMU into `q`.

use nalgebra as na;

use misarta::joint::JointType;
use misarta::model::{LinkInertia, Model, ModelBuilder};

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
    /// WBC-specific misarta model with a FreeFlyer at the root.
    model: Model<f64>,
    /// articara joint index → misarta joint index in `self.model`.
    /// Different from `MisartaCache::a2m` because the FreeFlyer
    /// shifts every subsequent index by 1.
    a2m: Vec<Option<usize>>,
    /// Per-leg misarta joint index for the foot frame.
    foot_misarta_idx: [Option<usize>; 4],

    /// Cartesian PD gains for swing legs (units 1/s² and 1/s, applied
    /// to a position / velocity error to produce a Cartesian
    /// acceleration target).
    pub swing_kp: f64,
    pub swing_kd: f64,
    /// Body linear / angular velocity-tracking gain driving the
    /// `base_accel` reference (units 1/s — applied to a velocity
    /// error in world frame).
    pub base_kp_lin: f64,
    pub base_kp_ang: f64,
    /// Friction coefficient for the contact pyramid (per foot).
    pub friction_mu: f64,

    /// Previous tick's body-frame foot-body targets, used to finite-
    /// difference the swing reference velocity. Initialised to the
    /// nominal stance pose so the first tick doesn't see a huge
    /// fictitious velocity.
    last_foot_body_des: [na::Vector3<f64>; 4],
}

impl WbcPipeline {
    pub fn new(robot: &RobotModel, foot_links: [String; 4]) -> Self {
        let (model, a2m, link_to_idx) = build_floating_base_model(robot);

        // Resolve foot link → misarta joint index. The foot's parent
        // joint is `*_foot_fixed`; its child link is the foot link
        // (which lives at the same misarta index as that joint).
        let mut foot_misarta_idx = [None; 4];
        for (slot, link) in foot_links.iter().enumerate() {
            foot_misarta_idx[slot] = link_to_idx.get(link).copied();
        }
        // Initialise last_foot_body_des with each leg's nominal foot
        // body position so the first finite-difference velocity is
        // (nominal − nominal) / dt = 0.
        let last_foot_body_des = [na::Vector3::zeros(); 4];

        Self {
            foot_links,
            model,
            a2m,
            foot_misarta_idx,
            swing_kp: 80.0,
            swing_kd: 8.0,
            base_kp_lin: 30.0,
            base_kp_ang: 30.0,
            friction_mu: 0.5,
            last_foot_body_des,
        }
    }

    /// One tick of the WBC pipeline.
    ///
    /// Returns a per-`robot.joints` torque vector. Entries for fixed
    /// joints stay at 0; entries for movable joints carry the WBC
    /// solution. Call [`MujocoSim::set_wbc_torques`] with this result
    /// (or `clear_wbc_torques` when the pipeline is disabled).
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
        let nv = self.model.nv;
        // Actuated count: total nv minus the 6 base DoFs. Includes any
        // non-leg movable joints (e.g. arm pitch on namiashi); those
        // get τ_GRAV ≈ 0 from the EoM constraint with no swing/stance
        // task so the WBC mostly issues their gravity-comp torque.
        let na_count = nv.saturating_sub(6);

        let dims = WbcDims {
            nv,
            nc: 4,
            na: na_count,
        };

        // ── Build q (FreeFlyer = identity, joints from RobotModel) ──
        let mut q = self.model.neutral_q();
        for ji in 0..robot.joints.len() {
            let Some(mi) = self.a2m.get(ji).and_then(|&m| m) else {
                continue;
            };
            if self.model.joints[mi].joint_type.nq() == 1 {
                let qi = self.model.q_idx[mi];
                q[qi] = robot.joint_positions[ji];
            }
        }

        // ── Build v ─────────────────────────────────────────────────
        // FreeFlyer's motion subspace S = I_6 expresses v[0..6] in the
        // **body** frame. With base at identity orientation this
        // numerically equals world-frame, so we can pass world-frame
        // velocities directly. (When we later sync the actual base
        // orientation, this will need a R_world_body^T rotation.)
        let mut v = vec![0.0_f64; nv];
        v[0] = v_obs_world.x;
        v[1] = v_obs_world.y;
        v[2] = v_obs_world.z;
        v[3] = omega_obs_world.x;
        v[4] = omega_obs_world.y;
        v[5] = omega_obs_world.z;
        for ji in 0..robot.joints.len() {
            let Some(mi) = self.a2m.get(ji).and_then(|&m| m) else {
                continue;
            };
            if self.model.joints[mi].joint_type.nv() != 1 {
                continue;
            }
            let vi = self.model.v_idx[mi];
            if let Some((_, qd)) = mj_sim.joint_q_qd(&robot.joints[ji].name) {
                v[vi] = qd;
            }
        }

        // ── M, h ────────────────────────────────────────────────────
        let mass = misarta::crba::crba(&self.model, &q);
        let h = misarta::rnea::nonlinear_effects(&self.model, &q, &v);

        // ── Per-foot J_linear (3×nv) and dJ·v (3) ──────────────────
        let mut j_contact = na::DMatrix::zeros(12, nv);
        let mut dj_v = na::DVector::zeros(12);
        for slot in 0..4 {
            let Some(mi) = self.foot_misarta_idx[slot] else {
                continue;
            };
            let j_full = misarta::jacobian::compute_joint_jacobian(&self.model, &q, mi);
            let dj_dt = misarta::jacobian::compute_joint_jacobian_time_derivative(
                &self.model,
                &q,
                &v,
                mi,
            );
            let v_dvec = na::DVector::from_column_slice(&v);
            let dj_v_full = dj_dt * v_dvec;
            for r in 0..3 {
                for c in 0..nv {
                    j_contact[(3 * slot + r, c)] = j_full[(r, c)];
                }
                dj_v[3 * slot + r] = dj_v_full[r];
            }
        }

        // ── a_base_des: P-control on body linear + angular velocity ─
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

        // ── a_swing_des per foot (Cartesian PD) ────────────────────
        let mut a_swing_des = na::DVector::zeros(12);
        for slot in 0..4 {
            let p_des = gait_out.legs[slot].foot_body;
            let leg_kin = kin.legs()[slot];
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

        // ── f_GRF_des: stack MPC GRFs ──────────────────────────────
        let mut f_grf_des = na::DVector::zeros(12);
        for slot in 0..4 {
            for k in 0..3 {
                f_grf_des[3 * slot + k] = f_grf_world_des[slot][k];
            }
        }

        // ── Per-actuator torque limits ─────────────────────────────
        // Indexed by the misarta v-index − 6 (which is the τ slot).
        let mut torque_max = na::DVector::from_element(na_count, 100.0);
        for ji in 0..robot.joints.len() {
            let Some(mi) = self.a2m.get(ji).and_then(|&m| m) else {
                continue;
            };
            if self.model.joints[mi].joint_type.nv() != 1 {
                continue;
            }
            let vi = self.model.v_idx[mi];
            if vi < 6 {
                continue;
            }
            let actuator_idx = vi - 6;
            if actuator_idx < na_count {
                torque_max[actuator_idx] = robot.joints[ji].effort.max(1.0);
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

        // ── Map sol.tau → robot.joints order ───────────────────────
        let mut robot_taus = vec![0.0_f64; robot.joints.len()];
        for ji in 0..robot.joints.len() {
            let Some(mi) = self.a2m.get(ji).and_then(|&m| m) else {
                continue;
            };
            if self.model.joints[mi].joint_type.nv() != 1 {
                continue;
            }
            let vi = self.model.v_idx[mi];
            if vi < 6 {
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

/// Build a misarta `Model` with `JointType::FreeFlyer` between universe
/// and the trunk, then BFS through `robot.joints` adding every other
/// joint with parent indices adjusted for the inserted FreeFlyer.
///
/// Returns:
/// - the model,
/// - `a2m`: articara joint index → misarta joint index,
/// - `link_to_idx`: link name → misarta joint index whose child link
///   is that link (used to resolve foot frame indices).
fn build_floating_base_model(
    robot: &RobotModel,
) -> (Model<f64>, Vec<Option<usize>>, std::collections::HashMap<String, usize>) {
    let mut builder = ModelBuilder::<f64>::new()
        .name(robot.name.clone())
        .root_link_name(robot.root_link.clone())
        .gravity(na::Vector3::new(0.0, 0.0, -9.81));

    let trunk_inertia = robot
        .link_map
        .get(&robot.root_link)
        .map(|&li| convert_link_inertia(&robot.links[li]))
        .unwrap_or_else(LinkInertia::zero);

    // Insert the FreeFlyer joint at index 1 (universe is index 0). Its
    // child link is the trunk; its placement is identity (so the
    // body-frame origin coincides with the world origin when q[0..7]
    // is at neutral, which is what we want for the upright body
    // assumption documented at the module level).
    builder = builder.add_joint_with_link(
        "trunk_freejoint",
        0,
        JointType::FreeFlyer,
        misarta::se3::identity(),
        trunk_inertia,
        robot.root_link.clone(),
    );

    let mut link_to_idx: std::collections::HashMap<String, usize> =
        std::collections::HashMap::new();
    link_to_idx.insert(robot.root_link.clone(), 1);

    // BFS over `robot.children_joints` from the root, mirroring the
    // shared `MisartaCache::build` loop — the only difference is that
    // the trunk's misarta index is 1 (not 0), so children attach to
    // 1 instead of 0.
    let mut a2m: Vec<Option<usize>> = vec![None; robot.joints.len()];
    let mut queue: Vec<String> = vec![robot.root_link.clone()];
    while let Some(link_name) = queue.pop() {
        let parent_idx = link_to_idx[&link_name];
        if let Some(child_joints) = robot.children_joints.get(&link_name) {
            for &ji in child_joints {
                let joint = &robot.joints[ji];
                let joint_type = convert_joint_type(joint);
                let placement = joint.origin.cast::<f64>();
                let child_link_name = &joint.child_link;
                let inertia = robot
                    .link_map
                    .get(child_link_name)
                    .map(|&li| convert_link_inertia(&robot.links[li]))
                    .unwrap_or_else(LinkInertia::zero);
                builder = builder.add_joint_with_link(
                    joint.name.clone(),
                    parent_idx,
                    joint_type,
                    placement,
                    inertia,
                    child_link_name.clone(),
                );
                let mi = a2m.iter().filter(|m| m.is_some()).count() + 2; // +1 for FreeFlyer, +1 for universe
                a2m[ji] = Some(mi);
                link_to_idx.insert(child_link_name.clone(), mi);
                queue.push(child_link_name.clone());
            }
        }
    }

    let model = builder.build();
    (model, a2m, link_to_idx)
}

// ─── Inline conversion helpers (mirror rbd::model::convert_*) ──────

fn convert_joint_type(joint: &crate::rbd::model::JointData) -> JointType<f64> {
    let axis = joint.axis.cast::<f64>();
    match joint.joint_type.as_str() {
        "revolute" | "continuous" => JointType::Revolute {
            axis: na::Unit::new_normalize(axis).into_inner(),
        },
        "prismatic" => JointType::Prismatic {
            axis: na::Unit::new_normalize(axis).into_inner(),
        },
        _ => JointType::Fixed,
    }
}

fn convert_link_inertia(link: &crate::rbd::model::LinkData) -> LinkInertia<f64> {
    let i = &link.inertial;
    let com = i.origin.translation.vector.cast::<f64>();
    let rot = i.origin.rotation.to_rotation_matrix();
    let r = rot.matrix().cast::<f64>();
    let i_com = na::Matrix3::new(
        i.ixx, i.ixy, i.ixz, i.ixy, i.iyy, i.iyz, i.ixz, i.iyz, i.izz,
    );
    let rotational_inertia = &r * &i_com * r.transpose();
    LinkInertia {
        mass: i.mass,
        center_of_mass: com,
        rotational_inertia,
    }
}

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

    /// Regression: the WBC model must have a 6-DoF floating base.
    /// The shared `MisartaCache::build` produces a fixed-base model
    /// (`nv = 13` for namiashi), which used to crash the WBC's
    /// `dims.nv == 6 + na_count` assertion. This test pins the
    /// dedicated FreeFlyer-rooted model so a future refactor can't
    /// silently fall back to fixed-base.
    #[test]
    fn namiashi_wbc_model_has_floating_base() {
        let path = namiashi_urdf();
        if !path.exists() {
            eprintln!("namiashi fixture missing — skipping");
            return;
        }
        let robot = RobotModel::from_urdf(&path).unwrap();
        let foot_links = [
            "FL_foot".to_string(),
            "FR_foot".to_string(),
            "RL_foot".to_string(),
            "RR_foot".to_string(),
        ];
        let pipeline = WbcPipeline::new(&robot, foot_links);
        // FreeFlyer (nv=6) + 12 leg joints (nv=1 each) + 1 arm joint = 19.
        assert_eq!(
            pipeline.model.nv, 19,
            "namiashi WBC model must have 19 DoFs (6 base + 12 legs + 1 arm)"
        );
        // Joint 1 must be the FreeFlyer.
        assert!(
            matches!(pipeline.model.joints[1].joint_type, JointType::FreeFlyer),
            "joint 1 must be the FreeFlyer base"
        );
        // All four foot links should resolve to valid misarta indices.
        for slot in 0..4 {
            assert!(
                pipeline.foot_misarta_idx[slot].is_some(),
                "foot {slot} must resolve to a misarta joint index"
            );
        }
    }
}
