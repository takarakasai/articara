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

use quadruped_gait::wbc::{
    self, WbcDims, WbcInputs, WbcSolution, WbcWarmStart, WbcWeights,
};
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
    /// Friction coefficient for the contact pyramid (per foot).
    pub friction_mu: f64,
    /// SRBD physical parameters used by [`predicted_base_accel_world`]
    /// to derive the WBC's `a_base_des` from the MPC's GRF prediction.
    /// Mirror these to the host's [`SrbdMpcConfig`] so the WBC
    /// reference is consistent with the MPC's optimisation. Defaults
    /// match `SrbdMpcConfig::default()` (Cheetah-class).
    pub mass_kg: f64,
    pub inertia_diag_body: na::Vector3<f64>,
    /// CoM-aware overrides for `a_base_des`. When `centroidal_inertia_body`
    /// is `Some`, the WBC pipeline computes its base-acceleration
    /// reference via [`quadruped_gait::predicted_base_accel_world_centroidal`]
    /// — using the CoM-shifted moment arm and the centroidal angular
    /// inertia. Hosts mirror these from
    /// [`quadruped_gait::CentroidalMpcConfig`] when the gait controller
    /// runs in `GaitMode::CentroidalSrbd`. Leave `None` to use the
    /// body-root SRBD path (default for `GaitMode::Mpc`).
    pub centroidal_inertia_body: Option<na::Matrix3<f64>>,
    pub com_offset_body: na::Vector3<f64>,

    /// Previous tick's joint q* in URDF sign convention, indexed by
    /// articara joint index. Updated **only for swing legs** so that
    /// during the stance phase the slot retains the swing-period
    /// terminal value — re-using it as `q*_prev` on the next swing
    /// entry produces a small, sane finite-diff q̇* (joint angles
    /// across consecutive swing cycles are close), instead of the
    /// "stance-hold q* → new swing-start q*" jump that a per-leg
    /// always-update would produce.
    last_q_target_urdf: Vec<f64>,

    /// EMA-smoothed `f_grf_des` from the SRBD MPC. The MPC's raw QP
    /// output jitters tick-to-tick (clarabel picks slightly different
    /// optima from a wide null space — observed 13 → 68 → 47 N at
    /// namiashi static stand). Smoothing here, on the **WBC reference**
    /// only, narrows the contact_force task's target without slowing
    /// the τ_ff feedforward used by Position-PD modes — `gait::tick`
    /// returns RAW GRFs for that path, only this WbcPipeline smooths.
    /// Held at zero before the first call; first solve seeds without
    /// blending.
    smoothed_f_grf: [na::Vector3<f64>; 4],
    /// True once `smoothed_f_grf` has been seeded; before that we
    /// initialise to the input verbatim instead of blending into a
    /// zeros vector.
    grf_smoothing_seeded: bool,
    /// EMA blending factor: `smoothed = α·new + (1-α)·prev`. 1.0 =
    /// no smoothing (raw), 0.3 default ≈ 3-solve window at default
    /// 30 ms ZOH.
    pub grf_smoothing_alpha: f64,

    /// Full decision-space solution `x = [q̈; f_GRF; τ]` from the
    /// last [`Self::solve`]. Fed back as the next tick's warm-start
    /// anchor (see `qp_prox_weight`) so the WBC's hierarchical QP
    /// picks consistent solutions inside its wide null space.
    /// Carrying the full `x_prev` (instead of per-level `y_prev`) lets
    /// the inner solver re-project into each tick's per-level basis,
    /// which would otherwise be invalid because the basis is rebuilt
    /// from a `q`-dependent equality matrix every tick.
    qp_x_prev: Option<na::DVector<f64>>,
    /// Per-task LSQ weights forwarded to
    /// [`wbc::solve_warm_with_weights`]. Public so tests can zero
    /// individual entries to isolate each task's contribution
    /// (= the lateral / yaw sign-flip diagnostic in `integration_walk`).
    pub weights: WbcWeights,
    /// Last [`WbcSolution`] returned by [`Self::solve`]. Cached so
    /// diagnostic test rigs can inspect `f_grf` / `q_ddot` / `tau`
    /// breakdowns without rerunning the QP. Populated from `solve()`'s
    /// internal `wbc::solve_warm` result; unchanged during ticks
    /// where the host bypasses `solve()` (e.g. the burn-in window).
    pub last_solution: Option<WbcSolution>,
    /// Proximal regularisation weight passed to
    /// [`misarta::qp::QpConfig::prox_weight`] inside each HoQp level.
    /// 0.0 disables warm-start (cold solve every tick — original
    /// behaviour). `1e-3` is a starting point: small enough to leave
    /// the task residual untouched, large enough to anchor an optimum
    /// to within ~mm/N units across ticks.
    pub qp_prox_weight: f64,
}

impl WbcPipeline {
    /// Test-only: borrow the internal misarta model with the
    /// FreeFlyer base. Used by the dynamics-consistency cross-check
    /// against MuJoCo so the test can call `compute_gravity` on
    /// exactly the same model the WBC sees per tick.
    #[doc(hidden)]
    pub fn model_for_test(&self) -> &Model<f64> {
        &self.model
    }

    /// Test-only: borrow the articara→misarta joint index mapping.
    /// Same use case as [`Self::model_for_test`].
    #[doc(hidden)]
    pub fn a2m_for_test(&self) -> &[Option<usize>] {
        &self.a2m
    }

    pub fn new(robot: &RobotModel, foot_links: [String; 4]) -> Self {
        let (model, a2m, link_to_idx) = build_floating_base_model(robot);

        // Resolve foot link → misarta joint index. The foot's parent
        // joint is `*_foot_fixed`; its child link is the foot link
        // (which lives at the same misarta index as that joint).
        let mut foot_misarta_idx = [None; 4];
        for (slot, link) in foot_links.iter().enumerate() {
            foot_misarta_idx[slot] = link_to_idx.get(link).copied();
        }
        // First-tick velocity is finite-differenced against zero, but
        // the initial p_des_world of an upright body at origin is also
        // close to the nominal, so the resulting "fictitious velocity"
        // is small. We seed at zeros and accept the first-tick bias.
        let last_q_target_urdf = vec![0.0_f64; robot.joints.len()];

        Self {
            foot_links,
            model,
            a2m,
            foot_misarta_idx,
            swing_kp: 80.0,
            swing_kd: 8.0,
            friction_mu: 0.5,
            mass_kg: 9.0,
            inertia_diag_body: na::Vector3::new(0.07, 0.26, 0.242),
            centroidal_inertia_body: None,
            com_offset_body: na::Vector3::zeros(),
            last_q_target_urdf,
            smoothed_f_grf: [na::Vector3::zeros(); 4],
            grf_smoothing_seeded: false,
            grf_smoothing_alpha: 1.0,
            qp_x_prev: None,
            qp_prox_weight: 1e-4,
            weights: WbcWeights::default(),
            last_solution: None,
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
        v_cmd_body: &na::Vector3<f64>,
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

        // ── Sync floating base from MuJoCo body pose ────────────────
        // misarta's FreeFlyer q layout: [px, py, pz, qx, qy, qz, qw].
        // Reading the actual MuJoCo body pose (xpos / xquat) — instead
        // of leaving q at neutral every tick — makes `crba`,
        // `nonlinear_effects`, and `compute_joint_jacobian` reflect the
        // **real** body tilt. Critical for gravity-comp: with
        // identity-quat q, the gravity term in `nle` always points
        // along world −z; if the actual body is tilting and we don't
        // sync, the WBC computes wrong support torques and the trunk
        // collapses.
        let body_pos_w = mj_sim
            .body_world_position(&robot.root_link)
            .map(|p| na::Vector3::new(p[0], p[1], p[2]))
            .unwrap_or_else(na::Vector3::zeros);
        let body_quat = mj_sim
            .body_world_orientation(&robot.root_link)
            .unwrap_or_else(na::UnitQuaternion::identity);
        let r_wb = body_quat.to_rotation_matrix();
        let r_bw = r_wb.transpose();

        let mut q = self.model.neutral_q();
        // FreeFlyer occupies q[0..7]; assume the trunk's misarta idx is 1
        // (which `build_floating_base_model` enforces).
        q[0] = body_pos_w.x;
        q[1] = body_pos_w.y;
        q[2] = body_pos_w.z;
        q[3] = body_quat.i;
        q[4] = body_quat.j;
        q[5] = body_quat.k;
        q[6] = body_quat.w;
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
        // body frame. MuJoCo's `cvel` returns world-frame velocity;
        // we rotate by R_body_world = R_world_body^T to body frame.
        //
        // **Layout: [angular; linear]** (matches Featherstone's
        // spatial-vector convention used by misarta — verified via
        // misarta_mujoco_gravity_consistency test where
        // `compute_gravity[5]` (= linear z) carries the m·g term and
        // `compute_gravity[2]` (= angular z) is zero. MuJoCo uses
        // the opposite [linear; angular] order — historic mistake of
        // mine before that test was added).
        let v_obs_body = r_bw * v_obs_world;
        let omega_obs_body = r_bw * omega_obs_world;
        let mut v = vec![0.0_f64; nv];
        v[0] = omega_obs_body.x;
        v[1] = omega_obs_body.y;
        v[2] = omega_obs_body.z;
        v[3] = v_obs_body.x;
        v[4] = v_obs_body.y;
        v[5] = v_obs_body.z;
        for ji in 0..robot.joints.len() {
            let Some(mi) = self.a2m.get(ji).and_then(|&m| m) else {
                continue;
            };
            if self.model.joints[mi].joint_type.nv() != 1 {
                continue;
            }
            let vi = self.model.v_idx[mi];
            if let Some((_, qd)) = mj_sim.joint_q_qd(&robot.joints[ji].name) {
                // Joint-velocity clip: the no_contact_motion task forms
                // `J_c · q̈ + J̇_c · v = 0`; when MuJoCo's contact solver
                // pushes a joint to abnormally high q̇ (transient
                // bouncing during stance touchdown), `J̇_c · v` becomes
                // large and the WBC has to request a matching big q̈,
                // which then needs a big f_z to satisfy EoM, which
                // bumps the joint via the integrator → positive
                // feedback loop that diverges in <100 ms. Clipping to
                // a reasonable joint speed (5 rad/s ≈ 287 °/s, well
                // above any realistic gait command) breaks the loop
                // without affecting normal operation.
                const JOINT_V_MAX: f64 = 5.0;
                v[vi] = qd.clamp(-JOINT_V_MAX, JOINT_V_MAX);
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
            // misarta's spatial Jacobian rows: [angular(0..3); linear(3..6)].
            // The contact tasks need the *linear* foot velocity, so
            // extract rows 3..6.
            for r in 0..3 {
                for c in 0..nv {
                    j_contact[(3 * slot + r, c)] = j_full[(3 + r, c)];
                }
                dj_v[3 * slot + r] = dj_v_full[3 + r];
            }
        }

        // ── a_base_des from MPC predicted accel (legged_control style) ─
        // legged_control's `formulateBaseAccelTask` derives the base
        // acceleration reference from the OCS2 NMPC's centroidal
        // momentum rate via `A_base⁻¹ · momentum_rate`. We don't have
        // OCS2's centroidal model, but the SRBD reduction is just
        // Newton's laws applied to the MPC's predicted GRFs:
        //   p̈ = (Σf)/m + g
        //   α = I⁻¹ · (Σ(r_i − p_body) × f_i  −  ω × (I·ω))
        // Feeding these directly (instead of a hand-tuned PD on body
        // velocity) makes the WBC track the MPC's own predicted body
        // motion, eliminating MPC-vs-WBC mismatch. Velocity / yaw
        // commands enter the MPC upstream and don't need a parallel
        // PD here.
        //
        // Foot positions in world frame: use the per-leg FK at the
        // current MuJoCo joint state — this is the same expression
        // the swing_leg block below builds for `p_meas_world`.
        let mut foot_pos_world: [na::Vector3<f64>; 4] = [na::Vector3::zeros(); 4];
        for slot in 0..4 {
            let leg_kin = kin.legs()[slot];
            let mut q_leg = [0.0_f64; 3];
            for k in 0..3 {
                let ji = joint_indices[slot][k];
                let sign = joint_signs[slot][k];
                if let Some((q_urdf, _)) = mj_sim.joint_q_qd(&robot.joints[ji].name) {
                    q_leg[k] = sign * q_urdf;
                }
            }
            let p_body = forward_leg_kinematics(leg_kin, q_leg[0], q_leg[1], q_leg[2]);
            foot_pos_world[slot] = body_pos_w + r_wb * p_body;
        }
        // a_base_des dispatch:
        //   * `centroidal_inertia_body` is `Some` ⇒ host signalled
        //     `GaitMode::CentroidalSrbd` — use the CoM-aware moment-arm
        //     formulation. This is what makes the WBC reference and
        //     the centroidal MPC's GRFs self-consistent (the failure
        //     mode that broke the C1/C2 attempts when the SRBD MPC was
        //     paired with body-root WBC reference).
        //   * Otherwise ⇒ body-root SRBD path, exactly as before.
        let (a_lin_world, a_ang_world) = if let Some(i_centroidal) =
            self.centroidal_inertia_body
        {
            let cent_cfg = quadruped_gait::CentroidalMpcConfig {
                mass_kg: self.mass_kg,
                centroidal_inertia_body: i_centroidal,
                com_offset_body: self.com_offset_body,
                ..quadruped_gait::CentroidalMpcConfig::default()
            };
            quadruped_gait::predicted_base_accel_world_centroidal(
                &cent_cfg,
                body_pos_w,
                body_quat,
                *omega_obs_world,
                f_grf_world_des,
                &foot_pos_world,
            )
        } else {
            let srbd_cfg = quadruped_gait::SrbdMpcConfig {
                mass_kg: self.mass_kg,
                inertia_diag_body: self.inertia_diag_body,
                ..quadruped_gait::SrbdMpcConfig::default()
            };
            let euler = body_quat.euler_angles();
            let srbd_state = quadruped_gait::SrbdState {
                orientation_rpy: na::Vector3::new(euler.0, euler.1, euler.2),
                position: body_pos_w,
                angular_velocity: omega_obs_body, // SRBD layout: body frame
                linear_velocity: *v_obs_world,    // SRBD layout: world frame
            };
            quadruped_gait::predicted_base_accel_world(
                &srbd_cfg,
                &srbd_state,
                f_grf_world_des,
                &foot_pos_world,
            )
        };
        // q̈[0..6] is body-frame for a FreeFlyer joint, so rotate the
        // world-frame predicted accels in. Layout: [angular; linear].
        let a_lin_body = r_bw * a_lin_world;
        let a_ang_body = r_bw * a_ang_world;
        // suppress unused-arg warnings during the PD-removal transition;
        // velocity / yaw commands now flow into the MPC layer instead.
        let _ = (v_cmd_body, wz_cmd, omega_obs_world);
        let a_base_des = na::DVector::from_iterator(
            6,
            [
                a_ang_body.x,
                a_ang_body.y,
                a_ang_body.z,
                a_lin_body.x,
                a_lin_body.y,
                a_lin_body.z,
            ],
        );

        // ── Joint-space swing-leg PD reference (legged_control 流) ──
        // Compute `q̈_des = kp·(q* − q) + kd·(q̇* − q̇)` per actuator
        // using the same `q*` that Position-PD tracks (= the gait
        // controller's IK output of the swing trajectory).
        //
        // q̇* is finite-differenced from successive **swing-period**
        // q* values (we skip stance updates so the per-leg slot
        // retains the previous swing's terminal value, giving a sane
        // q̇* on the next swing entry instead of the discontinuous
        // jump that updating during stance would produce).
        //
        // The `JointReference` helper in
        // [`quadruped_gait::mpc_reference`] documents the equivalent
        // legged_control mapping; we keep this loop inline because
        // its swing-only update behaviour is critical for stable
        // q̇* finite-diff and isn't easily expressed in a stateless
        // helper.
        let mut swing_q_ddot_des = na::DVector::zeros(na_count);
        let mut swing_actuator_flag = vec![false; na_count];
        for slot in 0..4 {
            if gait_out.legs[slot].phase.is_stance {
                continue;
            }
            let q_target_ik = [
                gait_out.legs[slot].q_hip,
                gait_out.legs[slot].q_thigh,
                gait_out.legs[slot].q_calf,
            ];
            for k in 0..3 {
                let ji = joint_indices[slot][k];
                let sign = joint_signs[slot][k];
                let q_target_urdf = sign * q_target_ik[k];
                let (q_actual, qd_actual) = mj_sim
                    .joint_q_qd(&robot.joints[ji].name)
                    .unwrap_or((0.0, 0.0));
                let qd_target_urdf = if dt > 1e-6 {
                    (q_target_urdf - self.last_q_target_urdf[ji]) / dt
                } else {
                    qd_actual
                };
                self.last_q_target_urdf[ji] = q_target_urdf;
                let q_ddot = self.swing_kp * (q_target_urdf - q_actual)
                    + self.swing_kd * (qd_target_urdf - qd_actual);
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
                let act_idx = vi - 6;
                if act_idx < na_count {
                    swing_q_ddot_des[act_idx] = q_ddot;
                    swing_actuator_flag[act_idx] = true;
                }
            }
        }

        // ── f_GRF_des: temporal EMA on MPC GRFs ────────────────────
        // The SRBD MPC's raw output jitters tick-to-tick (clarabel
        // picks slightly different optima from the wide null space —
        // observed 13 → 68 → 47 N at namiashi static stand).
        // Smoothing here, on the WBC reference only, avoids slowing
        // the τ_ff feedforward used by Position-PD modes (which
        // consume the raw GRFs via `gc.tick(...)`'s 3rd return).
        let alpha = self.grf_smoothing_alpha.clamp(0.0, 1.0);
        if !self.grf_smoothing_seeded || alpha >= 1.0 {
            self.smoothed_f_grf = *f_grf_world_des;
            self.grf_smoothing_seeded = true;
        } else {
            for slot in 0..4 {
                self.smoothed_f_grf[slot] = alpha * f_grf_world_des[slot]
                    + (1.0 - alpha) * self.smoothed_f_grf[slot];
            }
        }
        let mut f_grf_des = na::DVector::zeros(12);
        for slot in 0..4 {
            for k in 0..3 {
                f_grf_des[3 * slot + k] = self.smoothed_f_grf[slot][k];
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

        // ── τ_gravity: project compute_gravity(q) to actuator rows ──
        // RNEA's gravity-only call gives the static gravity-comp
        // generalised force in `nv`-space; we extract the actuated
        // component (`vi >= 6`) as the WBC's τ ≈ τ_grav anchor at
        // priority 3. Without this anchor the QP can collapse τ → 0
        // (contacts alone balance gravity) and the legs go floppy.
        let g_full = misarta::rnea::compute_gravity(&self.model, &q);
        let mut tau_gravity = na::DVector::zeros(na_count);
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
                tau_gravity[actuator_idx] = g_full[vi];
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
            swing_q_ddot_des: &swing_q_ddot_des,
            swing_actuator_flag: &swing_actuator_flag,
            f_grf_des: &f_grf_des,
            tau_gravity: &tau_gravity,
        };
        // Warm-start: feed back the previous tick's full decision-space
        // solution. Each HoQp level reprojects it into its current null-
        // space basis (`v_target = prev.zᵀ · (x_prev − prev.x)`) and
        // adds a (ρ/2)·‖v − v_target‖² term to the cost. This biases
        // the optimum toward the previous tick's solution — operator-
        // splitting-style warm-start at the cost level — without
        // touching hard constraints (EoM / friction cone / no-contact-
        // motion). Anchors stay valid across ticks even though the
        // basis rotates with `q`.
        let warm = WbcWarmStart {
            x_prev: self.qp_x_prev.as_ref(),
            prox_weight: self.qp_prox_weight,
        };
        let sol = wbc::solve_warm_with_weights(&inputs, &warm, &self.weights);
        // Persist for the next tick.
        self.qp_x_prev = Some(sol.x_full.clone());
        // Cache for diagnostic inspection (test rigs read q_ddot /
        // f_grf / tau directly).
        self.last_solution = Some(sol.clone());

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
pub fn build_floating_base_model(
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
// `pub` so siblings (e.g. estimator::LkfPipeline) can build the same
// floating-base misarta model from a `RobotModel` without duplicating
// the URDF→misarta translation.

pub fn convert_joint_type(joint: &crate::rbd::model::JointData) -> JointType<f64> {
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

pub fn convert_link_inertia(link: &crate::rbd::model::LinkData) -> LinkInertia<f64> {
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
