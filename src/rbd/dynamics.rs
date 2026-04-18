//! Core dynamics algorithms for rigid-body robots.
//!
//! Provides:
//! - **Gravity torque computation**: static torque at each joint due to gravity.
//! - **Descendant-link tree traversal** (used by torque computations).
//! - **CRBA** (Composite Rigid Body Algorithm): joint-space inertia matrix M(q).
//! - **RNEA** (Recursive Newton-Euler Algorithm): inverse dynamics / bias forces.
//! - **Constrained forward dynamics**: solve q̈ with ground-contact constraints.
//! - **Forward dynamics state & integrator**: semi-implicit Euler stepping.
//!
//! # Conventions
//!
//! All 6D spatial vectors use the **body-fixed** convention with ordering
//! `[angular(3); linear(3)]` (Featherstone convention).
//!
//! The robot is assumed to have a **fixed base** (the URDF root link is
//! rigidly attached to the world).  The floating-base extension (6 virtual
//! DOFs) is *not* modelled — instead, the base translation is advanced as
//! a rigid body driven by the net ground reaction force.

use nalgebra as na;
use std::collections::HashMap;

use super::adapter::ModelAdapter;
use super::model::RobotModel;
use super::kinematics::ChainJoint;

// ========== Constants ==========

/// Standard gravitational acceleration (m/s²).
pub const G: f64 = 9.80665;
/// Gravity vector (pointing downward in Z-down convention).
pub const G_VEC: na::Vector3<f64> = na::Vector3::new(0.0, 0.0, -G);

// ========== Result types ==========

/// Per-joint static torque information.
#[derive(Clone, Debug)]
pub struct JointTorqueInfo {
    pub joint_name: String,
    pub joint_idx: usize,
    /// Gravity torque in joint coordinates (N·m for revolute, N for prismatic).
    pub gravity_torque: f64,
    /// Effort (torque/force) limit from URDF.
    pub effort_limit: f64,
    /// `effort_limit - |gravity_torque|`. Negative means the joint is overloaded.
    pub torque_margin: f64,
    /// Additional torque/force per 1 kg of payload at the end-effector.
    /// Only populated when payload analysis is run.
    pub payload_torque_per_kg: f64,
}

// ========== Tree helpers ==========

/// Collect all descendant link indices (inclusive) reachable from `start_link`
/// through the kinematic tree.
pub fn descendant_links(model: &RobotModel, start_link: &str) -> Vec<usize> {
    let mut result = Vec::new();
    let mut stack = vec![start_link.to_string()];
    while let Some(link_name) = stack.pop() {
        if let Some(&li) = model.link_map.get(&link_name) {
            result.push(li);
        }
        if let Some(child_joints) = model.children_joints.get(&link_name) {
            for &ji in child_joints {
                stack.push(model.joints[ji].child_link.clone());
            }
        }
    }
    result
}

/// Return **movable** joint indices in parent-first (topological) order.
///
/// "Movable" means `revolute`, `continuous`, or `prismatic`.
/// The ordering guarantees that a parent joint always appears before its
/// child joints in the returned list.
pub fn topological_joint_order(model: &RobotModel) -> Vec<usize> {
    let mut order = Vec::new();
    let mut stack = vec![model.root_link.clone()];
    while let Some(link) = stack.pop() {
        if let Some(child_joints) = model.children_joints.get(&link) {
            for &ji in child_joints {
                let jt = model.joints[ji].joint_type.as_str();
                if jt == "revolute" || jt == "continuous" || jt == "prismatic" {
                    order.push(ji);
                }
                stack.push(model.joints[ji].child_link.clone());
            }
        }
    }
    order
}

// =========================================================================
//  CRBA — Composite Rigid Body Algorithm
// =========================================================================

/// Compute the joint-space inertia matrix **M(q)** using the Composite
/// Rigid Body Algorithm (CRBA).
///
/// Returns an N×N symmetric positive-definite matrix where N is the number
/// of movable joints (in topological order).
///
/// The `joint_order` parameter must be the output of [`topological_joint_order`].
/// `idx_in_M[joint_idx]` maps a global joint index to its column/row in M
/// (or `None` if the joint is fixed).
///
/// Delegates to [`misarta::crba::crba`] and extracts the submatrix for the
/// requested joint subset.
pub fn crba(
    model: &RobotModel,
    joint_order: &[usize],
) -> (na::DMatrix<f64>, Vec<Option<usize>>) {
    let adapter = ModelAdapter::from_robot_model(model);
    crba_with_adapter(model, joint_order, &adapter)
}

/// Like [`crba`] but reuses a pre-built [`ModelAdapter`] (avoids rebuilding
/// the misarta model on every call).
pub fn crba_with_adapter(
    model: &RobotModel,
    joint_order: &[usize],
    adapter: &ModelAdapter,
) -> (na::DMatrix<f64>, Vec<Option<usize>>) {
    let q = adapter.build_q(model);
    let m_full = misarta::crba::crba(&adapter.model, &q);
    adapter.extract_submatrix(&m_full, joint_order)
}

// =========================================================================
//  RNEA — Recursive Newton-Euler Algorithm (inverse dynamics)
// =========================================================================

/// Compute the **bias force** vector  h(q, q̇) = C(q, q̇)·q̇ + g(q)
/// using the Recursive Newton-Euler Algorithm.
///
/// Returns an N×1 vector (one entry per movable joint in `joint_order`).
///
/// `joint_velocities` maps global joint index → q̇.
///
/// Delegates to [`misarta::rnea::nonlinear_effects`] and extracts the
/// entries for the requested joint subset.
pub fn rnea_bias(
    model: &RobotModel,
    joint_order: &[usize],
    joint_velocities: &HashMap<usize, f64>,
) -> na::DVector<f64> {
    let adapter = ModelAdapter::from_robot_model(model);
    rnea_bias_with_adapter(model, joint_order, joint_velocities, &adapter)
}

/// Like [`rnea_bias`] but reuses a pre-built [`ModelAdapter`].
pub fn rnea_bias_with_adapter(
    model: &RobotModel,
    joint_order: &[usize],
    joint_velocities: &HashMap<usize, f64>,
    adapter: &ModelAdapter,
) -> na::DVector<f64> {
    let q = adapter.build_q(model);
    let v = adapter.build_v(joint_velocities);
    let h_full = misarta::rnea::nonlinear_effects(&adapter.model, &q, v.as_slice());
    let (h_sub, _) = adapter.extract_subvector(&h_full, joint_order);
    h_sub
}

// =========================================================================
//  Forward dynamics (unconstrained)
// =========================================================================

/// Solve **unconstrained** forward dynamics:
///
///   M(q) q̈ = τ − h(q, q̇)
///
/// Returns q̈ as a DVector (one entry per joint in `joint_order`).
pub fn forward_dynamics(
    m_mat: &na::DMatrix<f64>,
    h: &na::DVector<f64>,
    tau: &na::DVector<f64>,
) -> na::DVector<f64> {
    let rhs = tau - h;
    // Solve M q̈ = rhs  via Cholesky (M is SPD)
    match na::linalg::Cholesky::new(m_mat.clone()) {
        Some(chol) => chol.solve(&rhs),
        None => {
            // Fallback: LU decomposition (less efficient, but handles near-singular)
            m_mat.clone().lu().solve(&rhs).unwrap_or_else(|| na::DVector::zeros(h.len()))
        }
    }
}

// =========================================================================
//  Constrained forward dynamics (ground contact)
// =========================================================================

/// Compute the 3×N positional Jacobian **J_foot** for a foot link,
/// mapping joint velocities → foot Cartesian velocity.
///
/// Only joints in `joint_order` are included (mapped via `idx_in_m`).
pub fn foot_jacobian(
    model: &RobotModel,
    foot_chain: &[ChainJoint],
    joint_order: &[usize],
    idx_in_m: &[Option<usize>],
    transforms: &HashMap<String, na::Isometry3<f32>>,
    foot_pos: &na::Point3<f64>,
) -> na::DMatrix<f64> {
    let n = joint_order.len();
    let mut jac = na::DMatrix::zeros(3, n);

    for cj in foot_chain {
        let col = match idx_in_m.get(cj.joint_idx).and_then(|&c| c) {
            Some(c) => c,
            None => continue,
        };

        let joint = &model.joints[cj.joint_idx];
        let parent_tf = transforms
            .get(&joint.parent_link)
            .copied()
            .unwrap_or(na::Isometry3::identity());
        let joint_tf = parent_tf * joint.origin;
        let world_axis = (joint_tf.rotation * joint.axis).cast::<f64>();
        let joint_pos = joint_tf.translation.vector.cast::<f64>();

        let sign: f64 = if cj.inverted { -1.0 } else { 1.0 };

        match joint.joint_type.as_str() {
            "revolute" | "continuous" => {
                let r = foot_pos.coords - joint_pos;
                let j_col = world_axis.cross(&r) * sign;
                jac[(0, col)] = j_col.x;
                jac[(1, col)] = j_col.y;
                jac[(2, col)] = j_col.z;
            }
            "prismatic" => {
                jac[(0, col)] = world_axis.x * sign;
                jac[(1, col)] = world_axis.y * sign;
                jac[(2, col)] = world_axis.z * sign;
            }
            _ => {}
        }
    }

    jac
}

/// Solve **constrained** forward dynamics with ground contact.
///
/// The equations of motion with contact constraints are:
///
///   M q̈ + h = τ + Jᵀ λ
///   J q̈ = −J̇ q̇                     (acceleration-level constraint)
///
/// Rearranging into a KKT system:
///
///   | M  −Jᵀ | | q̈ |   | τ − h        |
///   | J   0  | | λ  | = | −J̇ q̇ (≈ 0) |
///
/// For simplicity we set J̇ q̇ ≈ 0 (valid when dt is small and the foot
/// isn't moving much).
///
/// Returns `(qdd, grf)` where `qdd` is the joint accelerations (N×1) and
/// `grf` is the ground reaction force at each foot (Σ of 3D forces).
pub fn constrained_forward_dynamics(
    m_mat: &na::DMatrix<f64>,
    h: &na::DVector<f64>,
    tau: &na::DVector<f64>,
    j_feet: &[na::DMatrix<f64>],  // one 3×N Jacobian per foot
) -> (na::DVector<f64>, Vec<na::Vector3<f64>>) {
    let n = m_mat.nrows();
    let n_constraints: usize = j_feet.iter().map(|j| j.nrows()).sum();

    if n_constraints == 0 {
        // No contacts — unconstrained
        let qdd = forward_dynamics(m_mat, h, tau);
        return (qdd, Vec::new());
    }

    // Build stacked Jacobian J (n_c × N)
    let mut j_stack = na::DMatrix::zeros(n_constraints, n);
    let mut row = 0;
    for j in j_feet {
        let nr = j.nrows();
        j_stack.view_mut((row, 0), (nr, n)).copy_from(j);
        row += nr;
    }

    // Build KKT system
    let kkt_size = n + n_constraints;
    let mut kkt = na::DMatrix::zeros(kkt_size, kkt_size);
    let mut rhs = na::DVector::zeros(kkt_size);

    // Top-left: M
    kkt.view_mut((0, 0), (n, n)).copy_from(m_mat);
    // Top-right: −Jᵀ
    kkt.view_mut((0, n), (n, n_constraints)).copy_from(&(-j_stack.transpose()));
    // Bottom-left: J
    kkt.view_mut((n, 0), (n_constraints, n)).copy_from(&j_stack);
    // Bottom-right: 0 (already zero) — add small regularisation for stability
    for i in 0..n_constraints {
        kkt[(n + i, n + i)] = -1e-9;
    }

    // RHS
    let tau_minus_h = tau - h;
    rhs.view_mut((0, 0), (n, 1)).copy_from(&tau_minus_h);
    // Lower part = −J̇ q̇ ≈ 0 (already zero)

    // Solve the KKT system
    let solution = kkt.lu().solve(&rhs).unwrap_or_else(|| na::DVector::zeros(kkt_size));

    let qdd = solution.rows(0, n).into_owned();
    let lambda_full = solution.rows(n, n_constraints).into_owned();

    // Split lambda back into per-foot forces
    let mut grfs = Vec::new();
    let mut offset = 0;
    for j in j_feet {
        let nr = j.nrows();
        let lam = lambda_full.rows(offset, nr);
        let f = na::Vector3::new(
            if nr > 0 { lam[0] } else { 0.0 },
            if nr > 1 { lam[1] } else { 0.0 },
            if nr > 2 { lam[2] } else { 0.0 },
        );
        grfs.push(f);
        offset += nr;
    }

    (qdd, grfs)
}

// =========================================================================
//  Joint trajectory profile
// =========================================================================

/// Trajectory shape for a joint.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum TrajectoryProfile {
    /// Symmetric cosine: smooth start **and** stop (zero velocity at both ends).
    ///
    /// $q(t) = q_0 + \frac{\Delta q}{2}(1 - \cos(\frac{\pi t}{T}))$
    Symmetric,

    /// Launch-optimised: smooth start, **maximum velocity at the end**.
    ///
    /// $q(t) = q_0 + \Delta q (1 - \cos(\frac{\pi t}{2T}))$
    ///
    /// At $t = T$ the velocity is $\frac{\pi \Delta q}{2T}$ — ideal for
    /// maximising launch speed in a jump.
    Launch,
}

/// Pre-computed cosine-family trajectory for one joint.
#[derive(Clone, Debug)]
pub struct JointTrajectoryPoint {
    /// Start angle (rad).
    pub q_start: f64,
    /// End angle (rad).
    pub q_end: f64,
    /// Duration of the trajectory (s).
    pub duration: f64,
    /// Trajectory shape.
    pub profile: TrajectoryProfile,
}

impl JointTrajectoryPoint {
    /// Evaluate desired position, velocity, and acceleration at time `t`.
    ///
    /// When `t >= duration` the trajectory is complete: returns
    /// `(q_end, 0, 0)` so the PD controller holds position.
    pub fn evaluate(&self, t: f64) -> (f64, f64, f64) {
        let dq = self.q_end - self.q_start;
        if self.duration <= 0.0 || t >= self.duration {
            return (self.q_end, 0.0, 0.0);
        }
        let t = t.max(0.0);
        match self.profile {
            TrajectoryProfile::Symmetric => {
                // Half-period cosine: zero velocity at both ends.
                //   ω = π / T
                let w = std::f64::consts::PI / self.duration;
                let phase = w * t;
                let q   = self.q_start + 0.5 * dq * (1.0 - phase.cos());
                let qd  = 0.5 * dq * w * phase.sin();
                let qdd = 0.5 * dq * w * w * phase.cos();
                (q, qd, qdd)
            }
            TrajectoryProfile::Launch => {
                // Quarter-period cosine: zero velocity at start,
                // maximum velocity at t = T.
                //   ω = π / (2T)
                let w = std::f64::consts::PI / (2.0 * self.duration);
                let phase = w * t;
                let q   = self.q_start + dq * (1.0 - phase.cos());
                let qd  = dq * w * phase.sin();
                let qdd = dq * w * w * phase.cos();
                (q, qd, qdd)
            }
        }
    }
}

// =========================================================================
//  Forward dynamics state machine
// =========================================================================

/// State for the forward-dynamics integrator.
///
/// Wraps joint velocities, the topological order, and per-foot contact
/// information needed to step the simulation forward in time.
#[derive(Clone, Debug)]
pub struct ForwardDynamicsState {
    /// Movable joint indices in topological order.
    pub joint_order: Vec<usize>,
    /// Current joint velocities (rad/s or m/s), keyed by global joint index.
    pub joint_velocities: HashMap<usize, f64>,
    /// Current base linear velocity (m/s) — for the floating base approximation.
    pub base_velocity: na::Vector3<f64>,
    /// Foot link names that are in ground contact.
    pub contact_feet: Vec<String>,
    /// IK chains from each foot to the body link (for Jacobian computation).
    pub foot_chains: Vec<Vec<ChainJoint>>,
    /// Body link name (trunk).
    pub body_link: String,
    /// Total robot mass (kg).
    pub total_mass: f64,
    /// Initial foot X positions at the start of the simulation,
    /// used by the position-feedback term to correct horizontal drift.
    pub initial_foot_x: Vec<f64>,
    /// Pre-computed joint-space trajectory (joint_idx → trajectory).
    /// When set, `step()` uses PD tracking instead of null-space control.
    pub trajectory: HashMap<usize, JointTrajectoryPoint>,
    /// Cumulative time elapsed since the trajectory started (s).
    pub trajectory_time: f64,
    /// PD position gain (N·m/rad).
    pub kp: f64,
    /// PD derivative gain (N·m·s/rad).
    pub kd: f64,
    /// Cached adapter for misarta Model (avoids rebuilding every step).
    pub adapter: ModelAdapter,
}

impl ForwardDynamicsState {
    /// Create a new state from the current model configuration.
    ///
    /// Only joints that appear in `foot_chains` (i.e. leg joints) **and**
    /// are NOT in `locked_joints` are included in the dynamics.
    /// Arm / head / locked joints are treated as fixed.
    pub fn new(
        model: &RobotModel,
        contact_feet: Vec<String>,
        foot_chains: Vec<Vec<ChainJoint>>,
        body_link: &str,
        locked_joints: &std::collections::HashSet<usize>,
    ) -> Self {
        // Collect the set of leg-joint indices from foot chains,
        // excluding any that are locked.
        let leg_joint_set: std::collections::HashSet<usize> = foot_chains
            .iter()
            .flat_map(|chain| chain.iter().map(|cj| cj.joint_idx))
            .filter(|ji| !locked_joints.contains(ji))
            .collect();

        // Keep only unlocked leg joints, in topological (parent-first) order.
        let all_order = topological_joint_order(model);
        let joint_order: Vec<usize> = all_order
            .into_iter()
            .filter(|ji| leg_joint_set.contains(ji))
            .collect();

        let total_mass: f64 = model.links.iter().map(|l| l.inertial.mass).sum();

        // Record initial foot X positions for drift correction.
        let transforms = model.compute_transforms();
        let initial_foot_x: Vec<f64> = contact_feet.iter().map(|name| {
            transforms.get(name)
                .map(|tf| tf.translation.vector.x as f64)
                .unwrap_or(0.0)
        }).collect();

        let adapter = ModelAdapter::from_robot_model(model);

        Self {
            joint_order,
            joint_velocities: HashMap::new(),
            base_velocity: na::Vector3::zeros(),
            contact_feet,
            foot_chains,
            body_link: body_link.to_string(),
            total_mass,
            initial_foot_x,
            trajectory: HashMap::new(),
            trajectory_time: 0.0,
            kp: 500.0,
            kd: 20.0,
            adapter,
        }
    }

    /// Perform one forward-dynamics integration step.
    ///
    /// When a joint trajectory is set (`self.trajectory` is non-empty),
    /// uses **PD position/velocity tracking** with gravity compensation:
    ///
    ///   $\tau = K_p (q_{des} - q) + K_d (\dot{q}_{des} - \dot{q}) + h$
    ///
    /// Otherwise falls back to the null-space velocity controller.
    ///
    /// `target_angles` is used only in the fallback path (null-space mode).
    ///
    /// This function only updates joint state, NOT `base_transform`.
    /// The caller handles base Z via FK foot-constraint.
    pub fn step(
        &mut self,
        model: &mut RobotModel,
        target_angles: &HashMap<usize, f64>,
        dt: f64,
    ) {
        if dt <= 0.0 || self.joint_order.is_empty() {
            return;
        }

        // Advance trajectory clock
        self.trajectory_time += dt;

        if !self.trajectory.is_empty() {
            self.step_pd(model, dt);
        } else {
            self.step_nullspace(model, target_angles, dt);
        }
    }

    /// Computed-torque (inverse-dynamics + PD) trajectory-tracking step
    /// **with foot-X constraint**.
    ///
    /// 1. Compute the PD-based desired joint acceleration:
    ///    $a_{pd} = \ddot{q}_{des} + K_p(q_{des}-q) + K_d(\dot{q}_{des}-\dot{q})$
    ///
    /// 2. If foot contacts exist, project $a_{pd}$ through the null-space
    ///    of the foot-X Jacobian $J_x$ and add a proportional-derivative
    ///    feedback term that corrects any X-axis drift:
    ///    $a_{cmd} = N \cdot a_{pd} + J_x^+ (-K_{fb} \Delta x - K_{dfb} \dot{x})$
    ///
    /// 3. $\tau = M \cdot a_{cmd} + h$, clamp to effort limits, FD, integrate.
    fn step_pd(&mut self, model: &mut RobotModel, dt: f64) {
        let n = self.joint_order.len();
        let t = self.trajectory_time;

        // --- 1. CRBA: M(q) ---
        let (m_mat, idx_in_m) = crba_with_adapter(model, &self.joint_order, &self.adapter);

        // --- 2. RNEA: h(q, q̇) = C(q,q̇)·q̇ + g(q) ---
        let h = rnea_bias_with_adapter(model, &self.joint_order, &self.joint_velocities, &self.adapter);

        // --- 3. PD acceleration command per joint ---
        let mut a_pd = na::DVector::zeros(n);
        for (col, &ji) in self.joint_order.iter().enumerate() {
            let q_cur = model.joint_positions[ji] as f64;
            let qd_cur = self.joint_velocities.get(&ji).copied().unwrap_or(0.0);

            let (q_des, qd_des, qdd_des) = if let Some(traj) = self.trajectory.get(&ji) {
                traj.evaluate(t)
            } else {
                (q_cur, 0.0, 0.0)
            };

            a_pd[col] = qdd_des
                + self.kp * (q_des - q_cur)
                + self.kd * (qd_des - qd_cur);
        }

        // --- 4. Foot-X null-space projection (when contacts exist) ---
        let a_cmd = if !self.contact_feet.is_empty() && !self.foot_chains.is_empty() {
            let transforms = model.compute_transforms();
            let n_feet = self.contact_feet.len().min(self.foot_chains.len());

            // Build J_x (n_feet × n) and foot drift / velocity
            let mut j_x = na::DMatrix::zeros(n_feet, n);
            let mut foot_dx = na::DVector::zeros(n_feet);
            let mut foot_vx = na::DVector::zeros(n_feet);

            for i in 0..n_feet {
                let foot_name = &self.contact_feet[i];
                let foot_tf = transforms
                    .get(foot_name)
                    .copied()
                    .unwrap_or(na::Isometry3::identity());
                let foot_pos = na::Point3::from(foot_tf.translation.vector.cast::<f64>());

                let jac_3d = foot_jacobian(
                    model,
                    &self.foot_chains[i],
                    &self.joint_order,
                    &idx_in_m,
                    &transforms,
                    &foot_pos,
                );

                // X row (row 0)
                for col in 0..n {
                    j_x[(i, col)] = jac_3d[(0, col)];
                }

                // Position drift
                let x0 = self.initial_foot_x.get(i).copied().unwrap_or(foot_pos.x);
                foot_dx[i] = foot_pos.x - x0;

                // Velocity: ẋ = J_x · q̇
                let mut vx = 0.0;
                for (col, &ji) in self.joint_order.iter().enumerate() {
                    vx += j_x[(i, col)] * self.joint_velocities.get(&ji).copied().unwrap_or(0.0);
                }
                foot_vx[i] = vx;
            }

            // Pseudoinverse: J_x^+ = J_xᵀ (J_x J_xᵀ + εI)⁻¹
            let j_x_t = j_x.transpose();
            let j_x_jxt = &j_x * &j_x_t;
            let eps = 1e-6;
            let mut j_x_jxt_reg = j_x_jxt.clone();
            for i in 0..n_feet {
                j_x_jxt_reg[(i, i)] += eps;
            }

            let j_x_pinv = match j_x_jxt_reg.try_inverse() {
                Some(inv) => &j_x_t * inv,
                None => na::DMatrix::zeros(n, n_feet),
            };

            // Null-space projector: N = I − J_x^+ J_x
            let identity_n = na::DMatrix::identity(n, n);
            let null_proj = &identity_n - &j_x_pinv * &j_x;

            // Foot-X feedback acceleration (PD in Cartesian X):
            //   a_fb = J_x^+ · (−Kfb·Δx − Kdfb·ẋ)
            let k_fb: f64 = 200.0;   // position feedback [1/s²]
            let k_dfb: f64 = 30.0;   // velocity damping  [1/s]
            let foot_accel_cmd = -&foot_dx * k_fb - &foot_vx * k_dfb;
            let a_fb = &j_x_pinv * &foot_accel_cmd;

            // Combined: project PD through null-space + foot correction
            &null_proj * &a_pd + a_fb
        } else {
            // No contacts → pure PD (flight phase)
            a_pd
        };

        // --- 5. Computed torque: τ = M·a_cmd + h ---
        let tau_unclamped = &m_mat * &a_cmd + &h;

        // Clamp to effort limits
        let mut tau = tau_unclamped;
        for (col, &ji) in self.joint_order.iter().enumerate() {
            let effort = model.joints[ji].effort;
            if effort > 0.0 {
                tau[col] = tau[col].clamp(-effort, effort);
            }
        }

        // --- 6. Forward dynamics: q̈ = M⁻¹(τ − h) ---
        let qdd = forward_dynamics(&m_mat, &h, &tau);

        // --- 7. Semi-implicit Euler integration ---
        self.integrate(model, &qdd, dt);
    }

    /// Null-space velocity controller step (legacy path).
    fn step_nullspace(
        &mut self,
        model: &mut RobotModel,
        target_angles: &HashMap<usize, f64>,
        dt: f64,
    ) {
        let n = self.joint_order.len();

        // --- 1. CRBA: M(q) ---
        let (m_mat, idx_in_m) = crba_with_adapter(model, &self.joint_order, &self.adapter);

        // --- 2. RNEA: h(q, q̇) = C(q,q̇)·q̇ + g(q) ---
        let h = rnea_bias_with_adapter(model, &self.joint_order, &self.joint_velocities, &self.adapter);

        // --- 3. Foot X-row Jacobians → J_x (n_feet × n) ---
        let transforms = model.compute_transforms();
        let n_feet = self.contact_feet.len().min(self.foot_chains.len());
        let mut j_x = na::DMatrix::zeros(n_feet, n);
        let mut foot_dx = na::DVector::zeros(n_feet); // X drift

        for i in 0..n_feet {
            let foot_name = &self.contact_feet[i];
            let foot_tf = transforms
                .get(foot_name)
                .copied()
                .unwrap_or(na::Isometry3::identity());
            let foot_pos = na::Point3::from(foot_tf.translation.vector.cast::<f64>());

            let jac_3d = foot_jacobian(
                model,
                &self.foot_chains[i],
                &self.joint_order,
                &idx_in_m,
                &transforms,
                &foot_pos,
            );

            // X row (row 0)
            for col in 0..n {
                j_x[(i, col)] = jac_3d[(0, col)];
            }

            // Foot X drift from initial position
            let x0 = self.initial_foot_x.get(i).copied().unwrap_or(foot_pos.x);
            foot_dx[i] = foot_pos.x - x0;
        }

        // --- 4. Pseudoinverse of J_x and null-space projector ---
        //
        //   J_x^+ = J_xᵀ (J_x J_xᵀ)⁻¹       (right pseudoinverse, m < n)
        //   N = I - J_x^+ J_x                  (null-space projector)
        let j_x_t = j_x.transpose();
        let j_x_jxt = &j_x * &j_x_t; // m × m

        // Regularised inverse: (J_x J_xᵀ + εI)⁻¹
        let eps = 1e-6;
        let mut j_x_jxt_reg = j_x_jxt.clone();
        for i in 0..n_feet {
            j_x_jxt_reg[(i, i)] += eps;
        }

        let j_x_pinv = match j_x_jxt_reg.clone().try_inverse() {
            Some(inv) => &j_x_t * inv,  // n × m
            None => {
                // Fallback: unconstrained (no foot correction)
                na::DMatrix::zeros(n, n_feet)
            }
        };

        let identity_n = na::DMatrix::identity(n, n);
        let null_proj = &identity_n - &j_x_pinv * &j_x; // N: n × n

        // --- 5. Extension velocity ---
        //
        // K_ext: high gain so that effort limits (not the gain) determine
        // the actual motion speed.  Units: [1/s].
        let k_ext: f64 = 30.0;
        let mut qd_ext = na::DVector::zeros(n);
        for (col, &ji) in self.joint_order.iter().enumerate() {
            let q_cur = model.joint_positions[ji] as f64;
            let q_target = target_angles.get(&ji).copied().unwrap_or(q_cur);
            qd_ext[col] = k_ext * (q_target - q_cur);
        }

        // --- 6. Null-space projection ---
        let qd_null = &null_proj * &qd_ext;

        // --- 7. Foot-X position feedback ---
        //
        // K_fb: position feedback gain [1/s].  Must be strong enough to
        // correct drift within a few timesteps.
        let k_fb: f64 = 100.0;
        let qd_fb = &j_x_pinv * &(-foot_dx * k_fb);

        // --- 8. Desired velocity ---
        let qd_des = &qd_null + &qd_fb;

        // --- 9. Inverse-dynamics torque ---
        //
        //   τ = M · (q̇_des − q̇) / dt + h
        //
        // This produces the torque needed to achieve q̇ → q̇_des in one step.
        let mut qd_current = na::DVector::zeros(n);
        for (col, &ji) in self.joint_order.iter().enumerate() {
            qd_current[col] = self.joint_velocities.get(&ji).copied().unwrap_or(0.0);
        }
        let qdd_des = (&qd_des - &qd_current) / dt;
        let tau_unclamped = &m_mat * &qdd_des + &h;

        // --- 10. Clamp to effort limits ---
        let mut tau = tau_unclamped.clone();
        for (col, &ji) in self.joint_order.iter().enumerate() {
            let effort = model.joints[ji].effort;
            if effort > 0.0 {
                tau[col] = tau[col].clamp(-effort, effort);
            }
        }

        // --- 11. Unconstrained forward dynamics: q̈ = M⁻¹(τ − h) ---
        let qdd = forward_dynamics(&m_mat, &h, &tau);

        // --- 12. Semi-implicit Euler integration ---
        self.integrate(model, &qdd, dt);
    }

    /// Semi-implicit Euler integration (shared by PD and null-space paths).
    fn integrate(&mut self, model: &mut RobotModel, qdd: &na::DVector<f64>, dt: f64) {
        for (col, &ji) in self.joint_order.iter().enumerate() {
            let qd = self.joint_velocities.entry(ji).or_insert(0.0);
            *qd += qdd[col] * dt;

            // Clamp velocity to URDF limit
            let vel_limit = model.joints[ji].velocity;
            if vel_limit > 0.0 {
                *qd = qd.clamp(-vel_limit, vel_limit);
            }

            // Integrate position
            let new_q = model.joint_positions[ji] as f64 + *qd * dt;
            let lo = model.joints[ji].lower;
            let hi = model.joints[ji].upper;
            if lo < hi {
                model.joint_positions[ji] = new_q.clamp(lo, hi) as f32;
                if new_q <= lo || new_q >= hi {
                    *qd = 0.0;
                }
            } else {
                model.joint_positions[ji] = new_q as f32;
            }
        }
    }
}

// ========== Gravity torque ==========

/// Compute the gravity torque about a joint axis from a specific set of links.
///
/// `joint_pos` is the joint position in world frame.
/// `world_axis` is the joint axis in world frame.
/// `link_indices` are the links to sum over.
pub fn gravity_torque_from_links(
    model: &RobotModel,
    transforms: &HashMap<String, na::Isometry3<f32>>,
    joint_pos: &na::Vector3<f64>,
    world_axis: &na::Vector3<f64>,
    joint_type: &str,
    link_indices: &[usize],
) -> f64 {
    let mut tau = 0.0_f64;
    for &li in link_indices {
        let link = &model.links[li];
        let mass = link.inertial.mass;
        if mass <= 0.0 {
            continue;
        }
        let link_tf = transforms
            .get(&link.name)
            .copied()
            .unwrap_or(na::Isometry3::identity());
        let com_local = link.inertial.origin.translation.vector.cast::<f64>();
        let com_world = link_tf.cast::<f64>() * na::Point3::from(com_local);
        let r = com_world.coords - joint_pos;
        let f_grav = G_VEC * mass;

        match joint_type {
            "revolute" | "continuous" => {
                tau += world_axis.dot(&r.cross(&f_grav));
            }
            "prismatic" => {
                tau += world_axis.dot(&f_grav);
            }
            _ => {}
        }
    }
    tau
}

/// For joints in a grounded leg, compute the "body-side" gravity torque.
///
/// In a grounded configuration (feet on the floor), each leg joint must support
/// the weight of links on the **body side** (ancestor side), not the foot side.
///
/// body-side torque = total_gravity_torque(all links) − descendant_gravity_torque
///
/// Returns a map: joint_idx → body-side gravity torque.
pub fn compute_body_side_gravity_torques(
    model: &RobotModel,
    joints: &[usize],
) -> HashMap<usize, f64> {
    let transforms = model.compute_transforms();
    let all_link_indices: Vec<usize> = (0..model.links.len()).collect();
    let mut result = HashMap::new();

    for &ji in joints {
        let joint = &model.joints[ji];
        let jt = joint.joint_type.as_str();
        if jt == "fixed" {
            continue;
        }

        let parent_tf = transforms
            .get(&joint.parent_link)
            .copied()
            .unwrap_or(na::Isometry3::identity());
        let joint_tf = parent_tf * joint.origin;
        let joint_pos = joint_tf.translation.vector.cast::<f64>();
        let world_axis = (joint_tf.rotation * joint.axis).cast::<f64>();

        let tau_all = gravity_torque_from_links(
            model, &transforms, &joint_pos, &world_axis, jt, &all_link_indices,
        );

        let descendants = descendant_links(model, &joint.child_link);
        let tau_descendants = gravity_torque_from_links(
            model, &transforms, &joint_pos, &world_axis, jt, &descendants,
        );

        let tau_body_side = tau_all - tau_descendants;
        result.insert(ji, tau_body_side);
    }

    result
}

/// Compute static gravity torque at every movable joint.
///
/// For revolute/continuous joints the result is in N·m; for prismatic joints in N.
///
/// Algorithm: for each joint, sum the gravitational moment contribution of all
/// descendant links (child-side of the joint), projected onto the joint axis.
pub fn compute_gravity_torques(model: &RobotModel) -> Vec<JointTorqueInfo> {
    let transforms = model.compute_transforms();
    let mut result = Vec::new();

    for (ji, joint) in model.joints.iter().enumerate() {
        let jt = joint.joint_type.as_str();
        if jt == "fixed" {
            continue;
        }

        let parent_tf = transforms
            .get(&joint.parent_link)
            .copied()
            .unwrap_or(na::Isometry3::identity());
        let joint_tf = parent_tf * joint.origin;
        let joint_pos = joint_tf.translation.vector.cast::<f64>();
        let world_axis = (joint_tf.rotation * joint.axis).cast::<f64>();

        let descendants = descendant_links(model, &joint.child_link);
        let mut tau = 0.0_f64;

        for &li in &descendants {
            let link = &model.links[li];
            let mass = link.inertial.mass;
            if mass <= 0.0 {
                continue;
            }

            let link_tf = transforms
                .get(&link.name)
                .copied()
                .unwrap_or(na::Isometry3::identity());
            let com_local = link.inertial.origin.translation.vector.cast::<f64>();
            let com_world = link_tf.cast::<f64>() * na::Point3::from(com_local);

            let r = com_world.coords - joint_pos;
            let f_grav = G_VEC * mass;

            match jt {
                "revolute" | "continuous" => {
                    tau += world_axis.dot(&r.cross(&f_grav));
                }
                "prismatic" => {
                    tau += world_axis.dot(&f_grav);
                }
                _ => {}
            }
        }

        result.push(JointTorqueInfo {
            joint_name: joint.name.clone(),
            joint_idx: ji,
            gravity_torque: tau,
            effort_limit: joint.effort,
            torque_margin: joint.effort - tau.abs(),
            payload_torque_per_kg: 0.0,
        });
    }

    result
}
