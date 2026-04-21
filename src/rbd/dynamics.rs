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

use super::model::{RobotModel, MisartaCache};

// ========== Constants ==========

/// Standard gravitational acceleration (m/s²).
pub const G: f64 = 9.80665;
/// Gravity vector (pointing downward in Z-down convention).
pub const G_VEC: na::Vector3<f64> = na::Vector3::new(0.0, 0.0, -G);

// ========== Tree helpers ==========

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
/// requested joint subset.  Reuses a pre-built [`ModelAdapter`].
pub fn crba(
    model: &RobotModel,
    joint_order: &[usize],
    mc: &MisartaCache,
) -> (na::DMatrix<f64>, Vec<Option<usize>>) {
    let q = mc.build_q(model);
    let m_full = misarta::crba::crba(&mc.model, &q);
    mc.extract_submatrix(&m_full, joint_order)
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
#[allow(dead_code)]
pub fn rnea_bias(
    model: &RobotModel,
    joint_order: &[usize],
    joint_velocities: &HashMap<usize, f64>,
) -> na::DVector<f64> {
    let mc = model.mc();
    rnea_bias_with_mc(model, joint_order, joint_velocities, mc)
}

/// Like [`rnea_bias`] but reuses a pre-built [`MisartaCache`].
pub fn rnea_bias_with_mc(
    model: &RobotModel,
    joint_order: &[usize],
    joint_velocities: &HashMap<usize, f64>,
    mc: &MisartaCache,
) -> na::DVector<f64> {
    let q = mc.build_q(model);
    let v = mc.build_v(joint_velocities);
    let h_full = misarta::rnea::nonlinear_effects(&mc.model, &q, v.as_slice());
    let (h_sub, _) = mc.extract_subvector(&h_full, joint_order);
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
/// Delegates to `ModelAdapter::foot_positional_jacobian` which uses
/// misarta's relative Jacobian internally.
///
/// Only joints in `joint_order` are included (mapped via `idx_in_m`).
pub fn foot_jacobian(
    model: &RobotModel,
    foot_link: &str,
    body_link: &str,
    joint_order: &[usize],
    idx_in_m: &[Option<usize>],
) -> na::DMatrix<f64> {
    model.foot_positional_jacobian(foot_link, body_link, joint_order, idx_in_m)
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
#[allow(dead_code)]
pub struct ForwardDynamicsState {
    /// Movable joint indices in topological order.
    pub joint_order: Vec<usize>,
    /// Current joint velocities (rad/s or m/s), keyed by global joint index.
    pub joint_velocities: HashMap<usize, f64>,
    /// Current base linear velocity (m/s) — for the floating base approximation.
    pub base_velocity: na::Vector3<f64>,
    /// Foot link names that are in ground contact.
    pub contact_feet: Vec<String>,
    /// Joint-index chains from each foot to the body link.
    pub foot_chains: Vec<Vec<usize>>,
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
        foot_chains: Vec<Vec<usize>>,
        body_link: &str,
        locked_joints: &std::collections::HashSet<usize>,
    ) -> Self {
        // Collect the set of leg-joint indices from foot chains,
        // excluding any that are locked.
        let leg_joint_set: std::collections::HashSet<usize> = foot_chains
            .iter()
            .flat_map(|chain| chain.iter().copied())
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
        let mc = model.mc();

        // --- 1. CRBA: M(q) ---
        let (m_mat, idx_in_m) = crba(model, &self.joint_order, mc);

        // --- 2. RNEA: h(q, q̇) = C(q,q̇)·q̇ + g(q) ---
        let h = rnea_bias_with_mc(model, &self.joint_order, &self.joint_velocities, mc);

        // --- 3. PD acceleration command per joint ---
        let mut a_pd = na::DVector::zeros(n);
        for (col, &ji) in self.joint_order.iter().enumerate() {
            let q_cur = model.joint_positions[ji];
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
                    foot_name,
                    &self.body_link,
                    &self.joint_order,
                    &idx_in_m,
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
        let mc = model.mc();

        // --- 1. CRBA: M(q) ---
        let (m_mat, idx_in_m) = crba(model, &self.joint_order, mc);

        // --- 2. RNEA: h(q, q̇) = C(q,q̇)·q̇ + g(q) ---
        let h = rnea_bias_with_mc(model, &self.joint_order, &self.joint_velocities, mc);

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
                foot_name,
                &self.body_link,
                &self.joint_order,
                &idx_in_m,
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
            let q_cur = model.joint_positions[ji];
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
            let new_q = model.joint_positions[ji] + *qd * dt;
            let lo = model.joints[ji].lower;
            let hi = model.joints[ji].upper;
            if lo < hi {
                model.joint_positions[ji] = new_q.clamp(lo, hi);
                if new_q <= lo || new_q >= hi {
                    *qd = 0.0;
                }
            } else {
                model.joint_positions[ji] = new_q;
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
/// Uses misarta's `compute_gravity` for full gravity vector, then computes
/// body-side as (total - descendant) using the per-joint gravity values.
///
/// Returns a map: joint_idx → body-side gravity torque.
pub fn compute_body_side_gravity_torques(
    model: &RobotModel,
    joints: &[usize],
) -> HashMap<usize, f64> {
    let mc = model.mc();
    let q = mc.build_q(model);
    let g_full = misarta::rnea::compute_gravity(&mc.model, &q);

    // Build a lookup: misarta joint idx → gravity torque value
    let g_for = |ji: usize| -> f64 {
        if let Some(mi) = mc.a2m.get(ji).and_then(|&m| m) {
            let nv = mc.model.joints[mi].joint_type.nv();
            if nv == 1 {
                return g_full[mc.model.v_idx[mi]];
            }
        }
        0.0
    };

    // For body-side torque we need: total_gravity - descendant_gravity.
    // descendant_gravity = g(q) for the full model, but restricted to the sub-tree.
    // However, g(q) from RNEA *already* is the descendant torque (it sums moments
    // from all descendant links). So body-side = total_from_all_links - g(q).
    // We compute total_from_all_links using the old method.
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

        let tau_descendants = g_for(ji);
        let tau_body_side = tau_all - tau_descendants;
        result.insert(ji, tau_body_side);
    }

    result
}

// =========================================================================
//  ABA — Articulated Body Algorithm (O(n) forward dynamics)
// =========================================================================

/// Compute forward dynamics via ABA: q̈ = M(q)⁻¹ (τ − C(q,q̇)q̇ − g(q))
///
/// This is O(n) and avoids forming M(q) explicitly.
/// Returns q̈ (one entry per joint in `joint_order`).
#[allow(dead_code)]
pub fn aba_forward_dynamics(
    model: &RobotModel,
    joint_order: &[usize],
    joint_velocities: &HashMap<usize, f64>,
    joint_torques: &HashMap<usize, f64>,
    mc: &MisartaCache,
) -> na::DVector<f64> {
    let q = mc.build_q(model);
    let v = mc.build_v(joint_velocities);
    let mut tau = na::DVector::zeros(mc.model.nv);
    for (&ji, &t) in joint_torques {
        if let Some(mi) = mc.a2m.get(ji).and_then(|&m| m) {
            let nv = mc.model.joints[mi].joint_type.nv();
            if nv == 1 {
                tau[mc.model.v_idx[mi]] = t;
            }
        }
    }
    let qdd_full = misarta::aba::aba(&mc.model, &q, v.as_slice(), tau.as_slice());
    let (qdd_sub, _) = mc.extract_subvector(&qdd_full, joint_order);
    qdd_sub
}

/// Compute M(q)⁻¹ τ using the O(n) ABA without forming M explicitly.
#[allow(dead_code)]
pub fn minv_times_vec(
    model: &RobotModel,
    joint_order: &[usize],
    joint_torques: &HashMap<usize, f64>,
    mc: &MisartaCache,
) -> na::DVector<f64> {
    let q = mc.build_q(model);
    let mut tau = na::DVector::zeros(mc.model.nv);
    for (&ji, &t) in joint_torques {
        if let Some(mi) = mc.a2m.get(ji).and_then(|&m| m) {
            let nv = mc.model.joints[mi].joint_type.nv();
            if nv == 1 {
                tau[mc.model.v_idx[mi]] = t;
            }
        }
    }
    let result = misarta::aba::compute_minv_times_vec(&mc.model, &q, tau.as_slice());
    let (sub, _) = mc.extract_subvector(&result, joint_order);
    sub
}

// =========================================================================
//  Centroidal dynamics — CoM, momentum
// =========================================================================

/// Compute world-frame center of mass position.
#[allow(dead_code)]
pub fn compute_com(model: &RobotModel, mc: &MisartaCache) -> na::Point3<f64> {
    let q = mc.build_q(model);
    let com = misarta::centroidal::compute_com(&mc.model, &q);
    na::Point3::from(com)
}

/// Compute total robot mass via misarta.
#[allow(dead_code)]
pub fn total_mass(mc: &MisartaCache) -> f64 {
    misarta::centroidal::total_mass(&mc.model)
}

/// Compute the CoM Jacobian (3 × nv), mapping generalized velocity to CoM velocity.
#[allow(dead_code)]
pub fn compute_com_jacobian(
    model: &RobotModel,
    joint_order: &[usize],
    mc: &MisartaCache,
) -> na::DMatrix<f64> {
    let q = mc.build_q(model);
    let j_full = misarta::centroidal::compute_com_jacobian(&mc.model, &q);
    let n = joint_order.len();
    let mut j_sub = na::DMatrix::zeros(3, n);
    for (col, &ji) in joint_order.iter().enumerate() {
        if let Some(mi) = mc.a2m.get(ji).and_then(|&m| m) {
            let nv = mc.model.joints[mi].joint_type.nv();
            if nv == 1 {
                let vi = mc.model.v_idx[mi];
                for r in 0..3 {
                    j_sub[(r, col)] = j_full[(r, vi)];
                }
            }
        }
    }
    j_sub
}

/// Compute the 6D centroidal momentum matrix (6 × nv).
#[allow(dead_code)]
pub fn compute_centroidal_momentum_matrix(
    model: &RobotModel,
    joint_order: &[usize],
    mc: &MisartaCache,
) -> na::DMatrix<f64> {
    let q = mc.build_q(model);
    let ag_full = misarta::centroidal::compute_centroidal_momentum_matrix(&mc.model, &q);
    let n = joint_order.len();
    let mut ag_sub = na::DMatrix::zeros(6, n);
    for (col, &ji) in joint_order.iter().enumerate() {
        if let Some(mi) = mc.a2m.get(ji).and_then(|&m| m) {
            let nv = mc.model.joints[mi].joint_type.nv();
            if nv == 1 {
                let vi = mc.model.v_idx[mi];
                for r in 0..6 {
                    ag_sub[(r, col)] = ag_full[(r, vi)];
                }
            }
        }
    }
    ag_sub
}

// =========================================================================
//  iLQR optimal control — re-export from misarta
// =========================================================================

/// Re-export misarta's iLQR types and solver so articara callers
/// can use them through the `rbd::dynamics` namespace.
#[allow(unused_imports)]
pub use misarta::optimization::{
    IlqrConfig, IlqrResult, solve_ilqr,
    discrete_dynamics_step,
};