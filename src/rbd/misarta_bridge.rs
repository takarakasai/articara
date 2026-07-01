//! misarta integration for [`RobotModel`]: cache building, FK / Jacobian /
//! IK solves, and `MisartaConfig` / `.misarta.toml` sidecar conversion.
//!
//! Everything in this module bridges articara's editable [`RobotModel`] to
//! the embedded `misarta::Model<f64>`. It is split out of `model.rs` so the
//! pure data types and tree navigation stay readable on their own; the
//! `impl RobotModel` blocks here extend the same type.

use nalgebra as na;
use na::Matrix3;
use std::collections::HashMap;

use misarta::joint::JointType;
use misarta::model::{LinkInertia, ModelBuilder};
use misarta::geometry::{GeometryModel, GeometryObject, GeometryShape};
use misarta::mesh::MeshData;

use super::model::*;

// =========================================================================
//  Misarta integration: model building, FK, Jacobians, IK
// =========================================================================

impl MisartaCache {
    /// Build the cache from a `RobotModel`.
    pub fn build(robot: &RobotModel) -> Self {
        let mut builder = ModelBuilder::<f64>::new()
            .name(robot.name.clone())
            .root_link_name(robot.root_link.clone());

        builder = builder.gravity(na::Vector3::new(0.0, 0.0, -9.81));

        let mut a2m: Vec<Option<usize>> = vec![None; robot.joints.len()];
        let mut m2a: Vec<Option<usize>> = vec![None]; // index 0 = universe

        let mut link_to_misarta_idx: HashMap<String, usize> = HashMap::new();
        link_to_misarta_idx.insert(robot.root_link.clone(), 0);

        // BFS from root
        let mut queue = vec![robot.root_link.clone()];
        while let Some(link_name) = queue.pop() {
            let parent_misarta_idx = link_to_misarta_idx[&link_name];
            if let Some(child_joint_indices) = robot.children_joints.get(&link_name) {
                for &ji in child_joint_indices {
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
                        parent_misarta_idx,
                        joint_type,
                        placement,
                        inertia,
                        child_link_name.clone(),
                    );

                    let misarta_idx = m2a.len();
                    a2m[ji] = Some(misarta_idx);
                    m2a.push(Some(ji));
                    link_to_misarta_idx.insert(child_link_name.clone(), misarta_idx);
                    queue.push(child_link_name.clone());
                }
            }
        }

        let mut model = builder.build();
        if let Some(&root_li) = robot.link_map.get(&robot.root_link) {
            model.inertias[0] = convert_link_inertia(&robot.links[root_li]);
        }

        Self { model, a2m, m2a }
    }

    /// Build the full q-vector from `RobotModel.joint_positions`.
    pub fn build_q(&self, robot: &RobotModel) -> Vec<f64> {
        let mut q = self.model.neutral_q();
        for (ji, &maybe_mi) in self.a2m.iter().enumerate() {
            if let Some(mi) = maybe_mi {
                let nq = self.model.joints[mi].joint_type.nq();
                if nq == 1 {
                    let qi = self.model.q_idx[mi];
                    q[qi] = robot.joint_positions[ji];
                }
            }
        }
        q
    }

    /// Build the full velocity vector from a sparse map (keyed by articara joint index).
    pub fn build_v(&self, velocities: &HashMap<usize, f64>) -> na::DVector<f64> {
        let mut v = na::DVector::zeros(self.model.nv);
        for (&ji, &qd) in velocities {
            if let Some(mi) = self.a2m.get(ji).and_then(|&m| m) {
                let nv = self.model.joints[mi].joint_type.nv();
                if nv == 1 {
                    let vi = self.model.v_idx[mi];
                    v[vi] = qd;
                }
            }
        }
        v
    }

    /// Map a result vector indexed by misarta v-indices to a vector indexed by `joint_order`.
    pub fn extract_subvector(
        &self,
        full: &na::DVector<f64>,
        joint_order: &[usize],
    ) -> (na::DVector<f64>, Vec<Option<usize>>) {
        let n = joint_order.len();
        let mut sub = na::DVector::zeros(n);
        let mut idx_in_result: Vec<Option<usize>> = vec![None; self.a2m.len()];
        for (col, &ji) in joint_order.iter().enumerate() {
            if let Some(mi) = self.a2m.get(ji).and_then(|&m| m) {
                let nv = self.model.joints[mi].joint_type.nv();
                if nv == 1 {
                    let vi = self.model.v_idx[mi];
                    sub[col] = full[vi];
                }
            }
            idx_in_result[ji] = Some(col);
        }
        (sub, idx_in_result)
    }

    /// Extract a sub-matrix from a full nv×nv matrix for joints in `joint_order`.
    pub fn extract_submatrix(
        &self,
        full: &na::DMatrix<f64>,
        joint_order: &[usize],
    ) -> (na::DMatrix<f64>, Vec<Option<usize>>) {
        let n = joint_order.len();
        let mut sub = na::DMatrix::zeros(n, n);
        let mut idx_in_m: Vec<Option<usize>> = vec![None; self.a2m.len()];

        let mut v_indices: Vec<Option<usize>> = Vec::with_capacity(n);
        for (col, &ji) in joint_order.iter().enumerate() {
            idx_in_m[ji] = Some(col);
            if let Some(mi) = self.a2m.get(ji).and_then(|&m| m) {
                let nv = self.model.joints[mi].joint_type.nv();
                if nv == 1 {
                    v_indices.push(Some(self.model.v_idx[mi]));
                } else {
                    v_indices.push(None);
                }
            } else {
                v_indices.push(None);
            }
        }

        for (r, vi_r) in v_indices.iter().enumerate() {
            for (c, vi_c) in v_indices.iter().enumerate() {
                if let (Some(vr), Some(vc)) = (vi_r, vi_c) {
                    sub[(r, c)] = full[(*vr, *vc)];
                }
            }
        }

        (sub, idx_in_m)
    }

    /// Look up the misarta joint index for a given articara link name.
    pub fn link_name_to_misarta_joint(&self, link_name: &str) -> Option<usize> {
        self.model.link_names.iter().position(|n| n == link_name)
    }

    /// Apply a solved misarta q-vector back to the `RobotModel`.
    #[allow(dead_code)]
    pub fn apply_q_to_robot(&self, robot: &mut RobotModel, q: &[f64]) {
        for (ji, &maybe_mi) in self.a2m.iter().enumerate() {
            if let Some(mi) = maybe_mi {
                let nq = self.model.joints[mi].joint_type.nq();
                if nq == 1 {
                    let qi = self.model.q_idx[mi];
                    robot.joint_positions[ji] = q[qi];
                }
            }
        }
    }
}

impl RobotModel {
    /// Rebuild the cached misarta model from current links/joints.
    ///
    /// Called automatically by constructors. Must be called explicitly
    /// after structural changes (add/remove joints) or serde deserialization.
    pub fn rebuild_misarta_model(&mut self) {
        self.misarta_cache = Some(MisartaCache::build(self));
    }

    /// Get the cached misarta model, or build a temporary one.
    pub(crate) fn mc_or_temp(&self) -> std::borrow::Cow<'_, MisartaCache> {
        match &self.misarta_cache {
            Some(mc) => std::borrow::Cow::Borrowed(mc),
            None => std::borrow::Cow::Owned(MisartaCache::build(self)),
        }
    }

    /// Get the cached misarta model. Panics if not built.
    pub fn mc(&self) -> &MisartaCache {
        self.misarta_cache.as_ref().expect("misarta model not built; call rebuild_misarta_model()")
    }

    /// Build the q-vector from current joint positions.
    pub fn build_q(&self) -> Vec<f64> {
        self.mc().build_q(self)
    }

    /// Build a velocity vector from a sparse map.
    #[allow(dead_code)]
    pub fn build_v(&self, velocities: &HashMap<usize, f64>) -> na::DVector<f64> {
        self.mc().build_v(velocities)
    }

    /// Compute 3×chain_len positional Jacobian for a chain of joint indices.
    ///
    /// When `root_link` is `Some`, uses a relative Jacobian.
    /// When `ee_offset_world` is `Some`, shifts the reference point from the
    /// link frame origin to an arbitrary point offset by that vector (world frame).
    /// Returns an f64 matrix expressed in the **world frame**.
    pub fn chain_positional_jacobian(
        &self,
        chain: &[usize],
        ee_link: &str,
        root_link: Option<&str>,
        ee_offset_world: Option<&na::Vector3<f64>>,
    ) -> na::DMatrix<f64> {
        // We want the Jacobian of `click_point` in world frame *with the
        // IK-root pinned to its initial pose* (the same constraint the GUI
        // re-applies via `base_transform` after each Δq). That isn't the
        // same thing as the unconstrained relative Jacobian
        // `J(ee) − J(base)`; pinning the base means we also need to undo
        // the rigid-body twist that the unconstrained motion imparted at
        // p_base. Working out the algebra for a single revolute joint θ
        // with axis `ω` at world position `p_θ`:
        //
        //   v_constrained(p_click) = v_click − v_base − ω_base × (p_click − p_base)
        //
        // Splitting by where θ sits in the URDF tree (relative to the
        // dragged EE link and the IK-root base link):
        //
        //   • θ upstream of EE only    →  v_constr = +ω × (p_click − p_θ)
        //   • θ upstream of BASE only  →  v_constr = −ω × (p_click − p_θ)
        //   • θ common ancestor        →  v_constr = 0
        //   • θ neither                →  v_constr = 0
        //
        // The earlier implementation built `J_rel = J(ee) − J(base)` from
        // misarta's relative Jacobian and tacked a uniform
        // `ω × (click − p_ee)` lever-arm correction onto every column.
        // That correction is right for the EE-upstream case but produces
        // the wrong column (and wrong sign in many geometries) for
        // BASE-upstream joints — the bug surfaced when picking RL_hip with
        // ik_root=RL_foot, where every chain joint sits in the
        // BASE-upstream branch. The dragged link then moved in the wrong
        // direction. Computing each column from the case table above
        // gives the correct constrained Jacobian and falls back to the
        // standard tip-link Jacobian when ik_root = URDF root.
        let n = chain.len();
        if n == 0 {
            return na::DMatrix::zeros(3, 0);
        }

        let mc = self.mc();
        let q = mc.build_q(self);
        let data = misarta::fk::forward_kinematics(&mc.model, &q);

        let ee_mi = mc.link_name_to_misarta_joint(ee_link).unwrap_or(0);
        let base_mi = root_link
            .and_then(|rl| mc.link_name_to_misarta_joint(rl))
            .unwrap_or(0);

        // Click and base reference points, both in URDF-root frame. The
        // result is rotated to world via `base_transform` at the end.
        let r_base = self.base_transform.rotation.to_rotation_matrix();
        let r_base_inv = r_base.transpose();
        let p_ee_root = if ee_mi > 0 {
            misarta::se3::translation(&data.oMi[ee_mi])
        } else {
            na::Vector3::zeros()
        };
        let click_root: na::Vector3<f64> = match ee_offset_world {
            Some(off_world) => p_ee_root + r_base_inv * off_world,
            None => p_ee_root,
        };
        // (We don't need `p_base_root` separately — both branches operate
        // on `click_root` only; see the case table.)

        // Ancestor sets for fast "is θ on the EE/BASE path" lookups.
        let ee_ancestors = ancestor_set(&mc.model, ee_mi);
        let base_ancestors = ancestor_set(&mc.model, base_mi);

        let mut jac = na::DMatrix::<f64>::zeros(3, n);
        for (col, &ji) in chain.iter().enumerate() {
            let mi = match mc.a2m.get(ji).and_then(|x| *x) {
                Some(m) if m > 0 => m,
                _ => continue,
            };
            let in_ee = ee_ancestors.contains(&mi);
            let in_base = base_ancestors.contains(&mi);
            if !in_ee && !in_base {
                continue;
            }

            // Joint axis (angular subspace col 0) and origin in URDF-root.
            let r_joint = misarta::se3::rotation_matrix(&data.oMi[mi]);
            let p_joint = misarta::se3::translation(&data.oMi[mi]);
            let qi = mc.model.q_idx[mi];
            let nq = mc.model.joints[mi].joint_type.nq();
            let s_local =
                mc.model.joints[mi].joint_type.motion_subspace(&q[qi..qi + nq]);
            let s_ang = na::Vector3::new(
                s_local[(0, 0)],
                s_local[(1, 0)],
                s_local[(2, 0)],
            );
            let omega_root = r_joint * s_ang;

            let mut v_root = na::Vector3::zeros();
            // Both branches use `click_root - p_joint` (NOT `p_base - p_joint`):
            // when θ is upstream of base only, pinning the base imparts a
            // rigid-body correction that, evaluated at p_click, also picks
            // up a `−ω × (p_click − p_base)` term — adding it to the raw
            // `−ω × (p_base − p_θ)` from `J(base)` collapses to the form
            // below (see the case derivation in the doc-comment above).
            if in_ee {
                v_root += omega_root.cross(&(click_root - p_joint));
            }
            if in_base {
                v_root -= omega_root.cross(&(click_root - p_joint));
            }
            let v_world = r_base * v_root;
            jac[(0, col)] = v_world.x;
            jac[(1, col)] = v_world.y;
            jac[(2, col)] = v_world.z;
        }
        jac
    }

    /// Compute 3×n_order positional Jacobian for a foot, remapped to `joint_order`.
    pub fn foot_positional_jacobian(
        &self,
        foot_link: &str,
        body_link: &str,
        joint_order: &[usize],
        idx_in_m: &[Option<usize>],
    ) -> na::DMatrix<f64> {
        let mc = self.mc();
        let q = mc.build_q(self);
        let n = joint_order.len();

        let ee_mi = match mc.link_name_to_misarta_joint(foot_link) {
            Some(v) if v > 0 => v,
            _ => return na::DMatrix::zeros(3, n),
        };
        let base_mi = match mc.link_name_to_misarta_joint(body_link) {
            Some(v) => v,
            None => return na::DMatrix::zeros(3, n),
        };

        let full_jac = if base_mi > 0 {
            misarta::jacobian::compute_relative_jacobian(&mc.model, &q, base_mi, ee_mi)
        } else {
            misarta::jacobian::compute_joint_jacobian(&mc.model, &q, ee_mi)
        };

        // misarta Jacobian is in URDF-root frame; rotate to world frame
        let r = self.base_transform.rotation.to_rotation_matrix();

        let mut jac = na::DMatrix::<f64>::zeros(3, n);
        for &ji in joint_order {
            let col = match idx_in_m.get(ji).and_then(|&c| c) {
                Some(c) => c,
                None => continue,
            };
            if let Some(&Some(mi)) = mc.a2m.get(ji) {
                let vi = mc.model.q_idx[mi];
                let v = na::Vector3::new(
                    full_jac[(3, vi)],
                    full_jac[(4, vi)],
                    full_jac[(5, vi)],
                );
                let v_world = r * v;
                jac[(0, col)] = v_world[0];
                jac[(1, col)] = v_world[1];
                jac[(2, col)] = v_world[2];
            }
        }
        jac
    }

    /// Differential IK step.
    ///
    /// Computes a small joint velocity update using the selected solver.
    /// Delegates to [`misarta::ik::differential_ik_step`] for the core solve,
    /// then optionally adds null-space posture stabilization.
    ///
    /// - `screen_axes`: if `Some((right, up))`, project to 2-DoF screen plane.
    /// - `joint_weights`: if `Some`, per-joint cost weights (one per chain element).
    pub fn solve_ik_step(
        &self,
        chain: &[usize],
        ee_link: &str,
        root_link: Option<&str>,
        ee_pos: &na::Point3<f64>,
        target_pos: &na::Point3<f64>,
        damping: f64,
        gain: f64,
        max_step: f64,
        ref_positions: Option<&[f64]>,
        solver: IkSolver,
        screen_axes: Option<(na::Vector3<f64>, na::Vector3<f64>)>,
        joint_weights: Option<&[f64]>,
        ee_offset_world: Option<&na::Vector3<f64>>,
    ) -> Vec<f64> {
        let n = chain.len();
        if n == 0 {
            return Vec::new();
        }

        // Build 3×n positional Jacobian in world frame
        let jac3 = self.chain_positional_jacobian(chain, ee_link, root_link, ee_offset_world);

        // Map articara IkSolver → misarta types
        let misarta_damping = match solver {
            IkSolver::SrInverse => misarta::ik::Damping::AdaptiveManipulability {
                lambda_min: 0.0,
                lambda_max: damping,
                manipulability_threshold: 0.05,
            },
            _ => misarta::ik::Damping::Fixed(damping),
        };
        let misarta_method = match solver {
            IkSolver::JacobianTranspose => misarta::ik::SolverMethod::JacobianTranspose,
            _ => misarta::ik::SolverMethod::DampedLeastSquares,
        };

        // Build task-space projection for 2-DoF screen plane
        let task_projection = screen_axes.map(|(cam_right, cam_up)| {
            let mut p = na::DMatrix::<f64>::zeros(2, 3);
            p[(0, 0)] = cam_right.x; p[(0, 1)] = cam_right.y; p[(0, 2)] = cam_right.z;
            p[(1, 0)] = cam_up.x;    p[(1, 1)] = cam_up.y;    p[(1, 2)] = cam_up.z;
            p
        });

        // Build misarta JointWeights
        let misarta_weights = joint_weights.map(|w| {
            misarta::ik::JointWeights {
                weights: (0..n).map(|i| if i < w.len() { w[i].max(1e-6) } else { 1.0 }).collect(),
            }
        });

        let diff_config = misarta::ik::DiffIkConfig {
            gain,
            max_joint_step: max_step,
            damping: misarta_damping,
            solver_method: misarta_method,
            joint_weights: misarta_weights.clone(),
            task_projection,
        };

        let ee_v = na::Vector3::new(ee_pos.x, ee_pos.y, ee_pos.z);
        let tgt_v = na::Vector3::new(target_pos.x, target_pos.y, target_pos.z);

        let result = misarta::ik::differential_ik_step(&jac3, &ee_v, &tgt_v, &diff_config);

        // Null-space posture stabilization (computed locally since it needs
        // chain→joint_positions mapping that misarta doesn't have).
        if let Some(ref_pos) = ref_positions {
            // Need pseudo-inverse for null-space projector
            let (jac, m) = if let Some((cam_right, cam_up)) = screen_axes {
                let mut p = na::DMatrix::<f64>::zeros(2, 3);
                p[(0, 0)] = cam_right.x; p[(0, 1)] = cam_right.y; p[(0, 2)] = cam_right.z;
                p[(1, 0)] = cam_up.x;    p[(1, 1)] = cam_up.y;    p[(1, 2)] = cam_up.z;
                (&p * &jac3, 2_usize)
            } else {
                (jac3.clone(), 3_usize)
            };

            // Build W⁻¹ diagonal
            let w_inv: Option<Vec<f64>> = joint_weights.map(|w| {
                (0..n).map(|i| 1.0 / (if i < w.len() { w[i] } else { 1.0 }).max(1e-6)).collect()
            });

            // Weighted JJᵀ
            let jjt = if let Some(ref wi) = w_inv {
                let mut jw = jac.clone();
                for col in 0..n {
                    for row in 0..m {
                        jw[(row, col)] *= wi[col];
                    }
                }
                &jw * jac.transpose()
            } else {
                &jac * jac.transpose()
            };

            let lambda_sq = damping * damping;
            let identity_m = na::DMatrix::<f64>::identity(m, m);
            let jjt_reg = &jjt + &identity_m * lambda_sq;
            if let Some(decomp_result) = jjt_reg.lu().solve(&na::DMatrix::identity(m, m)) {
                let mut j_pinv = jac.transpose() * &decomp_result;
                if let Some(ref wi) = w_inv {
                    for row in 0..n {
                        for col in 0..m {
                            j_pinv[(row, col)] *= wi[row];
                        }
                    }
                }
                let identity_n = na::DMatrix::<f64>::identity(n, n);
                let null_proj = &identity_n - &j_pinv * &jac;

                let k_ns = 0.5;
                let mut dq_posture = na::DVector::<f64>::zeros(n);
                for (i, &ji) in chain.iter().enumerate() {
                    if i < ref_pos.len() {
                        dq_posture[i] = k_ns * (ref_pos[i] - self.joint_positions[ji]);
                    }
                }

                let dq_primary = na::DVector::from_vec(result.dq);
                let dq = &dq_primary + &null_proj * &dq_posture;
                return (0..n).map(|i| dq[i]).collect();
            }
        }

        result.dq
    }

    /// Compute 3×nv positional Jacobian for a link in the **full** joint space
    /// (all model DoFs), expressed in the world frame.
    ///
    /// This is used by the multi-constraint IK solver where constraints span
    /// different kinematic branches and must share a common column space.
    ///
    /// When `ee_offset_world` is `Some`, shifts the reference point from the
    /// link frame origin by that vector (world frame): J_v += J_ω × r.
    pub fn link_positional_jacobian_full(
        &self,
        link_name: &str,
        ee_offset_world: Option<&na::Vector3<f64>>,
    ) -> na::DMatrix<f64> {
        let mc = self.mc();
        let q = mc.build_q(self);
        let nv = mc.model.nv;
        let mi = match mc.link_name_to_misarta_joint(link_name) {
            Some(v) if v > 0 => v,
            _ => return na::DMatrix::zeros(3, nv),
        };
        let full6 = misarta::jacobian::compute_joint_jacobian(&mc.model, &q, mi);
        let r = self.base_transform.rotation.to_rotation_matrix();

        let mut jac = na::DMatrix::<f64>::zeros(3, nv);
        for col in 0..nv {
            let v = na::Vector3::new(full6[(3, col)], full6[(4, col)], full6[(5, col)]);
            let mut v_world = r * v;
            // Apply offset correction: v_click = v_origin + ω × r
            if let Some(offset) = ee_offset_world {
                let omega = na::Vector3::new(full6[(0, col)], full6[(1, col)], full6[(2, col)]);
                let omega_world = r * omega;
                v_world += omega_world.cross(offset);
            }
            jac[(0, col)] = v_world[0];
            jac[(1, col)] = v_world[1];
            jac[(2, col)] = v_world[2];
        }
        jac
    }

    /// Compute 6×nv full (angular + linear) Jacobian for a link in the
    /// **full** joint space, expressed in the world frame.
    ///
    /// Row layout: [ω_x, ω_y, ω_z, v_x, v_y, v_z] (Featherstone ordering).
    pub fn link_full_jacobian_full(&self, link_name: &str) -> na::DMatrix<f64> {
        let mc = self.mc();
        let q = mc.build_q(self);
        let nv = mc.model.nv;
        let mi = match mc.link_name_to_misarta_joint(link_name) {
            Some(v) if v > 0 => v,
            _ => return na::DMatrix::zeros(6, nv),
        };
        let full6 = misarta::jacobian::compute_joint_jacobian(&mc.model, &q, mi);
        let r = self.base_transform.rotation.to_rotation_matrix();

        let mut jac = na::DMatrix::<f64>::zeros(6, nv);
        for col in 0..nv {
            // Angular part (rows 0-2)
            let w = na::Vector3::new(full6[(0, col)], full6[(1, col)], full6[(2, col)]);
            let w_world = r * w;
            jac[(0, col)] = w_world[0];
            jac[(1, col)] = w_world[1];
            jac[(2, col)] = w_world[2];
            // Linear part (rows 3-5)
            let v = na::Vector3::new(full6[(3, col)], full6[(4, col)], full6[(5, col)]);
            let v_world = r * v;
            jac[(3, col)] = v_world[0];
            jac[(4, col)] = v_world[1];
            jac[(5, col)] = v_world[2];
        }
        jac
    }

    /// Differential IK step with pinned-link constraints.
    ///
    /// Like [`solve_ik_step`], but additionally enforces equality constraints
    /// that keep pinned links at their target world positions/orientations
    /// (augmented Jacobian approach via
    /// `misarta::ik::differential_ik_step_with_constraints`).
    ///
    /// Returns deltas for **all model joints** (one per `joint_positions` entry),
    /// not just the chain.
    ///
    /// Each pin specifies link name, target position, optional target
    /// orientation, and whether to use 3-DoF (position) or 6-DoF (pose).
    pub fn solve_ik_step_with_pins(
        &self,
        ee_link: &str,
        ee_pos: &na::Point3<f64>,
        target_pos: &na::Point3<f64>,
        pins: &[PinSpec],
        damping: f64,
        gain: f64,
        max_step: f64,
        solver: IkSolver,
        screen_axes: Option<(na::Vector3<f64>, na::Vector3<f64>)>,
        joint_weights_raw: Option<&[f64]>,
        pin_weight: f64,
        extra_constraints: &[misarta::ik::DiffIkConstraint],
        ee_offset_world: Option<&na::Vector3<f64>>,
    ) -> Vec<f64> {
        let mc = self.mc();
        let nv = mc.model.nv;
        if nv == 0 {
            return Vec::new();
        }

        // Full-nv Jacobian for the primary task (EE)
        let jac_ee = self.link_positional_jacobian_full(ee_link, ee_offset_world);

        // Build constraints for each pinned link
        let transforms = self.compute_transforms();
        let mut constraints = Vec::with_capacity(pins.len());
        for pin in pins {
            let li = self.link_map.get(pin.link_name.as_str()).copied();

            if pin.pose_6dof {
                // 6-DoF constraint (position + orientation)
                let jac6 = self.link_full_jacobian_full(&pin.link_name);

                // Current world pose
                let (pin_pos, pin_rot) = li
                    .map(|idx| {
                        let pos = self.ee_world_pos(idx, &transforms).cast::<f64>();
                        let rot = self.link_world_orientation(idx, &transforms).cast::<f64>();
                        (pos, rot)
                    })
                    .unwrap_or((na::Point3::origin(), na::UnitQuaternion::identity()));

                // Position error (rows 3-5 in Featherstone order)
                let pos_err = pin_pos - pin.target_pos;
                // Orientation error: log(R_cur * R_target^{-1}) → axis-angle 3-vector
                let rot_err_q = pin_rot * pin.target_rot.inverse();
                let rot_err = rot_err_q.scaled_axis();

                // Error vector: [ω_err; v_err] (6D, Featherstone order)
                let err = na::DVector::from_column_slice(&[
                    rot_err.x, rot_err.y, rot_err.z,
                    pos_err.x, pos_err.y, pos_err.z,
                ]);
                constraints.push(misarta::ik::DiffIkConstraint {
                    jacobian: jac6,
                    error: err,
                    weight: pin_weight,
                });
            } else {
                // 3-DoF constraint (position only)
                let jac_pin = self.link_positional_jacobian_full(&pin.link_name, None);
                let pin_world = li
                    .and_then(|idx| {
                        let tf = transforms.get(&self.links[idx].name)?;
                        let (center, _) = self.link_bounding_sphere(idx);
                        Some(*tf * center)
                    })
                    .unwrap_or(na::Point3::origin())
                    .cast::<f64>();

                let err = pin_world - pin.target_pos;
                constraints.push(misarta::ik::DiffIkConstraint {
                    jacobian: jac_pin,
                    error: na::DVector::from_column_slice(&[err.x, err.y, err.z]),
                    weight: pin_weight,
                });
            }
        }

        // Map solver → misarta types
        let misarta_damping = match solver {
            IkSolver::SrInverse => misarta::ik::Damping::AdaptiveManipulability {
                lambda_min: 0.0,
                lambda_max: damping,
                manipulability_threshold: 0.05,
            },
            _ => misarta::ik::Damping::Fixed(damping),
        };
        let misarta_method = match solver {
            IkSolver::JacobianTranspose => misarta::ik::SolverMethod::JacobianTranspose,
            _ => misarta::ik::SolverMethod::DampedLeastSquares,
        };

        // Task projection for 2-DoF screen plane
        let task_projection = screen_axes.map(|(cam_right, cam_up)| {
            let mut p = na::DMatrix::<f64>::zeros(2, 3);
            p[(0, 0)] = cam_right.x; p[(0, 1)] = cam_right.y; p[(0, 2)] = cam_right.z;
            p[(1, 0)] = cam_up.x;    p[(1, 1)] = cam_up.y;    p[(1, 2)] = cam_up.z;
            p
        });

        // Full-nv joint weights
        let misarta_weights = joint_weights_raw.map(|w| {
            misarta::ik::JointWeights {
                weights: (0..nv).map(|i| if i < w.len() { w[i].max(1e-6) } else { 1.0 }).collect(),
            }
        });

        let diff_config = misarta::ik::DiffIkConfig {
            gain,
            max_joint_step: max_step,
            damping: misarta_damping,
            solver_method: misarta_method,
            joint_weights: misarta_weights,
            task_projection,
        };

        let ee_v = na::Vector3::new(ee_pos.x, ee_pos.y, ee_pos.z);
        let tgt_v = na::Vector3::new(target_pos.x, target_pos.y, target_pos.z);

        // Append any extra constraints (e.g. loop closures)
        constraints.extend_from_slice(extra_constraints);

        let result = misarta::ik::differential_ik_step_with_constraints(
            &jac_ee, &ee_v, &tgt_v, &constraints, &diff_config,
        );

        // Map full-nv deltas back to articara joint indices
        let mut deltas = vec![0.0_f64; self.joint_positions.len()];
        for (ji, maybe_mi) in mc.a2m.iter().enumerate() {
            if let Some(mi) = maybe_mi {
                let vi = mc.model.q_idx[*mi];
                if vi < result.dq.len() {
                    deltas[ji] = result.dq[vi];
                }
            }
        }
        deltas
    }

    /// Apply all-joint deltas (one per joint_positions entry), clamping to limits.
    pub fn apply_all_joint_deltas(&mut self, deltas: &[f64]) {
        for (ji, d) in deltas.iter().enumerate() {
            if ji < self.joints.len() && d.abs() > 1e-15 {
                let lower = self.joints[ji].lower;
                let upper = self.joints[ji].upper;
                self.joint_positions[ji] = (self.joint_positions[ji] + d).clamp(lower, upper);
            }
        }
    }

    /// Build collision `GeometryModel` from current model data.
    #[allow(dead_code)]
    pub fn build_collision_geometry(&self) -> GeometryModel {
        self.build_collision_geometry_with_map().0
    }

    /// Build collision `GeometryModel` with a map from geo-obj index → `(link_idx, collision_idx)`.
    #[allow(dead_code)]
    pub fn build_collision_geometry_with_map(&self) -> (GeometryModel, Vec<(usize, usize)>) {
        let mc = self.mc();
        let mut gmodel = GeometryModel::new();
        let mut geo_map: Vec<(usize, usize)> = Vec::new();

        for link in &self.links {
            let parent_joint = mc.link_name_to_misarta_joint(&link.name).unwrap_or(0);
            let li = self.link_map.get(&link.name).copied().unwrap_or(0);

            for (ci, col) in link.collisions.iter().enumerate() {
                let (shape, mesh_data) = match convert_geom_to_shape_with_mesh(&col.geometry) {
                    Some(pair) => pair,
                    None => continue,
                };
                let placement = col.origin.cast::<f64>();
                let mesh_scale = match &col.geometry {
                    GeomData::Mesh { scale, .. } => {
                        scale.map(|s| na::Vector3::new(s[0] as f64, s[1] as f64, s[2] as f64))
                    }
                    _ => None,
                };
                gmodel.add(GeometryObject {
                    name: format!("{}_collision_{}", link.name, ci),
                    parent_joint,
                    placement,
                    shape,
                    mesh_path: None,
                    mesh_scale,
                    mesh_data,
                    material: None,
                });
                geo_map.push((li, ci));
            }
        }

        (gmodel, geo_map)
    }

    /// Apply a solved misarta q-vector back to joint positions.
    #[allow(dead_code)]
    pub fn apply_q(&mut self, q: &[f64]) {
        // Take cache temporarily to avoid borrow conflict
        let mc = self.misarta_cache.take().expect("misarta model not built");
        mc.apply_q_to_robot(self, q);
        self.misarta_cache = Some(mc);
    }

    /// Enforce mimic constraints on current joint positions.
    #[allow(dead_code)]
    pub fn enforce_mimic(&mut self) {
        let mc = self.misarta_cache.as_ref().expect("misarta model not built");
        if mc.model.mimic.is_empty() {
            return;
        }
        let q = mc.build_q(self);
        let q_enforced = misarta::mimic::enforce_mimic(&mc.model, &q);
        // Must clone a2m since we borrow self mutably below
        let a2m = mc.a2m.clone();
        let model_joints = &mc.model.joints;
        let q_idx = &mc.model.q_idx;
        for (ji, &maybe_mi) in a2m.iter().enumerate() {
            if let Some(mi) = maybe_mi {
                let nq = model_joints[mi].joint_type.nq();
                if nq == 1 {
                    self.joint_positions[ji] = q_enforced[q_idx[mi]];
                }
            }
        }
    }
}

// =========================================================================
//  Constraint IK via misarta
// =========================================================================

use misarta::constraint::{
    ConstrainedIkConfig, ConstrainedIkResult, ConstraintModel,
    RigidConstraint,
};
use misarta::frames::Frame;

impl RobotModel {
    /// Build a misarta `Frame` for an articara link name.
    #[allow(dead_code)]
    pub fn frame_for_link(&self, link_name: &str) -> Option<Frame<f64>> {
        let mc = self.mc();
        let mi = mc.link_name_to_misarta_joint(link_name)?;
        Some(Frame {
            name: link_name.to_string(),
            parent_joint: mi,
            placement: misarta::se3::identity(),
        })
    }

    /// Build a misarta `Frame` for an articara link with a local offset.
    pub fn frame_for_link_with_offset(
        &self,
        link_name: &str,
        offset: na::Isometry3<f64>,
    ) -> Option<Frame<f64>> {
        let mc = self.mc();
        let mi = mc.link_name_to_misarta_joint(link_name)?;
        Some(Frame {
            name: link_name.to_string(),
            parent_joint: mi,
            placement: offset,
        })
    }

    /// Create a position-only (3D) constraint between two links.
    #[allow(dead_code)]
    pub fn position_constraint(
        &self,
        link_a: &str,
        link_b: &str,
    ) -> Option<RigidConstraint<f64>> {
        let f1 = self.frame_for_link(link_a)?;
        let f2 = self.frame_for_link(link_b)?;
        Some(RigidConstraint::position(f1, f2))
    }

    /// Create a full-pose (6D) constraint between two links.
    #[allow(dead_code)]
    pub fn pose_constraint(
        &self,
        link_a: &str,
        link_b: &str,
    ) -> Option<RigidConstraint<f64>> {
        let f1 = self.frame_for_link(link_a)?;
        let f2 = self.frame_for_link(link_b)?;
        Some(RigidConstraint::pose(f1, f2))
    }

    /// Solve constrained IK.
    #[allow(dead_code)]
    pub fn solve_constrained_ik(
        &self,
        constraints: Vec<RigidConstraint<f64>>,
        config: &ConstrainedIkConfig,
    ) -> ConstrainedIkResult {
        let mc = self.mc();
        let q0 = mc.build_q(self);
        let cm = ConstraintModel::from_constraints(constraints);
        misarta::constraint::solve_constrained_ik(&mc.model, &q0, &cm, config)
    }

    /// Solve IK with a primary task (position) and rigid constraints.
    #[allow(dead_code)]
    pub fn solve_task_with_constraints(
        &self,
        ee_link: &str,
        target: na::Vector3<f64>,
        constraints: Vec<RigidConstraint<f64>>,
        config: &ConstrainedIkConfig,
    ) -> ConstrainedIkResult {
        let mc = self.mc();
        let q0 = mc.build_q(self);
        let cm = ConstraintModel::from_constraints(constraints);
        let joint_idx = match mc.link_name_to_misarta_joint(ee_link) {
            Some(idx) => idx,
            None => {
                return ConstrainedIkResult {
                    q: q0,
                    iterations: 0,
                    constraint_error_norm: f64::INFINITY,
                    task_error_norm: f64::INFINITY,
                    converged: false,
                };
            }
        };
        misarta::constraint::solve_task_with_constraints(
            &mc.model, &q0, joint_idx, target, &cm, config,
        )
    }

    // ─── Loop closure helpers ─────────────────────────────────────────

    /// Build a [`ConstraintModel`] from this model's stored loop closures.
    pub fn build_loop_constraint_model(&self) -> ConstraintModel<f64> {
        let mut constraints = Vec::with_capacity(self.loop_closures.len());
        for lc in &self.loop_closures {
            let f1 = match self.frame_for_link_with_offset(&lc.link_a, lc.offset_a) {
                Some(f) => f,
                None => continue,
            };
            let f2 = match self.frame_for_link_with_offset(&lc.link_b, lc.offset_b) {
                Some(f) => f,
                None => continue,
            };
            let c = if lc.pose_6dof {
                RigidConstraint::pose(f1, f2).with_name(lc.name.clone())
            } else {
                RigidConstraint::position(f1, f2).with_name(lc.name.clone())
            };
            constraints.push(c);
        }
        ConstraintModel::from_constraints(constraints)
    }

    /// Build [`DiffIkConstraint`]s from stored loop closures at the current
    /// configuration, suitable for single-step differential IK.
    pub fn build_loop_diff_constraints(
        &self,
        weight: f64,
    ) -> Vec<misarta::ik::DiffIkConstraint> {
        let cm = self.build_loop_constraint_model();
        if cm.is_empty() {
            return Vec::new();
        }
        let mc = self.mc();
        let q = mc.build_q(self);
        misarta::constraint::build_diff_ik_constraints(&mc.model, &q, &cm, weight)
    }

    /// Compute the current loop-closure error norm.
    pub fn loop_closure_error(&self) -> f64 {
        let cm = self.build_loop_constraint_model();
        if cm.is_empty() {
            return 0.0;
        }
        let mc = self.mc();
        let q = mc.build_q(self);
        let err = misarta::constraint::compute_constraint_error(&mc.model, &q, &cm);
        err.norm()
    }

    /// Build a `KeyframeAnimation` for the named sequence, suitable for
    /// passing to [`crate::mujoco_sim::MujocoSim::start_sequence`]. The
    /// first keyframe is the *current* joint configuration at time 0;
    /// each subsequent keyframe sits at the cumulative sum of the steps'
    /// `duration` values, with the q-vector for the pose looked up via
    /// [`NamedPose::to_vector`] so renames / missing joints are handled.
    /// Returns `None` if the sequence (or any referenced pose) doesn't
    /// exist.
    pub fn build_sequence_animation(
        &self,
        sequence_name: &str,
    ) -> Option<misarta::trajectory::KeyframeAnimation<f64>> {
        let seq = self.sequences.iter().find(|s| s.name == sequence_name)?;
        let mut keyframes = Vec::with_capacity(seq.steps.len() + 1);
        // Anchor: current joint vector at t=0. Use the model's current
        // joint_positions so the first segment starts where the robot is.
        let mut q_prev = self.joint_positions.clone();
        keyframes.push(misarta::trajectory::Keyframe::new(
            0.0,
            q_prev.clone(),
            misarta::trajectory::InterpolationKind::Linear,
        ));
        let mut t_acc = 0.0;
        for step in &seq.steps {
            let pose = self.poses.iter().find(|p| p.name == step.pose_name)?;
            let q_target = pose.to_vector(self, &q_prev);
            t_acc += step.duration;
            keyframes.push(misarta::trajectory::Keyframe::new(
                t_acc,
                q_target.clone(),
                step.kind,
            ));
            q_prev = q_target;
        }
        Some(misarta::trajectory::KeyframeAnimation::new(keyframes))
    }

    /// Build a `MisartaConfig` from the current loop closures, named poses,
    /// and per-joint actuator settings (mode + Kp + Kv).
    pub fn to_misarta_config(&self) -> misarta::config::MisartaConfig {
        let mut cfg = misarta::config::MisartaConfig::new();
        for lc in &self.loop_closures {
            cfg.loop_closure.push(lc.to_config());
        }
        for p in &self.poses {
            cfg.pose.push(misarta::config::PoseConfig {
                name: p.name.clone(),
                angles: p.angles.clone(),
                duration: p.duration,
                kind: p.kind,
            });
        }
        // Persist actuator settings for every movable joint so re-loading
        // restores the exact controller behaviour. Fixed joints have no
        // actuator and are skipped.
        for j in &self.joints {
            if j.joint_type == "fixed" {
                continue;
            }
            cfg.actuator.push(misarta::config::ActuatorConfig {
                joint_name: j.name.clone(),
                mode: actuator_mode_to_config(j.actuator_mode),
                kp: j.actuator_kp,
                kv: j.actuator_kv,
                armature: j.armature,
                joint_damping: j.joint_damping,
            });
        }
        // Persist per-link-pair collision overrides. Pairs are stored
        // alphabetically so the TOML stays diff-friendly.
        for cp in &self.collision_pairs {
            cfg.collision_pair.push(misarta::config::CollisionPairConfig {
                link_a: cp.link_a.clone(),
                link_b: cp.link_b.clone(),
                enabled: cp.enabled,
            });
        }
        // Persist named sequences.
        for seq in &self.sequences {
            cfg.sequence.push(misarta::config::SequenceConfig {
                name: seq.name.clone(),
                steps: seq
                    .steps
                    .iter()
                    .map(|s| misarta::config::SequenceStepConfig {
                        pose_name: s.pose_name.clone(),
                        duration: s.duration,
                        kind: s.kind,
                    })
                    .collect(),
            });
        }
        // Persist mimics.
        for m in &self.mimics {
            cfg.mimic.push(misarta::config::MimicConfig {
                joint: m.joint.clone(),
                source: m.source.clone(),
                multiplier: m.multiplier,
                offset: m.offset,
            });
        }
        // Persist sensors.
        for s in &self.sensors {
            let q = s.origin.rotation.quaternion();
            cfg.sensor.push(misarta::config::SensorConfig {
                name: s.name.clone(),
                link: s.link.clone(),
                origin: s.origin.translation.vector.into(),
                orientation: [q.i, q.j, q.k, q.w],
                update_rate: s.update_rate,
                kind: sensor_kind_to_config(&s.kind),
            });
        }
        // Persist the home pose — current joint angles + floating-base
        // transform — so reopening the model resumes exactly where the
        // user left off. The map only includes movable joints (fixed
        // joints have no meaningful angle to record); joints not in the
        // map inherit the URDF default at load time.
        for (ji, joint) in self.joints.iter().enumerate() {
            if joint.joint_type == "fixed" {
                continue;
            }
            if let Some(&q) = self.joint_positions.get(ji) {
                cfg.home.joint_positions.insert(joint.name.clone(), q);
            }
        }
        let bp = self.base_transform.translation.vector;
        cfg.home.base_position = [bp.x, bp.y, bp.z];
        let q = self.base_transform.rotation.quaternion();
        cfg.home.base_orientation = [q.i, q.j, q.k, q.w];

        // Persist quadruped gait presets. The leg link lengths and hip
        // offsets are intentionally NOT serialised — they're auto-detected
        // from the URDF chain on every load so an out-of-date sidecar can
        // never silently override the kinematics.
        for g in &self.gaits {
            cfg.gait.push(misarta::config::GaitConfigEntry {
                name: g.name.clone(),
                gait_type: g.gait_type,
                cycle_period_s: g.cycle_period_s,
                duty_factor: g.duty_factor,
                swing_height_m: g.swing_height_m,
                max_step_length_m: g.max_step_length_m,
                fl_foot: g.fl_foot.clone(),
                fr_foot: g.fr_foot.clone(),
                rl_foot: g.rl_foot.clone(),
                rr_foot: g.rr_foot.clone(),
                knee_forward: g.knee_forward,
                four_support_fraction: g.four_support_fraction,
            });
        }
        cfg
    }

    /// Load loop closures, poses, and actuator settings from a
    /// `MisartaConfig`, replacing any existing ones (and updating per-joint
    /// actuator fields by name match).
    pub fn load_misarta_config(&mut self, cfg: &misarta::config::MisartaConfig) {
        self.loop_closures = cfg
            .loop_closure
            .iter()
            .map(LoopClosure::from_config)
            .collect();
        self.poses = cfg
            .pose
            .iter()
            .map(|p| NamedPose {
                name: p.name.clone(),
                angles: p.angles.clone(),
                duration: p.duration,
                kind: p.kind,
            })
            .collect();
        // Restore actuator settings; joints not mentioned in the config keep
        // their current values so partial sidecars don't blow away unrelated
        // tuning.
        for ac in &cfg.actuator {
            if let Some(&ji) = self.joint_map.get(&ac.joint_name) {
                self.joints[ji].actuator_mode =
                    actuator_mode_from_config(ac.mode);
                self.joints[ji].actuator_kp = ac.kp;
                self.joints[ji].actuator_kv = ac.kv;
                self.joints[ji].armature = ac.armature;
                self.joints[ji].joint_damping = ac.joint_damping;
            }
        }
        // Restore collision pair overrides. We keep entries even when the
        // referenced links are missing (the user might be mid-rename) so
        // round-tripping doesn't silently drop them.
        self.collision_pairs = cfg
            .collision_pair
            .iter()
            .map(|cp| CollisionPair::new(cp.link_a.clone(), cp.link_b.clone(), cp.enabled))
            .collect();
        // Restore sequences.
        self.sequences = cfg
            .sequence
            .iter()
            .map(|sc| Sequence {
                name: sc.name.clone(),
                steps: sc
                    .steps
                    .iter()
                    .map(|s| SequenceStep {
                        pose_name: s.pose_name.clone(),
                        duration: s.duration,
                        kind: s.kind,
                    })
                    .collect(),
            })
            .collect();
        // Restore mimics.
        self.mimics = cfg
            .mimic
            .iter()
            .map(|m| Mimic {
                joint: m.joint.clone(),
                source: m.source.clone(),
                multiplier: m.multiplier,
                offset: m.offset,
            })
            .collect();
        // Restore sensors.
        self.sensors = cfg
            .sensor
            .iter()
            .map(|s| Sensor {
                name: s.name.clone(),
                link: s.link.clone(),
                origin: na::Isometry3::from_parts(
                    na::Translation3::new(s.origin[0], s.origin[1], s.origin[2]),
                    na::UnitQuaternion::from_quaternion(na::Quaternion::new(
                        s.orientation[3],
                        s.orientation[0],
                        s.orientation[1],
                        s.orientation[2],
                    )),
                ),
                update_rate: s.update_rate,
                kind: sensor_kind_from_config(&s.kind),
            })
            .collect();
        // Restore home pose. Only joints listed in the map are touched;
        // unlisted joints keep whatever value the constructor (URDF /
        // SDF / etc.) seeded. Skip the base_transform write when the
        // sidecar carries the all-default identity since most newly
        // imported URDFs already have the root at the origin and
        // overwriting with identity would be a noisy no-op.
        if !cfg.home.joint_positions.is_empty() {
            for (name, q) in &cfg.home.joint_positions {
                if let Some(&ji) = self.joint_map.get(name) {
                    if ji < self.joint_positions.len() {
                        self.joint_positions[ji] = *q;
                    }
                }
            }
        }
        let bp = cfg.home.base_position;
        let bq = cfg.home.base_orientation;
        let identity_pos = bp == [0.0, 0.0, 0.0];
        let identity_quat = bq == [0.0, 0.0, 0.0, 1.0];
        if !(identity_pos && identity_quat) {
            self.base_transform = na::Isometry3::from_parts(
                na::Translation3::new(bp[0], bp[1], bp[2]),
                na::UnitQuaternion::from_quaternion(na::Quaternion::new(
                    bq[3], bq[0], bq[1], bq[2],
                )),
            );
        }

        // Restore quadruped gait presets.
        self.gaits = cfg
            .gait
            .iter()
            .map(|g| GaitDescriptor {
                name: g.name.clone(),
                gait_type: g.gait_type,
                cycle_period_s: g.cycle_period_s,
                duty_factor: g.duty_factor,
                swing_height_m: g.swing_height_m,
                max_step_length_m: g.max_step_length_m,
                fl_foot: g.fl_foot.clone(),
                fr_foot: g.fr_foot.clone(),
                rl_foot: g.rl_foot.clone(),
                rr_foot: g.rr_foot.clone(),
                knee_forward: g.knee_forward,
                four_support_fraction: g.four_support_fraction,
            })
            .collect();
    }

    /// Try to load the `.misarta.toml` sidecar file next to `source_path`.
    /// Returns `Some(SidecarLoadReport)` when a config was found, parsed, and
    /// applied; `None` when no sidecar exists.
    ///
    /// **Legacy path.** New work should use the `.misa` master format
    /// (which subsumes everything the sidecar carries plus the kinematic
    /// tree itself). This loader stays in place so users with existing
    /// URDF + `.misarta.toml` workflows aren't broken; it will be
    /// deprecated once those pairs have all been converted to `.misa`.
    pub fn load_sidecar_config(&mut self) -> Option<SidecarLoadReport> {
        let src = self.source_path.as_ref()?.clone();
        let toml_path = misarta::config::MisartaConfig::config_path_for(&src);
        if !toml_path.exists() {
            return None;
        }
        match misarta::config::MisartaConfig::load(&toml_path) {
            Ok(cfg) => {
                // Track which actuator entries failed to match a joint name —
                // those silently dropped before, which made it look like the
                // sidecar load did nothing when actually only the lookup failed.
                let mut applied = Vec::new();
                let mut unmatched = Vec::new();
                for ac in &cfg.actuator {
                    if self.joint_map.contains_key(&ac.joint_name) {
                        applied.push(ac.joint_name.clone());
                    } else {
                        unmatched.push(ac.joint_name.clone());
                    }
                }
                self.load_misarta_config(&cfg);
                log::info!(
                    "Loaded {} loop closure(s), {} pose(s), {}/{} actuator setting(s) from {}",
                    self.loop_closures.len(),
                    self.poses.len(),
                    applied.len(),
                    cfg.actuator.len(),
                    toml_path.display()
                );
                if !unmatched.is_empty() {
                    log::warn!(
                        "{} actuator entry(ies) skipped (joint not found in model): {}",
                        unmatched.len(),
                        unmatched.join(", ")
                    );
                }
                Some(SidecarLoadReport {
                    path: toml_path,
                    n_loop_closures: self.loop_closures.len(),
                    n_poses: self.poses.len(),
                    n_actuators_applied: applied.len(),
                    n_actuators_total: cfg.actuator.len(),
                    unmatched_actuators: unmatched,
                })
            }
            Err(e) => {
                log::warn!("Failed to load {}: {}", toml_path.display(), e);
                None
            }
        }
    }

    /// Save loop closures and other articara-specific metadata to a
    /// `.misarta.toml` sidecar next to `model_path`.
    ///
    /// If the configuration would be empty the file is NOT written, and
    /// any existing one is left untouched.
    ///
    /// **Legacy path.** Prefer [`RobotModel::save_as_misa`] — `.misa`
    /// carries every field this sidecar holds plus the full kinematic
    /// tree, so the model can round-trip from a single file. This
    /// sidecar saver remains for URDF-centric workflows that still need
    /// to interop with ROS-style consumers reading the `.urdf`.
    pub fn save_sidecar_config(&self, model_path: &std::path::Path) -> Result<(), String> {
        let cfg = self.to_misarta_config();
        if cfg.is_empty() {
            return Ok(());
        }
        let toml_path = misarta::config::MisartaConfig::config_path_for(model_path);
        cfg.save(&toml_path)
    }
}

/// Summary of what [`RobotModel::load_sidecar_config`] applied. The UI surfaces
/// this in the status bar so the user can confirm at a glance how many
/// actuator entries actually reached `JointData` (and which were silently
/// skipped because the joint name didn't match the model).
#[derive(Debug, Clone)]
pub struct SidecarLoadReport {
    pub path: std::path::PathBuf,
    pub n_loop_closures: usize,
    pub n_poses: usize,
    pub n_actuators_applied: usize,
    pub n_actuators_total: usize,
    pub unmatched_actuators: Vec<String>,
}

/// 1:1 conversion between in-memory [`SensorKind`] and the
/// serialisation-friendly [`misarta::config::SensorKind`].
fn sensor_kind_to_config(k: &SensorKind) -> misarta::config::SensorKind {
    match k {
        SensorKind::Camera { fov, width, height, near, far } => {
            misarta::config::SensorKind::Camera {
                fov: *fov, width: *width, height: *height, near: *near, far: *far,
            }
        }
        SensorKind::Lidar {
            range_min, range_max, h_fov, h_samples, v_fov, v_samples,
        } => misarta::config::SensorKind::Lidar {
            range_min: *range_min,
            range_max: *range_max,
            h_fov: *h_fov,
            h_samples: *h_samples,
            v_fov: *v_fov,
            v_samples: *v_samples,
        },
        SensorKind::Imu { gyro_noise, accel_noise } => {
            misarta::config::SensorKind::Imu {
                gyro_noise: *gyro_noise,
                accel_noise: *accel_noise,
            }
        }
        SensorKind::ForceTorque { joint } => {
            misarta::config::SensorKind::ForceTorque { joint: joint.clone() }
        }
        SensorKind::Contact { partner } => {
            misarta::config::SensorKind::Contact { partner: partner.clone() }
        }
        SensorKind::Generic { kind, params } => {
            misarta::config::SensorKind::Generic {
                kind: kind.clone(),
                params: params.clone(),
            }
        }
    }
}

fn sensor_kind_from_config(k: &misarta::config::SensorKind) -> SensorKind {
    match k {
        misarta::config::SensorKind::Camera { fov, width, height, near, far } => {
            SensorKind::Camera {
                fov: *fov, width: *width, height: *height, near: *near, far: *far,
            }
        }
        misarta::config::SensorKind::Lidar {
            range_min, range_max, h_fov, h_samples, v_fov, v_samples,
        } => SensorKind::Lidar {
            range_min: *range_min,
            range_max: *range_max,
            h_fov: *h_fov,
            h_samples: *h_samples,
            v_fov: *v_fov,
            v_samples: *v_samples,
        },
        misarta::config::SensorKind::Imu { gyro_noise, accel_noise } => {
            SensorKind::Imu {
                gyro_noise: *gyro_noise,
                accel_noise: *accel_noise,
            }
        }
        misarta::config::SensorKind::ForceTorque { joint } => {
            SensorKind::ForceTorque { joint: joint.clone() }
        }
        misarta::config::SensorKind::Contact { partner } => {
            SensorKind::Contact { partner: partner.clone() }
        }
        misarta::config::SensorKind::Generic { kind, params } => {
            SensorKind::Generic {
                kind: kind.clone(),
                params: params.clone(),
            }
        }
    }
}

/// Walk misarta's joint tree from `start` toward joint 0 (the URDF root)
/// collecting every joint encountered into a set. Returns an empty set when
/// `start` is 0. Used by [`RobotModel::chain_positional_jacobian`] to test
/// whether a chain joint lies on the EE / base path.
fn ancestor_set(
    model: &misarta::model::Model<f64>,
    start: usize,
) -> std::collections::HashSet<usize> {
    let mut set = std::collections::HashSet::new();
    if start == 0 || start >= model.joints.len() {
        return set;
    }
    let mut cur = start;
    while cur > 0 {
        set.insert(cur);
        cur = model.joints[cur].parent;
    }
    set
}

// ─── Conversion helpers ─────────────────────────────────────────────────────

/// Project the articara-side [`ActuatorMode`] onto the misarta config enum.
/// Both share the same three variants; the conversion is a 1:1 mapping kept
/// out-of-line so the misarta crate stays free of articara-specific imports.
fn actuator_mode_to_config(m: ActuatorMode) -> misarta::config::ActuatorMode {
    match m {
        ActuatorMode::Position => misarta::config::ActuatorMode::Position,
        ActuatorMode::Velocity => misarta::config::ActuatorMode::Velocity,
        ActuatorMode::Torque => misarta::config::ActuatorMode::Torque,
        ActuatorMode::ComputedTorque => misarta::config::ActuatorMode::ComputedTorque,
        ActuatorMode::Fixed => misarta::config::ActuatorMode::Fixed,
    }
}

/// Inverse of [`actuator_mode_to_config`].
fn actuator_mode_from_config(m: misarta::config::ActuatorMode) -> ActuatorMode {
    match m {
        misarta::config::ActuatorMode::Position => ActuatorMode::Position,
        misarta::config::ActuatorMode::Velocity => ActuatorMode::Velocity,
        misarta::config::ActuatorMode::Torque => ActuatorMode::Torque,
        misarta::config::ActuatorMode::ComputedTorque => ActuatorMode::ComputedTorque,
        misarta::config::ActuatorMode::Fixed => ActuatorMode::Fixed,
    }
}

/// Convert an articara `JointData.joint_type` string + axis to a misarta `JointType`.
fn convert_joint_type(joint: &JointData) -> JointType<f64> {
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

/// Convert an articara `LinkData` inertial properties to a misarta `LinkInertia`.
fn convert_link_inertia(link: &LinkData) -> LinkInertia<f64> {
    let i = &link.inertial;
    let mass = i.mass;
    let com = i.origin.translation.vector.cast::<f64>();
    let rot = i.origin.rotation.to_rotation_matrix();
    let r = rot.matrix().cast::<f64>();

    let i_com = Matrix3::new(
        i.ixx, i.ixy, i.ixz,
        i.ixy, i.iyy, i.iyz,
        i.ixz, i.iyz, i.izz,
    );
    let rotational_inertia = &r * &i_com * r.transpose();

    LinkInertia {
        mass,
        center_of_mass: com,
        rotational_inertia,
    }
}

/// Cast an `Isometry3<f64>` to `Isometry3<f32>`.
pub fn isometry_f64_to_f32(iso: &na::Isometry3<f64>) -> na::Isometry3<f32> {
    na::Isometry3::from_parts(
        na::Translation3::new(
            iso.translation.x as f32,
            iso.translation.y as f32,
            iso.translation.z as f32,
        ),
        na::UnitQuaternion::new_normalize(na::Quaternion::new(
            iso.rotation.w as f32,
            iso.rotation.i as f32,
            iso.rotation.j as f32,
            iso.rotation.k as f32,
        )),
    )
}

/// Convert an articara `GeomData` to a misarta `GeometryShape`,
/// optionally returning `MeshData` for mesh shapes.
#[allow(dead_code)]
pub fn convert_geom_to_shape_with_mesh(
    geom: &GeomData,
) -> Option<(GeometryShape, Option<MeshData>)> {
    match geom {
        GeomData::Box { hx, hy, hz } => {
            Some((GeometryShape::Box {
                x: *hx as f64 * 2.0,
                y: *hy as f64 * 2.0,
                z: *hz as f64 * 2.0,
            }, None))
        }
        GeomData::Sphere { radius } => {
            Some((GeometryShape::Sphere {
                radius: *radius as f64,
            }, None))
        }
        GeomData::Cylinder { radius, half_length } => {
            Some((GeometryShape::Cylinder {
                radius: *radius as f64,
                length: *half_length as f64 * 2.0,
            }, None))
        }
        GeomData::Capsule { radius, half_length } => {
            Some((GeometryShape::Capsule {
                radius: *radius as f64,
                length: *half_length as f64 * 2.0,
            }, None))
        }
        GeomData::Mesh { vertices, scale, .. } => {
            let s = scale.unwrap_or([1.0, 1.0, 1.0]);
            let n_verts = vertices.len() / 6;
            if n_verts < 3 {
                return None;
            }

            let mut points = Vec::with_capacity(n_verts);
            for i in 0..n_verts {
                let base = i * 6;
                points.push(na::Point3::new(
                    vertices[base] as f64 * s[0] as f64,
                    vertices[base + 1] as f64 * s[1] as f64,
                    vertices[base + 2] as f64 * s[2] as f64,
                ));
            }

            let mut indices = Vec::new();
            let mut face_normals = Vec::new();
            for i in (0..n_verts).step_by(3) {
                if i + 2 >= n_verts {
                    break;
                }
                indices.push([i as u32, (i + 1) as u32, (i + 2) as u32]);
                let v0 = &points[i];
                let v1 = &points[i + 1];
                let v2 = &points[i + 2];
                let e1 = v1 - v0;
                let e2 = v2 - v0;
                let n = e1.cross(&e2);
                let len = n.norm();
                if len > 1e-12 {
                    face_normals.push(n / len);
                } else {
                    face_normals.push(na::Vector3::z());
                }
            }
            if indices.is_empty() {
                return None;
            }

            let md = MeshData {
                vertices: points,
                indices,
                face_normals,
                vertex_normals: Vec::new(),
                texcoords: Vec::new(),
                materials: Vec::new(),
                submeshes: Vec::new(),
            };

            Some((GeometryShape::Mesh {
                scale: na::Vector3::new(1.0, 1.0, 1.0),
                filename: String::new(),
            }, Some(md)))
        }
    }
}
