//! Editor interactions: camera, IK (incl. closed loops), primitives, structural editing and gizmo transforms (split from regression.rs).
//!
//! Shared fixture paths live in `common::fixtures`.

mod common;

#[allow(unused_imports)]
use common::fixtures::*;
#[allow(unused_imports)]
use std::path::{Path, PathBuf};

// ============================================================
// camera.rs — camera math
// ============================================================
mod test_camera {
    use nalgebra as na;
    use articara::camera::OrbitCamera;

    #[test]
    fn default_camera() {
        let cam = OrbitCamera::new();
        assert!(cam.distance > 0.0);
        assert!(cam.fov_y > 0.0);
    }

    #[test]
    fn eye_position_changes_with_distance() {
        let mut cam = OrbitCamera::new();
        let eye1 = cam.eye();
        cam.distance *= 2.0;
        let eye2 = cam.eye();
        let d1 = na::distance(&eye1, &cam.target);
        let d2 = na::distance(&eye2, &cam.target);
        assert!((d2 - d1 * 2.0).abs() < 1e-3);
    }

    #[test]
    fn eye_at_target_distance() {
        let cam = OrbitCamera::new();
        let eye = cam.eye();
        let dist = na::distance(&eye, &cam.target);
        assert!((dist - cam.distance).abs() < 1e-4);
    }

    #[test]
    fn view_matrix_is_invertible() {
        let cam = OrbitCamera::new();
        let view = cam.view_matrix();
        assert!(view.try_inverse().is_some());
    }

    #[test]
    fn projection_matrix_is_invertible() {
        let cam = OrbitCamera::new();
        let proj = cam.projection_matrix(16.0 / 9.0);
        assert!(proj.try_inverse().is_some());
    }

    #[test]
    fn project_target_near_center() {
        let cam = OrbitCamera::new();
        let aspect = 16.0 / 9.0;
        let screen = cam.project(&cam.target, aspect);
        assert!(screen.is_some());
        let s = screen.unwrap();
        assert!((s.x - 0.5).abs() < 0.1, "Target should project near center, got x={}", s.x);
        assert!((s.y - 0.5).abs() < 0.1, "Target should project near center, got y={}", s.y);
    }

    #[test]
    fn screen_ray_center_points_at_target() {
        let cam = OrbitCamera::new();
        let aspect = 16.0 / 9.0;
        let (ro, rd) = cam.screen_ray(na::Point2::new(0.5, 0.5), aspect);
        // Ray from center of screen should point roughly from eye toward target
        let to_target = (cam.target - ro).normalize();
        let dot = rd.dot(&to_target);
        assert!(dot > 0.9, "Center ray should point toward target, dot={dot}");
    }

    #[test]
    fn screen_ray_origin_near_eye() {
        let cam = OrbitCamera::new();
        let (ro, _) = cam.screen_ray(na::Point2::new(0.5, 0.5), 1.0);
        let dist = na::distance(&ro, &cam.eye());
        assert!(dist < 0.1, "Ray origin should be near camera eye, dist={dist}");
    }
}

// ============================================================
// ik.rs — inverse kinematics (now via RobotModel + ModelAdapter)
// ============================================================
mod test_ik {
    use super::*;
    use nalgebra as na;
    use articara::robot::{RobotModel, IkSolver};

    /// Reproduces the user-reported case: with RL_foot as the IK root,
    /// dragging RL_hip should move it in the SAME direction as the cursor
    /// after the per-frame "pin IK root" base correction.
    ///
    /// Pre-fix, the chain Jacobian's lever-arm correction added a spurious
    /// term for joints that were upstream of the BASE only, biasing the
    /// solved Δq enough that the visible hip motion ended up reversed in
    /// some configurations. We assert the post-IK hip displacement is
    /// (a) non-zero and (b) on the same side of the original position as
    /// the requested target.
    #[test]
    fn ik_with_explicit_root_moves_hip_toward_target() {
        let path = namiashi_urdf();
        if !path.exists() {
            // Fixture submodule not initialised; skip.
            eprintln!("namiashi.urdf missing — skipping");
            return;
        }
        let mut model = RobotModel::from_file(&path).unwrap();
        // The IK ee link and its kinematic-root counterpart.
        let ee_link = "RL_hip";
        let root_link = "RL_foot";
        if !model.link_map.contains_key(ee_link)
            || !model.link_map.contains_key(root_link)
        {
            return;
        }

        let chain = model.chain_joints_between(ee_link, Some(root_link));
        assert!(!chain.is_empty(), "chain RL_foot → RL_hip should be non-empty");
        eprintln!("chain joints (foot→hip):");
        for &ji in &chain {
            eprintln!("  {} ({}) axis={:?}",
                model.joints[ji].name, model.joints[ji].joint_type,
                model.joints[ji].axis);
        }

        let transforms = model.compute_transforms();
        let hip_initial = transforms[ee_link].translation.vector.cast::<f64>();
        let foot_initial = transforms[root_link].translation.vector.cast::<f64>();
        eprintln!("hip_initial = {:?}", hip_initial);
        eprintln!("foot_initial = {:?}", foot_initial);

        // Target ~3 cm in +X from the hip's current position.
        let target = na::Point3::new(
            hip_initial.x + 0.03,
            hip_initial.y,
            hip_initial.z,
        );

        // The IK is iterative and can over/under-shoot at high gain, but
        // the *direction* of the very first step must point at the target
        // — that's the bit the constrained Jacobian determines, and the
        // bit the user-reported "drag left → goes right" symptom touches.
        // We assert direction at iter 1; later iterations may oscillate.
        let mut hip_after_first_step: Option<na::Vector3<f64>> = None;
        for iter in 0..3 {
            let cur_tf_dbg = model.compute_transforms();
            let hip_now = cur_tf_dbg[ee_link].translation.vector.cast::<f64>();
            let foot_now = cur_tf_dbg[root_link].translation.vector.cast::<f64>();
            eprintln!("iter {iter}: hip={:?} foot={:?} q_calf={} q_thigh={}",
                hip_now, foot_now,
                model.joint_positions[chain[0]], model.joint_positions[chain[1]]);
            let cur_tf = model.compute_transforms();
            let ee_now = na::Point3::from(
                cur_tf[ee_link].translation.vector.cast::<f64>(),
            );
            // No surface offset for the test (use link origin) so we focus
            // strictly on the chain Jacobian's directional correctness.
            let deltas = model.solve_ik_step(
                &chain,
                ee_link,
                Some(root_link),
                &ee_now,
                &target,
                0.05,
                0.3,
                0.1,
                None,
                IkSolver::Dls,
                None,
                None,
                None,
            );
            model.apply_joint_deltas(&chain, &deltas);

            // Pin RL_foot back to its original world pose by adjusting
            // base_transform — same correction the GUI does each frame.
            let saved_rot = model.base_transform.rotation;
            model.base_transform = na::Isometry3::identity();
            let id_tf = model.compute_transforms();
            if let Some(root_rel) = id_tf.get(root_link) {
                let desired = na::Isometry3::from_parts(
                    na::Translation3::from(foot_initial),
                    saved_rot,
                );
                model.base_transform = desired * root_rel.inverse().cast::<f64>();
            }

            // Capture hip's pose right after the first IK step + base
            // correction so we can assert direction independently of
            // later iterations' over/undershoot.
            if iter == 0 {
                hip_after_first_step = Some(
                    model.compute_transforms()[ee_link]
                        .translation
                        .vector
                        .cast::<f64>(),
                );
            }
        }

        let hip_first = hip_after_first_step.expect("first iter ran");
        let dx_first = hip_first.x - hip_initial.x;
        eprintln!("hip after step 1: {:?}, Δx = {}", hip_first, dx_first);
        assert!(
            dx_first > 0.0,
            "RL_hip should move toward +X target on first IK step, got Δx = {} \
             (hip {:?} → {:?})",
            dx_first,
            hip_initial,
            hip_first,
        );
    }

    #[test]
    fn build_chain_two_joints() {
        let model = RobotModel::from_urdf(&fixture_urdf()).unwrap();
        let chain = model.chain_joints("link2");
        assert_eq!(chain.len(), 2);
        assert_eq!(model.joints[chain[0]].name, "joint1");
        assert_eq!(model.joints[chain[1]].name, "joint2");
    }

    #[test]
    fn build_chain_one_joint() {
        let model = RobotModel::from_urdf(&fixture_urdf()).unwrap();
        let chain = model.chain_joints("link1");
        assert_eq!(chain.len(), 1);
        assert_eq!(model.joints[chain[0]].name, "joint1");
    }

    #[test]
    fn build_chain_root_is_empty() {
        let model = RobotModel::from_urdf(&fixture_urdf()).unwrap();
        let chain = model.chain_joints("base_link");
        assert_eq!(chain.len(), 0);
    }

    #[test]
    fn build_chain_fixed_joint_skipped() {
        let model = RobotModel::from_urdf(&fixture_urdf()).unwrap();
        let chain = model.chain_joints("fixed_part");
        assert_eq!(chain.len(), 0); // fixed joints are not in chain
    }

    #[test]
    fn jacobian_dimensions() {
        let model = RobotModel::from_urdf(&fixture_urdf()).unwrap();
        let chain = model.chain_joints("link2");
        let jac = model.chain_positional_jacobian(&chain, "link2", None, None);
        assert_eq!(jac.nrows(), 3);
        assert_eq!(jac.ncols(), 2);
    }

    #[test]
    fn ik_step_reduces_error() {
        let mut model = RobotModel::from_urdf(&fixture_urdf()).unwrap();
        let chain = model.chain_joints("link2");
        let tf = model.compute_transforms();
        let li = model.link_map["link2"];
        let ee_pos = model.ee_world_pos(li, &tf);
        let target = na::Point3::new(0.1, 0.0, 0.2_f32);
        let initial_error = na::distance(&ee_pos, &target);

        let ee_pos_f64: na::Point3<f64> = ee_pos.cast();
        let target_f64: na::Point3<f64> = target.cast();
        let deltas = model.solve_ik_step(
            &chain, "link2", None,
            &ee_pos_f64, &target_f64, 0.05, 1.0, 0.1,
            None,
            IkSolver::Dls,
            None,
            None,
            None,
        );
        assert_eq!(deltas.len(), 2);

        model.apply_joint_deltas(&chain, &deltas);
        let tf2 = model.compute_transforms();
        let new_pos = model.ee_world_pos(li, &tf2);
        let new_error = na::distance(&new_pos, &target);

        assert!(new_error < initial_error,
            "IK should reduce error: {initial_error} -> {new_error}");
    }

    #[test]
    fn apply_joint_deltas_respects_limits() {
        let mut model = RobotModel::from_urdf(&fixture_urdf()).unwrap();
        let chain = model.chain_joints("link2");

        // Try to apply huge deltas
        let deltas = vec![100.0, 100.0];
        model.apply_joint_deltas(&chain, &deltas);

        let ji1 = chain[0];
        let ji2 = chain[1];
        assert!(model.joint_positions[ji1] <= model.joints[ji1].upper + 1e-6);
        assert!(model.joint_positions[ji2] <= model.joints[ji2].upper + 1e-6);
    }

    // --- chain_joints_between tests ---

    #[test]
    fn build_chain_between_ancestor_root() {
        // root=base_link (ancestor of link2) → full chain
        let model = RobotModel::from_urdf(&fixture_urdf()).unwrap();
        let chain = model.chain_joints_between("link2", Some("base_link"));
        assert_eq!(chain.len(), 2);
        assert_eq!(model.joints[chain[0]].name, "joint1");
        assert_eq!(model.joints[chain[1]].name, "joint2");
    }

    #[test]
    fn build_chain_between_partial() {
        // root=link1 → only joint2
        let model = RobotModel::from_urdf(&fixture_urdf()).unwrap();
        let chain = model.chain_joints_between("link2", Some("link1"));
        assert_eq!(chain.len(), 1);
        assert_eq!(model.joints[chain[0]].name, "joint2");
    }

    #[test]
    fn build_chain_between_none_matches_default() {
        // root=None should behave identically to chain_joints
        let model = RobotModel::from_urdf(&fixture_urdf()).unwrap();
        let chain_default = model.chain_joints("link2");
        let chain_none = model.chain_joints_between("link2", None);
        assert_eq!(chain_default.len(), chain_none.len());
        for (a, b) in chain_default.iter().zip(chain_none.iter()) {
            assert_eq!(a, b);
        }
    }

    #[test]
    fn build_chain_between_same_link_empty() {
        // root == end → empty chain
        let model = RobotModel::from_urdf(&fixture_urdf()).unwrap();
        let chain = model.chain_joints_between("link2", Some("link2"));
        assert_eq!(chain.len(), 0);
    }

    #[test]
    fn build_chain_between_cross_branch() {
        // root=link2, end=fixed_part — these are on different branches
        // LCA = base_link
        // up path from link2 to base_link has 2 movable joints
        // down path from base_link to fixed_part has 0 movable joints (fixed)
        let model = RobotModel::from_urdf(&fixture_urdf()).unwrap();
        let chain = model.chain_joints_between("fixed_part", Some("link2"));
        assert_eq!(chain.len(), 2);
        assert_eq!(model.joints[chain[0]].name, "joint2");
        assert_eq!(model.joints[chain[1]].name, "joint1");
    }

    #[test]
    fn build_chain_between_child_to_parent() {
        // root=link2, end=link1 → goes up: link2 → joint2 → link1
        let model = RobotModel::from_urdf(&fixture_urdf()).unwrap();
        let chain = model.chain_joints_between("link1", Some("link2"));
        assert_eq!(chain.len(), 1);
        assert_eq!(model.joints[chain[0]].name, "joint2");
    }

    // --- Namiashi cross-branch tests ---

    #[test]
    fn namiashi_cross_branch_rl_to_arm() {
        // root=RL_hip, end=arm → cross-branch through trunk
        let model = RobotModel::from_urdf(&namiashi_urdf()).unwrap();
        let chain = model.chain_joints_between("arm", Some("RL_hip"));
        assert_eq!(chain.len(), 2);
    }

    #[test]
    fn namiashi_cross_branch_foot_to_arm() {
        // root=RL_foot, end=arm → goes up the whole RL leg, across trunk, down to arm
        let model = RobotModel::from_urdf(&namiashi_urdf()).unwrap();
        let chain = model.chain_joints_between("arm", Some("RL_foot"));
        assert_eq!(chain.len(), 4); // 3 leg joints + 1 arm joint
    }

    #[test]
    fn namiashi_same_leg_root_partial() {
        // root=RL_hip, end=RL_calf → same branch
        let model = RobotModel::from_urdf(&namiashi_urdf()).unwrap();
        let chain = model.chain_joints_between("RL_calf", Some("RL_hip"));
        assert_eq!(chain.len(), 2); // thigh + calf joints
    }

    #[test]
    fn namiashi_cross_leg() {
        // root=RL_foot, end=FL_foot → two different legs, through trunk
        let model = RobotModel::from_urdf(&namiashi_urdf()).unwrap();
        let chain = model.chain_joints_between("FL_foot", Some("RL_foot"));
        // 3 RL leg joints + 3 FL leg joints
        assert_eq!(chain.len(), 6);
    }
}

// ============================================================
// primitives.rs — geometry generation
// ============================================================
mod test_primitives {
    use articara::primitives;

    #[test]
    fn generate_box_vertex_count() {
        let v = primitives::generate_box(1.0, 1.0, 1.0);
        // 6 faces × 2 triangles × 3 vertices × 6 floats = 216
        assert_eq!(v.len(), 216);
    }

    #[test]
    fn generate_box_coords_within_bounds() {
        let v = primitives::generate_box(0.5, 0.3, 0.1);
        for chunk in v.chunks(6) {
            let (x, y, z) = (chunk[0], chunk[1], chunk[2]);
            assert!(x.abs() <= 0.5 + 1e-6);
            assert!(y.abs() <= 0.3 + 1e-6);
            assert!(z.abs() <= 0.1 + 1e-6);
        }
    }

    #[test]
    fn generate_cylinder_non_empty() {
        let v = primitives::generate_cylinder(0.5, 1.0, 16);
        assert!(!v.is_empty());
        // 16 segments × (6 side + 3 top + 3 bottom) vertices × 6 floats
        assert_eq!(v.len(), 16 * (6 + 3 + 3) * 6);
    }

    #[test]
    fn generate_cylinder_radius_bounded() {
        let v = primitives::generate_cylinder(0.5, 1.0, 16);
        for chunk in v.chunks(6) {
            let (x, y, z) = (chunk[0], chunk[1], chunk[2]);
            let r = (x * x + y * y).sqrt();
            assert!(r <= 0.5 + 1e-4, "radius={r}");
            assert!(z.abs() <= 1.0 + 1e-6, "z={z}");
        }
    }

    #[test]
    fn generate_sphere_non_empty() {
        let v = primitives::generate_sphere(1.0, 16, 8);
        assert!(!v.is_empty());
    }

    #[test]
    fn generate_sphere_radius_bounded() {
        let v = primitives::generate_sphere(1.0, 16, 8);
        for chunk in v.chunks(6) {
            let (x, y, z) = (chunk[0], chunk[1], chunk[2]);
            let r = (x * x + y * y + z * z).sqrt();
            assert!(r <= 1.0 + 1e-3, "sphere point at radius {r}");
        }
    }

    #[test]
    fn generate_grid_non_empty() {
        let v = primitives::generate_grid(1.0, 10);
        assert!(!v.is_empty());
    }

    #[test]
    fn generate_axes_six_endpoints() {
        let v = primitives::generate_axes(1.0);
        // 3 axes × 2 endpoints × 6 floats = 36
        assert_eq!(v.len(), 36);
    }

    #[test]
    fn generate_arrow_non_empty() {
        let v = primitives::generate_arrow(0.003, 0.06, 0.009, 0.02, 12);
        assert!(!v.is_empty());
    }

    #[test]
    fn generate_arrow_vertex_count() {
        // Per segment: 6 shaft side + 3 shaft cap + 3 cone side + 3 cone base = 15 vertices
        // 12 segments × 15 × 6 floats = 1080
        let v = primitives::generate_arrow(0.003, 0.06, 0.009, 0.02, 12);
        assert_eq!(v.len(), 12 * 15 * 6);
    }

    #[test]
    fn generate_arrow_z_bounded() {
        let shaft_len = 0.06_f32;
        let head_len = 0.02_f32;
        let v = primitives::generate_arrow(0.003, shaft_len, 0.009, head_len, 12);
        for chunk in v.chunks(6) {
            let z = chunk[2];
            assert!(z >= -1e-6, "z below origin: {z}");
            assert!(z <= shaft_len + head_len + 1e-6, "z above tip: {z}");
        }
    }

    #[test]
    fn generate_arrow_radius_bounded() {
        let head_radius = 0.009_f32;
        let v = primitives::generate_arrow(0.003, 0.06, head_radius, 0.02, 12);
        for chunk in v.chunks(6) {
            let r = (chunk[0] * chunk[0] + chunk[1] * chunk[1]).sqrt();
            assert!(r <= head_radius + 1e-4, "point beyond head_radius: r={r}");
        }
    }
}

// ============================================================
// Model editing — add_link, add_joint, add_child, remove_link
// ============================================================
mod test_model_editing {
    use nalgebra as na;
    use articara::robot::*;

    #[test]
    fn new_empty_has_root_link() {
        let model = RobotModel::new_empty("test");
        assert_eq!(model.name, "test");
        assert_eq!(model.links.len(), 1);
        assert_eq!(model.root_link, "base_link");
        assert!(model.link_map.contains_key("base_link"));
        assert_eq!(model.joints.len(), 0);
    }

    #[test]
    fn add_link_updates_maps() {
        let mut model = RobotModel::new_empty("test");
        let idx = model.add_link("arm", GeomData::Sphere { radius: 0.05 }, [1.0, 0.0, 0.0, 1.0]);
        assert_eq!(idx, 1);
        assert_eq!(model.links.len(), 2);
        assert_eq!(model.link_map["arm"], 1);
        assert_eq!(model.links[1].name, "arm");
    }

    #[test]
    fn add_joint_updates_maps_and_children() {
        let mut model = RobotModel::new_empty("test");
        model.add_link("child1", GeomData::Box { hx: 0.1, hy: 0.1, hz: 0.1 }, [1.0; 4]);
        let ji = model.add_joint(
            "j1", "revolute", "base_link", "child1",
            na::Isometry3::identity(),
            na::Vector3::z(),
            -1.0, 1.0,
        ).unwrap();
        assert_eq!(ji, 0);
        assert_eq!(model.joints.len(), 1);
        assert_eq!(model.joint_map["j1"], 0);
        assert_eq!(model.children_joints["base_link"], vec![0]);
        assert_eq!(model.joint_positions.len(), 1);
        assert_eq!(model.joint_positions[0], 0.0);
    }

    #[test]
    fn add_joint_invalid_parent_fails() {
        let mut model = RobotModel::new_empty("test");
        model.add_link("child1", GeomData::Sphere { radius: 0.1 }, [1.0; 4]);
        let result = model.add_joint(
            "j1", "revolute", "no_such_link", "child1",
            na::Isometry3::identity(),
            na::Vector3::z(),
            -1.0, 1.0,
        );
        assert!(result.is_err());
    }

    #[test]
    fn add_child_creates_link_and_joint() {
        let mut model = RobotModel::new_empty("test");
        let (li, ji) = model.add_child(
            "base_link", "leg", "base_to_leg", "revolute",
            na::Isometry3::new(na::Vector3::new(0.0, 0.0, 0.1), na::Vector3::zeros()),
            na::Vector3::y(),
            GeomData::Cylinder { radius: 0.02, half_length: 0.1 },
            [0.0, 1.0, 0.0, 1.0],
            -1.57, 1.57,
        ).unwrap();
        assert_eq!(li, 1);
        assert_eq!(ji, 0);
        assert_eq!(model.links.len(), 2);
        assert_eq!(model.joints.len(), 1);
        assert_eq!(model.joints[0].parent_link, "base_link");
        assert_eq!(model.joints[0].child_link, "leg");
    }

    #[test]
    fn add_child_transforms_valid() {
        let mut model = RobotModel::new_empty("robot");
        model.add_child(
            "base_link", "link1", "joint1", "revolute",
            na::Isometry3::new(na::Vector3::new(0.0, 0.0, 0.5), na::Vector3::zeros()),
            na::Vector3::z(),
            GeomData::Box { hx: 0.05, hy: 0.05, hz: 0.25 },
            [1.0; 4],
            -3.14, 3.14,
        ).unwrap();
        let tf = model.compute_transforms();
        assert!(tf.contains_key("base_link"));
        assert!(tf.contains_key("link1"));
        let link1_pos = tf["link1"].translation.vector;
        assert!((link1_pos.z - 0.5).abs() < 0.001);
    }

    #[test]
    fn generate_link_name_unique() {
        let mut model = RobotModel::new_empty("test");
        assert_eq!(model.generate_link_name("arm"), "arm");
        model.add_link("arm", GeomData::Sphere { radius: 0.05 }, [1.0; 4]);
        assert_eq!(model.generate_link_name("arm"), "arm_1");
    }

    #[test]
    fn generate_joint_name_unique() {
        let mut model = RobotModel::new_empty("test");
        model.add_link("child", GeomData::Sphere { radius: 0.05 }, [1.0; 4]);
        assert_eq!(model.generate_joint_name("j"), "j");
        model.add_joint("j", "fixed", "base_link", "child",
            na::Isometry3::identity(), na::Vector3::z(), 0.0, 0.0).unwrap();
        assert_eq!(model.generate_joint_name("j"), "j_1");
    }

    #[test]
    fn remove_link_basic() {
        let mut model = RobotModel::new_empty("test");
        model.add_child(
            "base_link", "arm", "base_to_arm", "revolute",
            na::Isometry3::identity(), na::Vector3::z(),
            GeomData::Sphere { radius: 0.05 }, [1.0; 4],
            -1.0, 1.0,
        ).unwrap();
        assert_eq!(model.links.len(), 2);
        let removed = model.remove_link("arm").unwrap();
        assert_eq!(removed, vec!["arm"]);
        assert_eq!(model.links.len(), 1);
        assert_eq!(model.joints.len(), 0);
        assert!(!model.link_map.contains_key("arm"));
    }

    #[test]
    fn remove_link_recursive() {
        let mut model = RobotModel::new_empty("test");
        model.add_child(
            "base_link", "arm", "j1", "revolute",
            na::Isometry3::identity(), na::Vector3::z(),
            GeomData::Sphere { radius: 0.05 }, [1.0; 4],
            -1.0, 1.0,
        ).unwrap();
        model.add_child(
            "arm", "hand", "j2", "revolute",
            na::Isometry3::identity(), na::Vector3::z(),
            GeomData::Sphere { radius: 0.03 }, [1.0; 4],
            -1.0, 1.0,
        ).unwrap();
        assert_eq!(model.links.len(), 3);
        let removed = model.remove_link("arm").unwrap();
        assert!(removed.contains(&"arm".to_string()));
        assert!(removed.contains(&"hand".to_string()));
        assert_eq!(model.links.len(), 1);
        assert_eq!(model.joints.len(), 0);
    }

    #[test]
    fn remove_root_link_fails() {
        let mut model = RobotModel::new_empty("test");
        let result = model.remove_link("base_link");
        assert!(result.is_err());
    }

    #[test]
    fn remove_nonexistent_link_fails() {
        let mut model = RobotModel::new_empty("test");
        let result = model.remove_link("no_such");
        assert!(result.is_err());
    }

    #[test]
    fn rebuild_indices_consistency() {
        let mut model = RobotModel::new_empty("test");
        model.add_child(
            "base_link", "a", "j1", "revolute",
            na::Isometry3::identity(), na::Vector3::z(),
            GeomData::Sphere { radius: 0.02 }, [1.0; 4], -1.0, 1.0,
        ).unwrap();
        model.add_child(
            "a", "b", "j2", "fixed",
            na::Isometry3::identity(), na::Vector3::z(),
            GeomData::Sphere { radius: 0.02 }, [1.0; 4], 0.0, 0.0,
        ).unwrap();
        model.rebuild_indices();
        assert_eq!(model.link_map.len(), 3);
        assert_eq!(model.joint_map.len(), 2);
        assert_eq!(model.joint_positions.len(), 2);
        // Verify map values match vector positions
        for (name, &idx) in &model.link_map {
            assert_eq!(model.links[idx].name, *name);
        }
        for (name, &idx) in &model.joint_map {
            assert_eq!(model.joints[idx].name, *name);
        }
    }

    #[test]
    fn link_names_returns_all() {
        let mut model = RobotModel::new_empty("test");
        model.add_link("a", GeomData::Sphere { radius: 0.05 }, [1.0; 4]);
        model.add_link("b", GeomData::Sphere { radius: 0.05 }, [1.0; 4]);
        let names = model.link_names();
        assert_eq!(names.len(), 3);
        assert!(names.contains(&"base_link".to_string()));
        assert!(names.contains(&"a".to_string()));
        assert!(names.contains(&"b".to_string()));
    }

    #[test]
    fn added_model_exports_valid_urdf() {
        let mut model = RobotModel::new_empty("built_robot");
        model.add_child(
            "base_link", "arm", "base_to_arm", "revolute",
            na::Isometry3::new(na::Vector3::new(0.0, 0.0, 0.2), na::Vector3::zeros()),
            na::Vector3::z(),
            GeomData::Box { hx: 0.05, hy: 0.05, hz: 0.1 },
            [0.8, 0.2, 0.2, 1.0],
            -1.57, 1.57,
        ).unwrap();
        let xml = model.export_urdf().expect("export_urdf failed");
        assert!(xml.contains("<robot"));
        assert!(xml.contains("built_robot"));
        assert!(xml.contains("base_link"));
        assert!(xml.contains("arm"));
        assert!(xml.contains("base_to_arm"));
        assert!(xml.contains("revolute"));
    }

    #[test]
    fn added_model_exports_valid_sdf() {
        use articara::sdf::export_sdf;
        let mut model = RobotModel::new_empty("sdf_test");
        model.add_child(
            "base_link", "link1", "j1", "fixed",
            na::Isometry3::identity(), na::Vector3::z(),
            GeomData::Sphere { radius: 0.1 }, [1.0; 4], 0.0, 0.0,
        ).unwrap();
        let sdf = export_sdf(&model);
        assert!(sdf.contains("<sdf"));
        assert!(sdf.contains("sdf_test"));
        assert!(sdf.contains("link1"));
    }

    #[test]
    fn added_model_exports_valid_mjcf() {
        use articara::mjcf::export_mjcf;
        let mut model = RobotModel::new_empty("mjcf_test");
        model.add_child(
            "base_link", "link1", "j1", "revolute",
            na::Isometry3::identity(), na::Vector3::z(),
            GeomData::Cylinder { radius: 0.03, half_length: 0.15 },
            [0.5, 0.5, 1.0, 1.0],
            -3.14, 3.14,
        ).unwrap();
        let mjcf = export_mjcf(&model);
        assert!(mjcf.contains("<mujoco"));
        assert!(mjcf.contains("mjcf_test"));
    }

    #[test]
    fn multiple_children_from_same_parent() {
        let mut model = RobotModel::new_empty("multi");
        model.add_child(
            "base_link", "left", "j_left", "revolute",
            na::Isometry3::new(na::Vector3::new(0.0, 0.1, 0.0), na::Vector3::zeros()),
            na::Vector3::z(),
            GeomData::Sphere { radius: 0.05 }, [1.0, 0.0, 0.0, 1.0],
            -1.0, 1.0,
        ).unwrap();
        model.add_child(
            "base_link", "right", "j_right", "revolute",
            na::Isometry3::new(na::Vector3::new(0.0, -0.1, 0.0), na::Vector3::zeros()),
            na::Vector3::z(),
            GeomData::Sphere { radius: 0.05 }, [0.0, 0.0, 1.0, 1.0],
            -1.0, 1.0,
        ).unwrap();
        assert_eq!(model.links.len(), 3);
        assert_eq!(model.joints.len(), 2);
        assert_eq!(model.children_joints["base_link"].len(), 2);
        let tf = model.compute_transforms();
        let left_y = tf["left"].translation.vector.y;
        let right_y = tf["right"].translation.vector.y;
        assert!((left_y - 0.1).abs() < 0.001);
        assert!((right_y + 0.1).abs() < 0.001);
    }
}

// ============================================================
// Gizmo / Offset Adjustment tests
// ============================================================
mod test_gizmo {
    use nalgebra as na;
    use articara::robot::{self, GeomData, RobotModel};
    use super::fixture_urdf;

    #[test]
    fn ray_axis_closest_perpendicular_hit() {
        // Ray along +Y, axis along +X at origin → should be closest at t_line=0, dist=0
        let ro = na::Point3::new(0.0, -1.0, 0.0);
        let rd = na::Vector3::new(0.0, 1.0, 0.0);
        let origin = na::Point3::origin();
        let axis = na::Vector3::x();
        let (t_line, dist) = robot::ray_axis_closest(&ro, &rd, &origin, &axis);
        assert!(dist.abs() < 1e-5, "dist={dist}");
        assert!(t_line.abs() < 1e-5, "t_line={t_line}");
    }

    #[test]
    fn ray_axis_closest_offset_hit() {
        // Ray along +Y at x=0.5, axis along +X at origin
        // Closest point on axis should be at t_line=0.5 (i.e., [0.5, 0, 0])
        let ro = na::Point3::new(0.5, -1.0, 0.0);
        let rd = na::Vector3::new(0.0, 1.0, 0.0);
        let origin = na::Point3::origin();
        let axis = na::Vector3::x();
        let (t_line, dist) = robot::ray_axis_closest(&ro, &rd, &origin, &axis);
        assert!(dist.abs() < 1e-5, "dist={dist}");
        assert!((t_line - 0.5).abs() < 1e-5, "t_line={t_line}");
    }

    #[test]
    fn ray_axis_closest_skew_lines() {
        // Ray along +Z at (1,0,0), axis along +X at origin
        // Distance should be 0 (they intersect at (1,0,0) with t_line=1)
        let ro = na::Point3::new(1.0, 0.0, -1.0);
        let rd = na::Vector3::new(0.0, 0.0, 1.0);
        let origin = na::Point3::origin();
        let axis = na::Vector3::x();
        let (t_line, dist) = robot::ray_axis_closest(&ro, &rd, &origin, &axis);
        assert!(dist.abs() < 1e-5, "dist={dist}");
        assert!((t_line - 1.0).abs() < 1e-5, "t_line={t_line}");
    }

    #[test]
    fn ray_axis_closest_nonzero_distance() {
        // Ray along +Z at (0, 0.1, 0), axis along +X at origin
        // Closest distance should be 0.1 (the Y-offset)
        let ro = na::Point3::new(0.0, 0.1, -1.0);
        let rd = na::Vector3::new(0.0, 0.0, 1.0);
        let origin = na::Point3::origin();
        let axis = na::Vector3::x();
        let (t_line, dist) = robot::ray_axis_closest(&ro, &rd, &origin, &axis);
        assert!((dist - 0.1).abs() < 1e-5, "dist={dist}");
        assert!(t_line.abs() < 1e-5, "t_line={t_line}");
    }

    #[test]
    fn ray_axis_closest_parallel_rays() {
        // Ray parallel to axis (both along X, offset in Y)
        let ro = na::Point3::new(0.0, 0.5, 0.0);
        let rd = na::Vector3::new(1.0, 0.0, 0.0);
        let origin = na::Point3::origin();
        let axis = na::Vector3::x();
        let (_t_line, dist) = robot::ray_axis_closest(&ro, &rd, &origin, &axis);
        assert!((dist - 0.5).abs() < 1e-5, "dist={dist}");
    }

    #[test]
    fn joint_origin_translation_editable() {
        // Build a simple robot model with a joint, verify we can modify the origin
        let mut model = RobotModel::new_empty("gizmo_test");
        model
            .add_child(
                "base_link",
                "link1",
                "joint1",
                "revolute",
                na::Isometry3::new(
                    na::Vector3::new(0.0, 0.0, 0.1),
                    na::Vector3::zeros(),
                ),
                na::Vector3::z(),
                GeomData::Sphere { radius: 0.02 },
                [1.0, 0.0, 0.0, 1.0],
                -1.0,
                1.0,
            )
            .unwrap();

        // Verify initial translation
        let ji = 0;
        let orig = model.joints[ji].origin.translation.vector;
        assert!((orig.z - 0.1).abs() < 1e-6);

        // Simulate offset adjustment: move joint along X by 0.05
        model.joints[ji].origin.translation.vector.x += 0.05;
        let tf = model.compute_transforms();
        let link1_pos = tf["link1"].translation.vector;
        assert!((link1_pos.x - 0.05).abs() < 1e-5, "x={}", link1_pos.x);
        assert!((link1_pos.z - 0.1).abs() < 1e-5, "z={}", link1_pos.z);
    }

    #[test]
    fn gizmo_transform_at_joint_world_position() {
        // Verify gizmo would be placed at the correct world position
        let mut model = RobotModel::new_empty("gizmo_pos");
        model
            .add_child(
                "base_link",
                "link1",
                "joint1",
                "revolute",
                na::Isometry3::new(
                    na::Vector3::new(0.0, 0.0, 0.2),
                    na::Vector3::zeros(),
                ),
                na::Vector3::z(),
                GeomData::Sphere { radius: 0.02 },
                [1.0, 0.0, 0.0, 1.0],
                -1.0,
                1.0,
            )
            .unwrap();

        let tf = model.compute_transforms();
        let joint = &model.joints[0];
        let parent_tf = tf
            .get(&joint.parent_link)
            .copied()
            .unwrap_or(na::Isometry3::identity());
        let joint_world = parent_tf * joint.origin;

        // Gizmo position should match joint world position
        let gizmo_pos = joint_world.translation.vector;
        assert!((gizmo_pos.z - 0.2).abs() < 1e-5);
    }

    #[test]
    fn ancestor_links_root() {
        let model = articara::robot::RobotModel::from_file(&fixture_urdf()).unwrap();
        // Root link has no ancestors
        let ancestors = model.ancestor_links(&model.root_link);
        assert!(ancestors.is_empty());
    }

    #[test]
    fn ancestor_links_child() {
        let model = articara::robot::RobotModel::from_file(&fixture_urdf()).unwrap();
        let root = &model.root_link;
        // Find a child of the root
        if let Some(joints) = model.children_joints.get(root.as_str()) {
            if let Some(&ji) = joints.first() {
                let child_name = &model.joints[ji].child_link;
                let ancestors = model.ancestor_links(child_name);
                assert_eq!(ancestors.len(), 1);
                assert_eq!(&ancestors[0], root);
            }
        }
    }

    #[test]
    fn ancestor_links_deep() {
        // Build a chain: base_link -> A -> B
        let mut model = articara::robot::RobotModel::new_empty("test");
        model
            .add_child(
                "base_link",
                "child_A",
                "j1",
                "revolute",
                na::Isometry3::identity(),
                na::Vector3::z(),
                articara::robot::GeomData::Sphere { radius: 0.01 },
                [1.0, 1.0, 1.0, 1.0],
                -1.0,
                1.0,
            )
            .unwrap();
        model
            .add_child(
                "child_A",
                "child_B",
                "j2",
                "revolute",
                na::Isometry3::identity(),
                na::Vector3::z(),
                articara::robot::GeomData::Sphere { radius: 0.01 },
                [1.0, 1.0, 1.0, 1.0],
                -1.0,
                1.0,
            )
            .unwrap();
        let ancestors = model.ancestor_links("child_B");
        assert_eq!(ancestors, vec!["base_link".to_string(), "child_A".to_string()]);
    }

    #[test]
    fn generate_ring_non_empty() {
        let data = articara::primitives::generate_ring(0.05, 0.003, 48, 8);
        // Each vertex has 6 floats (pos + normal)
        assert!(!data.is_empty());
        assert_eq!(data.len() % 6, 0);
    }

    #[test]
    fn generate_ring_vertex_count() {
        let ring_segs: u32 = 24;
        let tube_segs: u32 = 6;
        let data = articara::primitives::generate_ring(0.05, 0.003, ring_segs, tube_segs);
        // Torus: ring_segs * tube_segs quads, 6 verts each
        // Arrowheads: 2 arrows × 12 cone_segs × 2 tris × 3 verts = 144
        let torus_verts = (ring_segs * tube_segs * 6) as usize;
        let arrow_verts = 2 * 12 * 2 * 3; // 2 arrows, 12 cone_segs, side+cap tri, 3 verts
        let total_verts = torus_verts + arrow_verts;
        assert_eq!(data.len() / 6, total_verts);
    }

    #[test]
    fn generate_ring_bounded_radius() {
        let ring_r = 0.05_f32;
        let tube_r = 0.003_f32;
        let data = articara::primitives::generate_ring(ring_r, tube_r, 48, 8);
        // Arrowhead tip extends along tangent by ring_r*0.28 and cone radius is tube_r*2.8
        // The maximum distance from origin includes both the ring radius and arrow geometry
        let arrow_len = ring_r * 0.28;
        let arrow_r = tube_r * 2.8;
        let outer = ((ring_r + arrow_r).powi(2) + arrow_len.powi(2)).sqrt() + 1e-4;
        for chunk in data.chunks(6) {
            let x = chunk[0];
            let y = chunk[1];
            let z = chunk[2];
            let dist = (x * x + y * y + z * z).sqrt();
            assert!(
                dist <= outer,
                "vertex ({x},{y},{z}) dist {dist} > outer bound {outer}"
            );
        }
    }

    #[test]
    fn generate_scale_handle_non_empty() {
        let data = articara::primitives::generate_scale_handle(0.003, 0.06, 0.006, 12);
        assert!(!data.is_empty());
        assert_eq!(data.len() % 6, 0);
    }

    #[test]
    fn generate_scale_handle_vertex_count() {
        let segs: u32 = 12;
        let data = articara::primitives::generate_scale_handle(0.003, 0.06, 0.006, segs);
        // Shaft: segs * 2 tris * 3 verts = segs * 6
        // Cube: 6 faces * 2 tris * 3 verts = 36
        let expected = (segs * 6 + 36) as usize;
        assert_eq!(data.len() / 6, expected);
    }
}

// ============================================================
// Closed-loop IK tests
// ============================================================
mod test_closed_loop_ik {
    use super::*;
    use articara::robot::*;
    use articara::sdf;

    fn fixture_four_bar() -> PathBuf {
        fixtures_dir().join("sdf").join("four_bar.sdf")
    }

    fn fixture_five_bar() -> PathBuf {
        fixtures_dir().join("sdf").join("five_bar_parallel.sdf")
    }

    /// Helper: compute loop-closure error for the four-bar linkage.
    fn four_bar_loop_error(model: &RobotModel) -> f32 {
        let transforms = model.compute_transforms();
        let coupler_tf = transforms["coupler"];
        let coupler_tip = coupler_tf * nalgebra::Point3::new(0.3_f32, 0.0, 0.0);
        let crank_r_tf = transforms["crank_right"];
        let crank_r_tip = crank_r_tf * nalgebra::Point3::new(0.0_f32, 0.0, 0.2);
        (coupler_tip - crank_r_tip).norm()
    }

    #[test]
    fn four_bar_constraint_model_builds() {
        let mut model = sdf::import_sdf(&fixture_four_bar()).unwrap();
        model.loop_closures.push(LoopClosure::position(
            "four_bar_loop",
            "coupler",
            nalgebra::Vector3::new(0.3, 0.0, 0.0),
            "crank_right",
            nalgebra::Vector3::new(0.0, 0.0, 0.2),
        ));

        let cm = model.build_loop_constraint_model();
        assert_eq!(cm.len(), 1);
        assert_eq!(cm.total_dim(), 3); // position-only = 3 rows
    }

    #[test]
    fn four_bar_diff_constraints_build() {
        let mut model = sdf::import_sdf(&fixture_four_bar()).unwrap();
        model.loop_closures.push(LoopClosure::position(
            "loop",
            "coupler",
            nalgebra::Vector3::new(0.3, 0.0, 0.0),
            "crank_right",
            nalgebra::Vector3::new(0.0, 0.0, 0.2),
        ));

        let diffs = model.build_loop_diff_constraints(10.0);
        assert_eq!(diffs.len(), 1);
        assert_eq!(diffs[0].jacobian.nrows(), 3);
        let mc = model.mc();
        assert_eq!(diffs[0].jacobian.ncols(), mc.model.nv);
    }

    #[test]
    fn four_bar_ik_step_maintains_closure() {
        let mut model = sdf::import_sdf(&fixture_four_bar()).unwrap();
        model.loop_closures.push(LoopClosure::position(
            "loop",
            "coupler",
            nalgebra::Vector3::new(0.3, 0.0, 0.0),
            "crank_right",
            nalgebra::Vector3::new(0.0, 0.0, 0.2),
        ));

        // Perturb joint_left slightly (will break loop)
        model.joint_positions[0] = 0.3;
        let err_before = four_bar_loop_error(&model);
        assert!(err_before > 0.01, "expected broken loop: err={}", err_before);

        // Run several IK steps with loop constraint to restore closure
        // Using the coupler link as "EE" target, moving it toward a position
        // while maintaining the loop.
        let loop_cs = model.build_loop_diff_constraints(50.0);
        let transforms = model.compute_transforms();
        let ee_pos = model.ee_world_pos(
            *model.link_map.get("coupler").unwrap(),
            &transforms,
        ).cast::<f64>();
        // Target = current position (just maintain loop)
        let target = ee_pos;

        for _ in 0..50 {
            let lc = model.build_loop_diff_constraints(50.0);
            let deltas = model.solve_ik_step_with_pins(
                "coupler",
                &ee_pos,
                &target,
                &[],
                0.01,   // damping
                0.3,    // gain
                0.15,   // max_step
                IkSolver::Dls,
                None,
                None,
                10.0,
                &lc,
                None,
            );
            model.apply_all_joint_deltas(&deltas);
        }

        let err_after = four_bar_loop_error(&model);
        assert!(
            err_after < err_before * 0.5,
            "Loop closure should improve: before={} after={}",
            err_before, err_after
        );
    }

    #[test]
    fn five_bar_ik_maintains_closure() {
        let mut model = sdf::import_sdf(&fixture_five_bar()).unwrap();

        // Loop constraint: end_effector ↔ distal_right tip
        model.loop_closures.push(LoopClosure::position(
            "five_bar_loop",
            "end_effector",
            nalgebra::Vector3::new(0.0, 0.0, 0.0),
            "distal_right",
            nalgebra::Vector3::new(0.0, 0.0, 0.2),
        ));

        // Perturb joint_L1
        model.joint_positions[0] = 0.2;

        let initial_err = model.loop_closure_error();
        assert!(initial_err > 0.001, "expected broken loop: err={}", initial_err);

        // Run iterative constraint-only IK to close the loop
        let mc = model.mc();
        let q0 = mc.build_q(&model);
        let cm = model.build_loop_constraint_model();
        let config = misarta::constraint::ConstrainedIkConfig {
            max_iters: 200,
            tol_constraint: 1e-5,
            step_size: 0.5,
            damping: 1e-3,
            constraint_weight: 10.0,
            tol_task: 1e-6,
        };
        let result = misarta::constraint::solve_constrained_ik(
            &mc.model, &q0, &cm, &config,
        );

        assert!(
            result.converged || result.constraint_error_norm < 1e-3,
            "Five-bar loop closure should converge: err={}, iters={}",
            result.constraint_error_norm, result.iterations
        );
    }

    #[test]
    fn misarta_build_diff_ik_constraints_basic() {
        // Test the misarta-level bridge function directly
        use misarta::{model::*, joint, se3};
        use misarta::frames::Frame;
        use misarta::constraint::*;

        // Simple 3-joint chain
        let model = ModelBuilder::<f64>::new()
            .add_joint("j1", 0, joint::revolute_z(), se3::identity(), LinkInertia::zero())
            .add_joint("j2", 1, joint::revolute_y(), se3::identity(), LinkInertia::zero())
            .add_joint("j3", 1, joint::revolute_x(), se3::identity(), LinkInertia::zero())
            .build();

        let f1 = Frame { name: "a".into(), parent_joint: 2, placement: se3::identity() };
        let f2 = Frame { name: "b".into(), parent_joint: 3, placement: se3::identity() };

        // 3D constraint
        let cm3 = ConstraintModel::from_constraints(vec![
            RigidConstraint::position(f1.clone(), f2.clone()),
        ]);
        let q = vec![0.0; model.nq];
        let cs3 = build_diff_ik_constraints(&model, &q, &cm3, 10.0);
        assert_eq!(cs3.len(), 1);
        assert_eq!(cs3[0].jacobian.nrows(), 3);
        assert_eq!(cs3[0].weight, 10.0);

        // 6D constraint
        let cm6 = ConstraintModel::from_constraints(vec![
            RigidConstraint::pose(f1, f2),
        ]);
        let cs6 = build_diff_ik_constraints(&model, &q, &cm6, 5.0);
        assert_eq!(cs6.len(), 1);
        assert_eq!(cs6[0].jacobian.nrows(), 6);
        assert_eq!(cs6[0].weight, 5.0);
    }

    /// Verify that the `.misarta.toml` sidecar is loaded automatically via `load_sidecar_config()`.
    #[test]
    fn five_bar_sidecar_toml_loaded() {
        let mut model = sdf::import_sdf(&fixture_five_bar()).unwrap();
        // Before loading, no loop closures
        assert!(model.loop_closures.is_empty());

        // load_sidecar_config looks for five_bar_parallel.misarta.toml next to the .sdf
        let loaded = model.load_sidecar_config();
        assert!(loaded.is_some(), "Expected .misarta.toml sidecar to be found and loaded");

        // Should have exactly 1 loop closure
        assert_eq!(model.loop_closures.len(), 1);
        let lc = &model.loop_closures[0];
        assert_eq!(lc.name, "ee_loop");
        assert_eq!(lc.link_a, "end_effector");
        assert_eq!(lc.link_b, "distal_right");
        assert!(!lc.pose_6dof); // position-only (3-DoF)

        // offset_a should be zero, offset_b should be (0, 0, 0.2)
        let oa = lc.offset_a.translation.vector;
        assert!((oa.norm()) < 1e-10, "offset_a should be zero, got {:?}", oa);
        let ob = lc.offset_b.translation.vector;
        assert!((ob - nalgebra::Vector3::new(0.0, 0.0, 0.2)).norm() < 1e-10,
            "offset_b should be (0,0,0.2), got {:?}", ob);

        // The loaded constraint should produce a valid constraint model and near-zero error at q=0
        model.rebuild_misarta_model();
        let err = model.loop_closure_error();
        assert!(err < 0.01, "Loop closure error at q=0 should be near zero, got {}", err);
    }
}

