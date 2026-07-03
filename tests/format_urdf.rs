//! URDF import / export, transforms and ray picking (split from regression.rs).
//!
//! Shared fixture paths live in `common::fixtures`.

mod common;

#[allow(unused_imports)]
use common::fixtures::*;
#[allow(unused_imports)]
use std::path::{Path, PathBuf};

// ============================================================
// robot.rs — URDF loading, transforms, ray intersection
// ============================================================
mod test_robot {
    use super::*;
    use nalgebra as na;
    use articara::robot::*;

    #[test]
    fn load_fixture_urdf() {
        let model = RobotModel::from_urdf(&fixture_urdf()).expect("Failed to load fixture URDF");
        assert_eq!(model.name, "test_robot");
        assert_eq!(model.links.len(), 4);
        assert_eq!(model.joints.len(), 3);
        assert_eq!(model.root_link, "base_link");
    }

    #[test]
    fn link_map_contains_all_links() {
        let model = RobotModel::from_urdf(&fixture_urdf()).unwrap();
        for link in &model.links {
            assert!(model.link_map.contains_key(&link.name), "Missing link: {}", link.name);
        }
    }

    #[test]
    fn joint_types_correct() {
        let model = RobotModel::from_urdf(&fixture_urdf()).unwrap();
        let j1 = model.joints.iter().find(|j| j.name == "joint1").unwrap();
        assert_eq!(j1.joint_type, "revolute");
        let jf = model.joints.iter().find(|j| j.name == "fixed_joint").unwrap();
        assert_eq!(jf.joint_type, "fixed");
    }

    #[test]
    fn joint_limits_parsed() {
        let model = RobotModel::from_urdf(&fixture_urdf()).unwrap();
        let j1 = model.joints.iter().find(|j| j.name == "joint1").unwrap();
        assert!((j1.lower - (-1.57)).abs() < 1e-4);
        assert!((j1.upper - 1.57).abs() < 1e-4);
        assert!((j1.effort - 10.0).abs() < 1e-4);
        assert!((j1.velocity - 5.0).abs() < 1e-4);
    }

    #[test]
    fn joint_axis_parsed() {
        let model = RobotModel::from_urdf(&fixture_urdf()).unwrap();
        let j1 = model.joints.iter().find(|j| j.name == "joint1").unwrap();
        assert!((j1.axis.x).abs() < 1e-6);
        assert!((j1.axis.y - 1.0).abs() < 1e-6);
        assert!((j1.axis.z).abs() < 1e-6);
    }

    #[test]
    fn inertial_parsed() {
        let model = RobotModel::from_urdf(&fixture_urdf()).unwrap();
        let base = model.links.iter().find(|l| l.name == "base_link").unwrap();
        assert!((base.inertial.mass - 1.0).abs() < 1e-6);
        assert!((base.inertial.ixx - 0.01).abs() < 1e-6);
    }

    #[test]
    fn visual_geometry_types() {
        let model = RobotModel::from_urdf(&fixture_urdf()).unwrap();

        let base = model.links.iter().find(|l| l.name == "base_link").unwrap();
        assert_eq!(base.visuals.len(), 1);
        assert!(matches!(base.visuals[0].geometry, GeomData::Box { .. }));

        let l1 = model.links.iter().find(|l| l.name == "link1").unwrap();
        assert!(matches!(l1.visuals[0].geometry, GeomData::Cylinder { .. }));

        let l2 = model.links.iter().find(|l| l.name == "link2").unwrap();
        assert!(matches!(l2.visuals[0].geometry, GeomData::Sphere { .. }));
    }

    #[test]
    fn materials_and_colors() {
        let model = RobotModel::from_urdf(&fixture_urdf()).unwrap();
        // base_link uses "gray" material → [0.5, 0.5, 0.5, 1.0]
        let base = model.links.iter().find(|l| l.name == "base_link").unwrap();
        let c = base.visuals[0].color;
        assert!((c[0] - 0.5).abs() < 1e-3);
        assert!((c[3] - 1.0).abs() < 1e-3);
    }

    #[test]
    fn joint_positions_initialized_to_zero() {
        let model = RobotModel::from_urdf(&fixture_urdf()).unwrap();
        assert_eq!(model.joint_positions.len(), model.joints.len());
        for &pos in &model.joint_positions {
            assert!((pos).abs() < 1e-10);
        }
    }

    #[test]
    fn source_path_stored() {
        let model = RobotModel::from_urdf(&fixture_urdf()).unwrap();
        assert!(model.source_path.is_some());
    }

    #[test]
    fn children_joints_structure() {
        let model = RobotModel::from_urdf(&fixture_urdf()).unwrap();
        // base_link has joint1 and fixed_joint as children
        let kids = model.children_joints.get("base_link").unwrap();
        assert_eq!(kids.len(), 2);
        // link1 has joint2 as child
        let kids1 = model.children_joints.get("link1").unwrap();
        assert_eq!(kids1.len(), 1);
    }

    #[test]
    fn parent_joint_of_link() {
        let model = RobotModel::from_urdf(&fixture_urdf()).unwrap();
        assert!(model.parent_joint_of_link("base_link").is_none());
        let ji = model.parent_joint_of_link("link1").unwrap();
        assert_eq!(model.joints[ji].name, "joint1");
    }

    // --- Transforms ---

    #[test]
    fn compute_transforms_at_zero() {
        let model = RobotModel::from_urdf(&fixture_urdf()).unwrap();
        let tf = model.compute_transforms();

        // Root link = identity
        let base_tf = tf.get("base_link").unwrap();
        assert!((base_tf.translation.vector.norm()).abs() < 1e-6);

        // link1 should be at z=0.05 (joint1 origin)
        let l1_tf = tf.get("link1").unwrap();
        assert!((l1_tf.translation.z - 0.05).abs() < 1e-4);

        // link2 should be at z=0.05+0.2=0.25 (joint1 + joint2 origins)
        let l2_tf = tf.get("link2").unwrap();
        assert!((l2_tf.translation.z - 0.25).abs() < 1e-4);
    }

    #[test]
    fn compute_transforms_with_joint_rotation() {
        let mut model = RobotModel::from_urdf(&fixture_urdf()).unwrap();
        // Set joint1 to 90 degrees (pi/2) around Y axis
        let ji = model.joints.iter().position(|j| j.name == "joint1").unwrap();
        model.joint_positions[ji] = std::f64::consts::FRAC_PI_2;

        let tf = model.compute_transforms();
        let l1_tf = tf.get("link1").unwrap();
        // After 90° rotation around Y at z=0.05,
        // link1 origin should still be at z=0.05 (rotation doesn't move the pivot)
        assert!((l1_tf.translation.z - 0.05).abs() < 1e-4);

        // link2 origin (at joint2 offset z=0.2 from link1) should now be shifted in X
        let l2_tf = tf.get("link2").unwrap();
        // Original z=0.2 offset rotated 90° around Y → x=0.2, z≈0
        assert!((l2_tf.translation.x - 0.2).abs() < 1e-3);
        assert!((l2_tf.translation.z - 0.05).abs() < 1e-3);
    }

    #[test]
    fn fixed_joint_child_transform() {
        let model = RobotModel::from_urdf(&fixture_urdf()).unwrap();
        let tf = model.compute_transforms();
        let fp_tf = tf.get("fixed_part").unwrap();
        assert!((fp_tf.translation.x - 0.1).abs() < 1e-4);
        assert!((fp_tf.translation.z).abs() < 1e-4);
    }

    // --- Bounding sphere ---

    #[test]
    fn bounding_sphere_base_link() {
        let model = RobotModel::from_urdf(&fixture_urdf()).unwrap();
        let li = model.link_map["base_link"];
        let (center, radius) = model.link_bounding_sphere(li);
        // Box 0.2×0.2×0.1 → half-extents 0.1, 0.1, 0.05
        // Diagonal = sqrt(0.1² + 0.1² + 0.05²) ≈ 0.15
        assert!(radius > 0.1);
        assert!(radius < 0.25);
        assert!(center.coords.norm() < 0.01); // centered at origin
    }

    #[test]
    fn bounding_sphere_empty_visuals() {
        // A link with no visuals should have zero radius
        let model = RobotModel::from_urdf(&fixture_urdf()).unwrap();
        // All links in fixture have visuals, but let's check link_bounding_sphere
        // handles the general case via the fixture anyway.
        for (i, link) in model.links.iter().enumerate() {
            let (_, r) = model.link_bounding_sphere(i);
            if link.visuals.is_empty() {
                assert!(r < 1e-6);
            } else {
                assert!(r > 0.0);
            }
        }
    }

    // --- Ray intersection ---

    #[test]
    fn ray_sphere_hit() {
        let origin = na::Point3::new(0.0, 0.0, -5.0_f32);
        let dir = na::Vector3::new(0.0, 0.0, 1.0);
        let center = na::Point3::origin();
        let result = ray_sphere_intersect(&origin, &dir, &center, 1.0);
        assert!(result.is_some());
        let t = result.unwrap();
        assert!((t - 4.0).abs() < 1e-4); // hits at z=-1 → distance=4
    }

    #[test]
    fn ray_sphere_miss() {
        let origin = na::Point3::new(5.0, 0.0, 0.0_f32);
        let dir = na::Vector3::new(0.0, 0.0, 1.0);
        let center = na::Point3::origin();
        assert!(ray_sphere_intersect(&origin, &dir, &center, 1.0).is_none());
    }

    #[test]
    fn ray_box_hit() {
        let origin = na::Point3::new(0.0, 0.0, -5.0_f32);
        let dir = na::Vector3::new(0.0, 0.0, 1.0);
        let result = ray_box_intersect(&origin, &dir, 1.0, 1.0, 1.0);
        assert!(result.is_some());
        let t = result.unwrap();
        assert!((t - 4.0).abs() < 1e-4); // hits at z=-1 → distance=4
    }

    #[test]
    fn ray_box_miss() {
        let origin = na::Point3::new(5.0, 0.0, 0.0_f32);
        let dir = na::Vector3::new(0.0, 0.0, 1.0);
        assert!(ray_box_intersect(&origin, &dir, 1.0, 1.0, 1.0).is_none());
    }

    #[test]
    fn ray_cylinder_hit_side() {
        let origin = na::Point3::new(5.0, 0.0, 0.0_f32);
        let dir = na::Vector3::new(-1.0, 0.0, 0.0);
        let result = ray_cylinder_intersect(&origin, &dir, 1.0, 2.0);
        assert!(result.is_some());
        let t = result.unwrap();
        assert!((t - 4.0).abs() < 1e-4);
    }

    #[test]
    fn ray_cylinder_hit_cap() {
        let origin = na::Point3::new(0.0, 0.0, 5.0_f32);
        let dir = na::Vector3::new(0.0, 0.0, -1.0);
        let result = ray_cylinder_intersect(&origin, &dir, 1.0, 2.0);
        assert!(result.is_some());
        let t = result.unwrap();
        assert!((t - 3.0).abs() < 1e-4); // cap at z=2 → distance=3
    }

    #[test]
    fn ray_triangle_hit() {
        let origin = na::Point3::new(0.25, 0.25, -1.0_f32);
        let dir = na::Vector3::new(0.0, 0.0, 1.0);
        let v0 = na::Point3::new(0.0, 0.0, 0.0);
        let v1 = na::Point3::new(1.0, 0.0, 0.0);
        let v2 = na::Point3::new(0.0, 1.0, 0.0);
        let result = ray_triangle_intersect(&origin, &dir, &v0, &v1, &v2);
        assert!(result.is_some());
        assert!((result.unwrap() - 1.0).abs() < 1e-4);
    }

    #[test]
    fn ray_triangle_miss() {
        let origin = na::Point3::new(2.0, 2.0, -1.0_f32);
        let dir = na::Vector3::new(0.0, 0.0, 1.0);
        let v0 = na::Point3::new(0.0, 0.0, 0.0);
        let v1 = na::Point3::new(1.0, 0.0, 0.0);
        let v2 = na::Point3::new(0.0, 1.0, 0.0);
        assert!(ray_triangle_intersect(&origin, &dir, &v0, &v1, &v2).is_none());
    }

    #[test]
    fn ray_mesh_intersect_with_flat_vertices() {
        // Two triangles forming a quad on the Z=0 plane
        #[rustfmt::skip]
        let verts: Vec<f32> = vec![
            // Triangle 1: (0,0,0) (1,0,0) (0,1,0) — normal (0,0,1)
            0.0, 0.0, 0.0,  0.0, 0.0, 1.0,
            1.0, 0.0, 0.0,  0.0, 0.0, 1.0,
            0.0, 1.0, 0.0,  0.0, 0.0, 1.0,
            // Triangle 2: (1,0,0) (1,1,0) (0,1,0) — normal (0,0,1)
            1.0, 0.0, 0.0,  0.0, 0.0, 1.0,
            1.0, 1.0, 0.0,  0.0, 0.0, 1.0,
            0.0, 1.0, 0.0,  0.0, 0.0, 1.0,
        ];
        let origin = na::Point3::new(0.5, 0.5, -2.0_f32);
        let dir = na::Vector3::new(0.0, 0.0, 1.0);
        let result = ray_mesh_intersect(&origin, &dir, &verts);
        assert!(result.is_some());
        assert!((result.unwrap() - 2.0).abs() < 1e-4);
    }

    // --- resolve_package_path ---

    #[test]
    fn resolve_package_path_basic() {
        let pkg = Path::new("/opt/ros/packages/my_robot");
        let result = resolve_package_path("package://my_robot/meshes/foo.stl", pkg);
        assert_eq!(result, PathBuf::from("/opt/ros/packages/my_robot/meshes/foo.stl"));
    }

    #[test]
    fn resolve_package_path_file_uri() {
        let pkg = Path::new("/tmp");
        let result = resolve_package_path("file:///absolute/path.stl", pkg);
        assert_eq!(result, PathBuf::from("/absolute/path.stl"));
    }

    #[test]
    fn resolve_package_path_relative() {
        let pkg = Path::new("/tmp");
        let result = resolve_package_path("meshes/foo.stl", pkg);
        assert_eq!(result, PathBuf::from("meshes/foo.stl"));
    }

    // --- URDF export round-trip ---

    #[test]
    fn export_urdf_roundtrip() {
        let model = RobotModel::from_urdf(&fixture_urdf()).unwrap();
        let xml = model.export_urdf().expect("export_urdf failed");
        assert!(xml.contains("<robot"));
        assert!(xml.contains("test_robot"));
        assert!(xml.contains("joint1"));
        assert!(xml.contains("base_link"));
    }

    // --- from_file format dispatcher ---

    #[test]
    fn from_file_urdf() {
        let model = RobotModel::from_file(&fixture_urdf()).expect("from_file URDF failed");
        assert_eq!(model.name, "test_robot");
    }

    #[test]
    fn from_file_sdf() {
        let model = RobotModel::from_file(&fixture_sdf()).expect("from_file SDF failed");
        assert_eq!(model.name, "test_sdf_robot");
    }

    #[test]
    fn from_file_mjcf() {
        let model = RobotModel::from_file(&fixture_mjcf()).expect("from_file MJCF failed");
        assert_eq!(model.name, "test_mjcf_robot");
    }

    #[test]
    fn from_file_unknown_extension() {
        let result = RobotModel::from_file(Path::new("/tmp/robot.png"));
        assert!(result.is_err());
    }

    // --- Namiashi full integration ---

    #[test]
    fn load_namiashi_urdf() {
        let path = namiashi_urdf();
        if !path.exists() {
            eprintln!("Skipping namiashi test — URDF not found at {:?}", path);
            return;
        }
        let model = RobotModel::from_urdf(&path).expect("Failed to load namiashi");
        assert_eq!(model.name, "namiashi_description");
        assert!(model.links.len() > 10);
        assert!(model.joints.len() > 10);
        assert_eq!(model.root_link, "trunk");

        // Verify meshes were loaded (trunk should have non-empty visual vertices)
        let trunk_inertia = model.links.iter().find(|l| l.name == "trunk_interia").unwrap();
        assert!(!trunk_inertia.visuals.is_empty());
        if let GeomData::Mesh { mesh, filename, .. } = &trunk_inertia.visuals[0].geometry {
            assert!(mesh.num_triangles() > 0, "trunk mesh vertices should not be empty");
            assert!(filename.as_ref().unwrap().contains("trunk.stl"));
        } else {
            panic!("Expected trunk_interia to have mesh geometry");
        }
    }

    #[test]
    fn namiashi_transforms_reasonable() {
        let path = namiashi_urdf();
        if !path.exists() { return; }
        let model = RobotModel::from_urdf(&path).unwrap();
        let tf = model.compute_transforms();

        // All transforms should exist for all links
        for link in &model.links {
            assert!(tf.contains_key(&link.name), "Missing transform for {}", link.name);
        }

        // All link positions should be within reasonable bounds (robot is ~0.5m)
        for (name, iso) in &tf {
            let pos = iso.translation.vector;
            assert!(pos.norm() < 2.0, "Link {} has unreasonable position: {:?}", name, pos);
        }
    }

    #[test]
    fn namiashi_pick_link() {
        let path = namiashi_urdf();
        if !path.exists() { return; }
        let model = RobotModel::from_urdf(&path).unwrap();
        let tf = model.compute_transforms();

        // Shoot a ray downward from above → should hit trunk
        let origin = na::Point3::new(0.0, 0.0, 1.0_f32);
        let dir = na::Vector3::new(0.0, 0.0, -1.0);
        let hit = model.pick_link(&origin, &dir, &tf);
        assert!(hit.is_some(), "Ray should hit a link when shooting at robot");
    }
}
