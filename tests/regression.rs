//! Regression tests for articara.
//!
//! Tests are organized by module with a shared helper for locating test fixtures.
//! Run with: cargo test

use std::path::{Path, PathBuf};

/// Return the absolute path of the `tests/fixtures` directory.
fn fixtures_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests").join("fixtures")
}

/// Return the fixture URDF path.
fn fixture_urdf() -> PathBuf {
    fixtures_dir().join("urdf").join("test_robot.urdf")
}

/// Return the fixture SDF path.
fn fixture_sdf() -> PathBuf {
    fixtures_dir().join("sdf").join("test_robot.sdf")
}

/// Return the fixture MJCF path.
fn fixture_mjcf() -> PathBuf {
    fixtures_dir().join("mjcf").join("test_robot.xml")
}

/// Return the real namiashi URDF path (for full integration tests).
fn namiashi_urdf() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("sample")
        .join("namiashi_description")
        .join("urdf")
        .join("namiashi.urdf")
}

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
        if let GeomData::Mesh { vertices, filename, .. } = &trunk_inertia.visuals[0].geometry {
            assert!(!vertices.is_empty(), "trunk mesh vertices should not be empty");
            assert!(filename.as_ref().unwrap().contains("trunk.STL"));
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

// ============================================================
// format.rs — format detection
// ============================================================
mod test_format {
    use articara::format::RobotFormat;
    use std::path::Path;

    #[test]
    fn detect_urdf_extension() {
        let fmt = RobotFormat::detect_from_extension(Path::new("robot.urdf"));
        assert_eq!(fmt, Some(RobotFormat::Urdf));
    }

    #[test]
    fn detect_xacro_extension() {
        let fmt = RobotFormat::detect_from_extension(Path::new("robot.xacro"));
        assert_eq!(fmt, Some(RobotFormat::Urdf));
    }

    #[test]
    fn detect_sdf_extension() {
        let fmt = RobotFormat::detect_from_extension(Path::new("model.sdf"));
        assert_eq!(fmt, Some(RobotFormat::Sdf));
    }

    #[test]
    fn detect_world_extension() {
        let fmt = RobotFormat::detect_from_extension(Path::new("scene.world"));
        assert_eq!(fmt, Some(RobotFormat::Sdf));
    }

    #[test]
    fn detect_xml_extension() {
        let fmt = RobotFormat::detect_from_extension(Path::new("robot.xml"));
        assert_eq!(fmt, Some(RobotFormat::Mjcf));
    }

    #[test]
    fn detect_unknown_extension() {
        let fmt = RobotFormat::detect_from_extension(Path::new("image.png"));
        assert_eq!(fmt, None);
    }

    #[test]
    fn detect_no_extension() {
        let fmt = RobotFormat::detect_from_extension(Path::new("robot"));
        assert_eq!(fmt, None);
    }

    #[test]
    fn supports_import() {
        assert!(RobotFormat::Urdf.supports_import());
        assert!(RobotFormat::Sdf.supports_import());
        assert!(RobotFormat::Mjcf.supports_import());
        assert!(RobotFormat::IsaacUsd.supports_import());
    }

    #[test]
    fn supports_export() {
        for &fmt in RobotFormat::ALL {
            assert!(fmt.supports_export());
        }
    }

    #[test]
    fn all_contains_four() {
        assert_eq!(RobotFormat::ALL.len(), 4);
    }

    #[test]
    fn labels_non_empty() {
        for &fmt in RobotFormat::ALL {
            assert!(!fmt.label().is_empty());
        }
    }

    #[test]
    fn extensions_non_empty() {
        for &fmt in RobotFormat::ALL {
            assert!(!fmt.extension().is_empty());
        }
    }

    #[test]
    fn display_trait() {
        assert_eq!(format!("{}", RobotFormat::Urdf), "URDF");
        assert_eq!(format!("{}", RobotFormat::Mjcf), "MJCF");
    }

    #[test]
    fn detect_from_fixture_sdf() {
        use super::fixture_sdf;
        let fmt = RobotFormat::detect(&fixture_sdf());
        assert_eq!(fmt, Some(RobotFormat::Sdf));
    }

    #[test]
    fn detect_from_fixture_mjcf() {
        use super::fixture_mjcf;
        // .xml file containing <mujoco → should detect as MJCF
        let fmt = RobotFormat::detect(&fixture_mjcf());
        assert_eq!(fmt, Some(RobotFormat::Mjcf));
    }
}

// ============================================================
// sdf.rs — SDF import and export
// ============================================================
mod test_sdf {
    use super::*;
    use articara::robot::*;
    use articara::sdf;

    #[test]
    fn import_sdf_basic() {
        let model = sdf::import_sdf(&fixture_sdf()).expect("SDF import failed");
        assert_eq!(model.name, "test_sdf_robot");
        assert_eq!(model.links.len(), 3);
        assert_eq!(model.joints.len(), 2);
        assert_eq!(model.root_link, "base_link");
    }

    #[test]
    fn sdf_link_inertial() {
        let model = sdf::import_sdf(&fixture_sdf()).unwrap();
        let base = model.links.iter().find(|l| l.name == "base_link").unwrap();
        assert!((base.inertial.mass - 1.0).abs() < 1e-6);
        assert!((base.inertial.ixx - 0.01).abs() < 1e-6);
    }

    #[test]
    fn sdf_visual_geometry() {
        let model = sdf::import_sdf(&fixture_sdf()).unwrap();
        let base = model.links.iter().find(|l| l.name == "base_link").unwrap();
        assert_eq!(base.visuals.len(), 1);
        match &base.visuals[0].geometry {
            GeomData::Box { hx, hy, hz } => {
                assert!((hx - 0.1).abs() < 1e-4);
                assert!((hy - 0.1).abs() < 1e-4);
                assert!((hz - 0.05).abs() < 1e-4);
            }
            _ => panic!("Expected Box geometry"),
        }
    }

    #[test]
    fn sdf_cylinder_geometry() {
        let model = sdf::import_sdf(&fixture_sdf()).unwrap();
        let l1 = model.links.iter().find(|l| l.name == "link1").unwrap();
        match &l1.visuals[0].geometry {
            GeomData::Cylinder { radius, half_length } => {
                assert!((*radius - 0.02).abs() < 1e-4);
                assert!((*half_length - 0.1).abs() < 1e-4);
            }
            _ => panic!("Expected Cylinder geometry"),
        }
    }

    #[test]
    fn sdf_sphere_geometry() {
        let model = sdf::import_sdf(&fixture_sdf()).unwrap();
        let l2 = model.links.iter().find(|l| l.name == "link2").unwrap();
        match &l2.visuals[0].geometry {
            GeomData::Sphere { radius } => {
                assert!((*radius - 0.03).abs() < 1e-4);
            }
            _ => panic!("Expected Sphere geometry"),
        }
    }

    #[test]
    fn sdf_joint_properties() {
        let model = sdf::import_sdf(&fixture_sdf()).unwrap();
        let j1 = model.joints.iter().find(|j| j.name == "joint1").unwrap();
        assert_eq!(j1.joint_type, "revolute");
        assert_eq!(j1.parent_link, "base_link");
        assert_eq!(j1.child_link, "link1");
        assert!((j1.lower - (-1.57)).abs() < 1e-4);
        assert!((j1.upper - 1.57).abs() < 1e-4);
    }

    #[test]
    fn sdf_visual_color() {
        let model = sdf::import_sdf(&fixture_sdf()).unwrap();
        let l1 = model.links.iter().find(|l| l.name == "link1").unwrap();
        let c = l1.visuals[0].color;
        assert!((c[0] - 1.0).abs() < 1e-3); // red=1
        assert!((c[1]).abs() < 1e-3);         // green=0
    }

    #[test]
    fn sdf_collision_parsed() {
        let model = sdf::import_sdf(&fixture_sdf()).unwrap();
        let base = model.links.iter().find(|l| l.name == "base_link").unwrap();
        assert_eq!(base.collisions.len(), 1);
    }

    #[test]
    fn sdf_source_path() {
        let model = sdf::import_sdf(&fixture_sdf()).unwrap();
        assert!(model.source_path.is_some());
    }

    #[test]
    fn export_sdf_contains_model() {
        let model = sdf::import_sdf(&fixture_sdf()).unwrap();
        let xml = sdf::export_sdf(&model);
        assert!(xml.contains("<sdf"));
        assert!(xml.contains("<model name=\"test_sdf_robot\""));
        assert!(xml.contains("<link name=\"base_link\""));
        assert!(xml.contains("<joint name=\"joint1\""));
        assert!(xml.contains("<mass>"));
    }

    #[test]
    fn sdf_roundtrip_data_preserved() {
        let model = sdf::import_sdf(&fixture_sdf()).unwrap();
        let xml = sdf::export_sdf(&model);

        // Write to temp, re-import
        let tmp = std::env::temp_dir().join("articara_test_sdf_roundtrip.sdf");
        std::fs::write(&tmp, &xml).unwrap();
        let model2 = sdf::import_sdf(&tmp).expect("Re-import SDF failed");
        std::fs::remove_file(&tmp).ok();

        assert_eq!(model.name, model2.name);
        assert_eq!(model.links.len(), model2.links.len());
        assert_eq!(model.joints.len(), model2.joints.len());
        assert_eq!(model.root_link, model2.root_link);

        // Check inertial mass round-trips
        for (a, b) in model.links.iter().zip(model2.links.iter()) {
            assert!((a.inertial.mass - b.inertial.mass).abs() < 1e-4,
                "Mass mismatch for {}: {} vs {}", a.name, a.inertial.mass, b.inertial.mass);
        }
    }

    // ---- Closed-loop models ----

    fn fixture_four_bar() -> PathBuf {
        fixtures_dir().join("sdf").join("four_bar.sdf")
    }

    fn fixture_five_bar() -> PathBuf {
        fixtures_dir().join("sdf").join("five_bar_parallel.sdf")
    }

    #[test]
    fn four_bar_loads() {
        let model = sdf::import_sdf(&fixture_four_bar()).expect("four_bar import failed");
        assert_eq!(model.name, "four_bar_linkage");
        assert_eq!(model.links.len(), 4);
        assert_eq!(model.joints.len(), 3);
        assert_eq!(model.root_link, "base_link");
    }

    #[test]
    fn four_bar_loop_closed_at_q0() {
        let model = sdf::import_sdf(&fixture_four_bar()).unwrap();
        let transforms = model.compute_transforms();

        // Coupler tip = coupler origin + (0.3, 0, 0) in coupler frame
        let coupler_tf = transforms["coupler"];
        let coupler_tip = coupler_tf * nalgebra::Point3::new(0.3_f32, 0.0, 0.0);

        // Crank-right tip = crank_right origin + (0, 0, 0.2) in crank_right frame
        let crank_r_tf = transforms["crank_right"];
        let crank_r_tip = crank_r_tf * nalgebra::Point3::new(0.0_f32, 0.0, 0.2);

        let err = (coupler_tip - crank_r_tip).norm();
        assert!(
            err < 0.01,
            "Four-bar loop not closed at q=0: coupler_tip={:?} crank_r_tip={:?} err={}",
            coupler_tip, crank_r_tip, err
        );
    }

    #[test]
    fn five_bar_loads() {
        let model = sdf::import_sdf(&fixture_five_bar()).expect("five_bar import failed");
        assert_eq!(model.name, "five_bar_parallel");
        assert_eq!(model.links.len(), 6);  // base + 2 proximal + 2 distal + EE
        // 4 revolute + 1 fixed = 5 joints
        assert_eq!(model.joints.len(), 5);
        assert_eq!(model.root_link, "base_link");
    }

    #[test]
    fn five_bar_loop_closed_at_q0() {
        let model = sdf::import_sdf(&fixture_five_bar()).unwrap();
        let transforms = model.compute_transforms();

        // EE position (fixed to distal_left tip)
        let ee_pos = transforms["end_effector"]
            * nalgebra::Point3::new(0.0_f32, 0.0, 0.0);

        // Distal-right tip = distal_right origin + (0, 0, 0.2) in its frame
        let dr_tf = transforms["distal_right"];
        let dr_tip = dr_tf * nalgebra::Point3::new(0.0_f32, 0.0, 0.2);

        let err = (ee_pos - dr_tip).norm();
        assert!(
            err < 0.01,
            "Five-bar loop not closed at q=0: ee={:?} dr_tip={:?} err={}",
            ee_pos, dr_tip, err
        );
    }
}

// ============================================================
// mjcf.rs — MJCF import and export
// ============================================================
mod test_mjcf {
    use super::*;
    use articara::mjcf;
    use articara::robot::*;

    #[test]
    fn import_mjcf_basic() {
        let model = mjcf::import_mjcf(&fixture_mjcf()).expect("MJCF import failed");
        assert_eq!(model.name, "test_mjcf_robot");
        // base_link, link1, link2
        assert!(model.links.len() >= 3);
        // joint1, joint2
        assert!(model.joints.len() >= 2);
    }

    #[test]
    fn mjcf_link_names() {
        let model = mjcf::import_mjcf(&fixture_mjcf()).unwrap();
        let names: Vec<&str> = model.links.iter().map(|l| l.name.as_str()).collect();
        assert!(names.contains(&"base_link"));
        assert!(names.contains(&"link1"));
        assert!(names.contains(&"link2"));
    }

    #[test]
    fn mjcf_joint_properties() {
        let model = mjcf::import_mjcf(&fixture_mjcf()).unwrap();
        let j1 = model.joints.iter().find(|j| j.name == "joint1").unwrap();
        assert_eq!(j1.joint_type, "revolute");
        assert!((j1.lower - (-1.57)).abs() < 1e-4);
        assert!((j1.upper - 1.57).abs() < 1e-4);
    }

    #[test]
    fn mjcf_inertial() {
        let model = mjcf::import_mjcf(&fixture_mjcf()).unwrap();
        let base = model.links.iter().find(|l| l.name == "base_link").unwrap();
        assert!((base.inertial.mass - 1.0).abs() < 1e-6);
    }

    #[test]
    fn mjcf_visual_geometry() {
        let model = mjcf::import_mjcf(&fixture_mjcf()).unwrap();
        let base = model.links.iter().find(|l| l.name == "base_link").unwrap();
        assert!(!base.visuals.is_empty());
        assert!(matches!(&base.visuals[0].geometry, GeomData::Box { .. }));
    }

    #[test]
    fn export_mjcf_contains_mujoco() {
        let model = mjcf::import_mjcf(&fixture_mjcf()).unwrap();
        let xml = mjcf::export_mjcf(&model);
        assert!(xml.contains("<mujoco"));
        assert!(xml.contains("test_mjcf_robot"));
        assert!(xml.contains("joint1"));
    }

    #[test]
    fn mjcf_roundtrip_data_preserved() {
        let model = mjcf::import_mjcf(&fixture_mjcf()).unwrap();
        let xml = mjcf::export_mjcf(&model);

        let tmp = std::env::temp_dir().join("articara_test_mjcf_roundtrip.xml");
        std::fs::write(&tmp, &xml).unwrap();
        let model2 = mjcf::import_mjcf(&tmp).expect("Re-import MJCF failed");
        std::fs::remove_file(&tmp).ok();

        assert_eq!(model.name, model2.name);
        assert_eq!(model.links.len(), model2.links.len());
        assert_eq!(model.joints.len(), model2.joints.len());
    }
}

// ============================================================
// isaac.rs — Isaac export
// ============================================================
mod test_isaac {
    use super::*;
    use articara::isaac;
    use articara::robot::RobotModel;

    #[test]
    fn export_isaac_python_script() {
        let model = RobotModel::from_urdf(&fixture_urdf()).unwrap();
        let script = isaac::export_isaac_python(&model, "urdf/test_robot.urdf");
        assert!(script.contains("import omni"));
        assert!(script.contains("test_robot"));
        assert!(script.contains("URDF_PATH"));
        assert!(script.contains("urdf/test_robot.urdf"));
        assert!(script.contains("DriveAPI"));
    }

    #[test]
    fn isaac_script_has_joint_config() {
        let model = RobotModel::from_urdf(&fixture_urdf()).unwrap();
        let script = isaac::export_isaac_python(&model, "urdf/test_robot.urdf");
        // joint1 and joint2 are revolute → should have angular drive
        assert!(script.contains("joint1"));
        assert!(script.contains("joint2"));
        assert!(script.contains("angular"));
        // fixed_joint should be skipped from drive configuration
        assert!(!script.contains("# Joint: fixed_joint"),
            "fixed_joint should be skipped in drive config");
    }

    #[test]
    fn isaac_script_has_physics_scene() {
        let model = RobotModel::from_urdf(&fixture_urdf()).unwrap();
        let script = isaac::export_isaac_python(&model, "urdf/test_robot.urdf");
        assert!(script.contains("PhysicsScene"));
        assert!(script.contains("gravityDirection"));
    }

    #[test]
    fn export_isaac_to_dir_creates_files() {
        let model = RobotModel::from_urdf(&fixture_urdf()).unwrap();
        let tmp = std::env::temp_dir().join("articara_isaac_test");
        let _ = std::fs::remove_dir_all(&tmp);
        std::fs::create_dir_all(&tmp).unwrap();

        let result = isaac::export_isaac_to_dir(&model, &tmp);
        assert!(result.is_ok(), "Isaac export failed: {:?}", result.err());

        // Check URDF was created
        let urdf_dir = tmp.join("urdf");
        assert!(urdf_dir.exists());
        let urdf_file = urdf_dir.join("test_robot.urdf");
        assert!(urdf_file.exists(), "URDF file not created");

        // Check Python script was created
        let script_file = tmp.join("import_test_robot.py");
        assert!(script_file.exists(), "Python script not created");

        let _ = std::fs::remove_dir_all(&tmp);
    }
}

// ============================================================
// usd.rs — USD ASCII export
// ============================================================
mod test_usd {
    use super::*;
    use articara::robot::RobotModel;
    use articara::usd;

    #[test]
    fn export_usda_header() {
        let model = RobotModel::new_empty("my_robot");
        let usda = usd::export_usda(&model);
        assert!(usda.starts_with("#usda 1.0"));
        assert!(usda.contains("defaultPrim = \"World\""));
        assert!(usda.contains("metersPerUnit = 1.0"));
        assert!(usda.contains("upAxis = \"Z\""));
    }

    #[test]
    fn export_usda_robot_prim() {
        let model = RobotModel::new_empty("my_robot");
        let usda = usd::export_usda(&model);
        assert!(usda.contains("def Xform \"my_robot\""));
        assert!(usda.contains("PhysicsArticulationRootAPI"));
    }

    #[test]
    fn export_usda_link_physics_apis() {
        let model = RobotModel::new_empty("my_robot");
        let usda = usd::export_usda(&model);
        assert!(usda.contains("PhysicsRigidBodyAPI"));
        assert!(usda.contains("PhysicsMassAPI"));
        assert!(usda.contains("physics:mass"));
    }

    #[test]
    fn export_usda_physics_scene() {
        let model = RobotModel::new_empty("my_robot");
        let usda = usd::export_usda(&model);
        assert!(usda.contains("def PhysicsScene \"PhysicsScene\""));
        assert!(usda.contains("physics:gravityDirection"));
        assert!(usda.contains("physics:gravityMagnitude"));
    }

    #[test]
    fn export_usda_box_geometry() {
        let model = RobotModel::new_empty("my_robot");
        let usda = usd::export_usda(&model);
        // new_empty uses a Box visual
        assert!(usda.contains("def Cube \"visual_0\""));
        assert!(usda.contains("double size = 2.0"));
    }

    #[test]
    fn export_usda_material() {
        let model = RobotModel::new_empty("my_robot");
        let usda = usd::export_usda(&model);
        assert!(usda.contains("def Scope \"Materials\""));
        assert!(usda.contains("def Material \"material_0\""));
        assert!(usda.contains("UsdPreviewSurface"));
        assert!(usda.contains("inputs:diffuseColor"));
    }

    #[test]
    fn export_usda_fixture_robot() {
        let model = RobotModel::from_urdf(&fixture_urdf()).unwrap();
        let usda = usd::export_usda(&model);

        // All links present
        assert!(usda.contains("def Xform \"base_link\""));
        assert!(usda.contains("def Xform \"link1\""));
        assert!(usda.contains("def Xform \"link2\""));

        // Joints present
        assert!(usda.contains("PhysicsRevoluteJoint"));
        assert!(usda.contains("physics:axis"));
        assert!(usda.contains("physics:body0"));
        assert!(usda.contains("physics:body1"));
        assert!(usda.contains("physics:localPos0"));
        assert!(usda.contains("physics:localRot0"));
    }

    #[test]
    fn export_usda_joint_limits_in_degrees() {
        let model = RobotModel::from_urdf(&fixture_urdf()).unwrap();
        let usda = usd::export_usda(&model);
        // The fixture robot has revolute joints with limits in radians.
        // USD should have them converted to degrees.
        assert!(usda.contains("physics:lowerLimit"));
        assert!(usda.contains("physics:upperLimit"));
    }

    #[test]
    fn export_usda_fixed_joint() {
        let model = RobotModel::from_urdf(&fixture_urdf()).unwrap();
        let usda = usd::export_usda(&model);
        // The fixture has a fixed joint
        assert!(usda.contains("PhysicsFixedJoint"));
    }

    #[test]
    fn export_usda_drive_properties() {
        let model = RobotModel::from_urdf(&fixture_urdf()).unwrap();
        let usda = usd::export_usda(&model);
        // Revolute joints should have angular drive
        assert!(usda.contains("PhysicsDriveAPI:angular"));
        assert!(usda.contains("drive:angular:physics:damping"));
        assert!(usda.contains("drive:angular:physics:stiffness"));
    }

    #[test]
    fn export_usda_collision_api() {
        let model = RobotModel::from_urdf(&fixture_urdf()).unwrap();
        let usda = usd::export_usda(&model);
        // Collisions should have PhysicsCollisionAPI
        assert!(usda.contains("PhysicsCollisionAPI"));
    }

    #[test]
    fn export_usda_to_dir_creates_file() {
        let model = RobotModel::from_urdf(&fixture_urdf()).unwrap();
        let tmp = std::env::temp_dir().join("articara_usda_test");
        let _ = std::fs::remove_dir_all(&tmp);
        std::fs::create_dir_all(&tmp).unwrap();

        let result = usd::export_usda_to_dir(&model, &tmp);
        assert!(result.is_ok(), "USDA export failed: {:?}", result.err());

        let path = result.unwrap();
        assert!(path.exists(), "USDA file not created: {:?}", path);
        assert!(
            path.extension().unwrap_or_default() == "usda",
            "Wrong extension: {:?}",
            path
        );

        // Verify content
        let content = std::fs::read_to_string(&path).unwrap();
        assert!(content.starts_with("#usda 1.0"));

        let _ = std::fs::remove_dir_all(&tmp);
    }

    #[test]
    fn export_usda_sanitises_names() {
        // Robot name with special characters
        let model = RobotModel::new_empty("my-robot.v2");
        let usda = usd::export_usda(&model);
        // Name should be sanitised (hyphens/dots → underscores)
        assert!(usda.contains("def Xform \"my_robot_v2\""));
    }

    #[test]
    fn export_usda_namiashi() {
        let namiashi = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("test_fixtures/namiashi/urdf/namiashi.urdf");
        if !namiashi.exists() {
            return; // skip if fixture not available
        }
        let model = RobotModel::from_urdf(&namiashi).unwrap();
        let usda = usd::export_usda(&model);

        // Should have a Mesh geometry for STL links
        assert!(usda.contains("def Mesh"));
        assert!(usda.contains("point3f[] points"));
        assert!(usda.contains("faceVertexCounts"));
        assert!(usda.contains("faceVertexIndices"));
    }
}

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
// Cross-format integration tests
// ============================================================
mod test_cross_format {
    use super::*;
    use articara::robot::RobotModel;

    /// Load URDF, export to SDF, re-import → verify data preservation.
    #[test]
    fn urdf_to_sdf_roundtrip() {
        let model = RobotModel::from_urdf(&fixture_urdf()).unwrap();
        let sdf_xml = articara::sdf::export_sdf(&model);

        let tmp = std::env::temp_dir().join("articara_urdf2sdf.sdf");
        std::fs::write(&tmp, &sdf_xml).unwrap();
        let model2 = articara::sdf::import_sdf(&tmp).expect("SDF re-import failed");
        std::fs::remove_file(&tmp).ok();

        assert_eq!(model.links.len(), model2.links.len());
        assert_eq!(model.joints.len(), model2.joints.len());

        for (a, b) in model.links.iter().zip(model2.links.iter()) {
            assert_eq!(a.name, b.name);
            assert!((a.inertial.mass - b.inertial.mass).abs() < 1e-3,
                "Mass mismatch for {}: {} vs {}", a.name, a.inertial.mass, b.inertial.mass);
        }
    }

    /// Load URDF, export to MJCF, re-import → verify data preservation.
    #[test]
    fn urdf_to_mjcf_roundtrip() {
        let model = RobotModel::from_urdf(&fixture_urdf()).unwrap();
        let mjcf_xml = articara::mjcf::export_mjcf(&model);

        let tmp = std::env::temp_dir().join("articara_urdf2mjcf.xml");
        std::fs::write(&tmp, &mjcf_xml).unwrap();
        let model2 = articara::mjcf::import_mjcf(&tmp).expect("MJCF re-import failed");
        std::fs::remove_file(&tmp).ok();

        // MJCF might reorder or rename things, so check counts and find by name
        assert_eq!(model.links.len(), model2.links.len());
        for a in &model.links {
            let b = model2.links.iter().find(|l| l.name == a.name);
            assert!(b.is_some(), "Link {} missing after URDF→MJCF roundtrip", a.name);
            let b = b.unwrap();
            assert!((a.inertial.mass - b.inertial.mass).abs() < 1e-3,
                "Mass mismatch for {}: {} vs {}", a.name, a.inertial.mass, b.inertial.mass);
        }
    }

    /// Verify all three importable formats produce valid transforms.
    #[test]
    fn all_formats_produce_valid_transforms() {
        let paths = [fixture_urdf(), fixture_sdf(), fixture_mjcf()];
        for path in &paths {
            let model = RobotModel::from_file(path)
                .unwrap_or_else(|e| panic!("Failed to load {:?}: {}", path, e));
            let tf = model.compute_transforms();
            assert!(!tf.is_empty(), "Empty transforms for {:?}", path);
            for (name, iso) in &tf {
                assert!(iso.translation.vector.norm() < 10.0,
                    "Unreasonable position for {} in {:?}", name, path);
            }
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

// =========================================================================
//  Inertia computation tests
// =========================================================================

mod test_inertia {
    use articara::robot::*;

    #[test]
    fn box_inertia_values() {
        // 1kg box with half-extents 0.1, 0.2, 0.3  (full: 0.2, 0.4, 0.6)
        let geom = GeomData::Box { hx: 0.1, hy: 0.2, hz: 0.3 };
        let i = compute_geometry_inertia(&geom, 1.0);
        // Ixx = m/12 * (b² + c²) = (0.4² + 0.6²) / 12 = 0.0433..
        let expected_ixx = (0.4f64.powi(2) + 0.6f64.powi(2)) / 12.0;
        let expected_iyy = (0.2f64.powi(2) + 0.6f64.powi(2)) / 12.0;
        let expected_izz = (0.2f64.powi(2) + 0.4f64.powi(2)) / 12.0;
        assert!((i.ixx - expected_ixx).abs() < 1e-6, "ixx={}", i.ixx);
        assert!((i.iyy - expected_iyy).abs() < 1e-6, "iyy={}", i.iyy);
        assert!((i.izz - expected_izz).abs() < 1e-6, "izz={}", i.izz);
        assert!((i.ixy).abs() < 1e-10);
        assert!((i.ixz).abs() < 1e-10);
        assert!((i.iyz).abs() < 1e-10);
    }

    #[test]
    fn cylinder_inertia_values() {
        // 2kg cylinder, radius=0.05, half_length=0.1  (height=0.2)
        let geom = GeomData::Cylinder { radius: 0.05, half_length: 0.1 };
        let i = compute_geometry_inertia(&geom, 2.0);
        let r2 = 0.05f64.powi(2);
        let h2 = 0.2f64.powi(2);
        let expected_ixx = 2.0 / 12.0 * (3.0 * r2 + h2);
        let expected_izz = 2.0 / 2.0 * r2;
        assert!((i.ixx - expected_ixx).abs() < 1e-6, "ixx={}", i.ixx);
        assert!((i.iyy - expected_ixx).abs() < 1e-6, "iyy={}", i.iyy);
        assert!((i.izz - expected_izz).abs() < 1e-6, "izz={}", i.izz);
    }

    #[test]
    fn sphere_inertia_values() {
        // 3kg sphere, radius=0.1
        let geom = GeomData::Sphere { radius: 0.1 };
        let i = compute_geometry_inertia(&geom, 3.0);
        let expected = 2.0 / 5.0 * 3.0 * 0.1f64.powi(2);
        assert!((i.ixx - expected).abs() < 1e-6);
        assert!((i.iyy - expected).abs() < 1e-6);
        assert!((i.izz - expected).abs() < 1e-6);
    }

    #[test]
    fn volume_box() {
        let geom = GeomData::Box { hx: 0.5, hy: 0.5, hz: 0.5 };
        let vol = compute_geometry_volume(&geom);
        assert!((vol - 1.0).abs() < 1e-6); // 1x1x1 = 1 m³
    }

    #[test]
    fn volume_cylinder() {
        let geom = GeomData::Cylinder { radius: 1.0, half_length: 0.5 };
        let vol = compute_geometry_volume(&geom);
        assert!((vol - std::f64::consts::PI).abs() < 1e-6); // π×1²×1
    }

    #[test]
    fn volume_sphere() {
        let geom = GeomData::Sphere { radius: 1.0 };
        let vol = compute_geometry_volume(&geom);
        let expected = 4.0 / 3.0 * std::f64::consts::PI;
        assert!((vol - expected).abs() < 1e-6);
    }

    #[test]
    fn combined_inertia_single_centered_visual() {
        // A single visual at origin should produce the same inertia as
        // compute_geometry_inertia, with CoM at origin.
        use nalgebra as na;
        let vis = VisualData {
            origin: na::Isometry3::identity(),
            geometry: GeomData::Box { hx: 0.1, hy: 0.1, hz: 0.1 },
            color: [0.7, 0.7, 0.7, 1.0],
        };
        let result = compute_link_inertia(&[vis], 1.0);
        assert!((result.mass - 1.0).abs() < 1e-15);
        assert!(result.origin.translation.vector.norm() < 1e-10); // CoM at origin
        let expected = compute_geometry_inertia(&GeomData::Box { hx: 0.1, hy: 0.1, hz: 0.1 }, 1.0);
        assert!((result.ixx - expected.ixx).abs() < 1e-10);
        assert!((result.iyy - expected.iyy).abs() < 1e-10);
        assert!((result.izz - expected.izz).abs() < 1e-10);
    }

    #[test]
    fn combined_inertia_offset_visual() {
        // A visual offset along X should shift CoM and increase Iyy, Izz
        // via the parallel axis theorem.
        use nalgebra as na;
        let vis = VisualData {
            origin: na::Isometry3::from_parts(
                na::Translation3::new(1.0, 0.0, 0.0),
                na::UnitQuaternion::identity(),
            ),
            geometry: GeomData::Sphere { radius: 0.1 },
            color: [1.0, 0.0, 0.0, 1.0],
        };
        let result = compute_link_inertia(&[vis], 2.0);
        // CoM should be at (1, 0, 0)
        assert!((result.origin.translation.x - 1.0).abs() < 1e-6);
        assert!(result.origin.translation.y.abs() < 1e-6);
        // With CoM exactly at the sphere center, the inertia should be
        // the same as the sphere inertia (no parallel axis shift).
        let expected = compute_geometry_inertia(&GeomData::Sphere { radius: 0.1 }, 2.0);
        assert!((result.ixx - expected.ixx).abs() < 1e-8);
        assert!((result.iyy - expected.iyy).abs() < 1e-8);
    }

    #[test]
    fn combined_inertia_two_visuals_parallel_axis() {
        // Two identical spheres at ±0.5 on X axis should have:
        // - CoM at origin
        // - Iyy, Izz increased by parallel axis theorem
        use nalgebra as na;
        let vis1 = VisualData {
            origin: na::Isometry3::from_parts(
                na::Translation3::new(0.5, 0.0, 0.0),
                na::UnitQuaternion::identity(),
            ),
            geometry: GeomData::Sphere { radius: 0.1 },
            color: [0.7, 0.7, 0.7, 1.0],
        };
        let vis2 = VisualData {
            origin: na::Isometry3::from_parts(
                na::Translation3::new(-0.5, 0.0, 0.0),
                na::UnitQuaternion::identity(),
            ),
            geometry: GeomData::Sphere { radius: 0.1 },
            color: [0.7, 0.7, 0.7, 1.0],
        };
        let result = compute_link_inertia(&[vis1, vis2], 2.0);
        // CoM at origin
        assert!(result.origin.translation.vector.norm() < 1e-6);
        // Each sphere: mass=1.0, sphere inertia + parallel axis shift d=0.5
        let i_sphere = compute_geometry_inertia(&GeomData::Sphere { radius: 0.1 }, 1.0);
        // Iyy for each = i_sphere.iyy + 1.0 * 0.5² = i_sphere.iyy + 0.25
        let expected_iyy = 2.0 * (i_sphere.iyy + 1.0 * 0.25);
        // Ixx should have NO parallel axis shift (displacement along X, so d_y=d_z=0)
        let expected_ixx = 2.0 * i_sphere.ixx;
        assert!(
            (result.ixx - expected_ixx).abs() < 1e-8,
            "ixx: got {} expected {}",
            result.ixx,
            expected_ixx
        );
        assert!(
            (result.iyy - expected_iyy).abs() < 1e-8,
            "iyy: got {} expected {}",
            result.iyy,
            expected_iyy
        );
    }

    #[test]
    fn inertia_scales_with_mass() {
        let geom = GeomData::Box { hx: 0.1, hy: 0.1, hz: 0.1 };
        let i1 = compute_geometry_inertia(&geom, 1.0);
        let i2 = compute_geometry_inertia(&geom, 5.0);
        assert!((i2.ixx / i1.ixx - 5.0).abs() < 1e-6);
        assert!((i2.iyy / i1.iyy - 5.0).abs() < 1e-6);
        assert!((i2.izz / i1.izz - 5.0).abs() < 1e-6);
    }
}

// ========== Inertia Validation Tests ==========

mod test_inertia_validation {
    use articara::robot::*;

    fn make_link(name: &str, mass: f64, ixx: f64, iyy: f64, izz: f64,
                 ixy: f64, ixz: f64, iyz: f64) -> LinkData {
        LinkData {
            name: name.to_string(),
            visuals: vec![],
            collisions: vec![],
            inertial: InertialData {
                origin: nalgebra::Isometry3::identity(),
                mass,
                ixx, ixy, ixz,
                iyy, iyz, izz,
            },
        }
    }

    #[test]
    fn valid_box_inertia_passes() {
        // 1 kg box 0.2×0.2×0.2 m
        let m = 1.0;
        let s = 0.2_f64;
        let i_diag = m / 12.0 * (s * s + s * s); // ~0.006667
        let link = make_link("box", m, i_diag, i_diag, i_diag, 0.0, 0.0, 0.0);
        let v = validate_inertia(&link);
        assert!(v.is_ok(), "Expected OK, got: {:?}", v.issues);
    }

    #[test]
    fn negative_mass_is_error() {
        let link = make_link("bad", -1.0, 0.01, 0.01, 0.01, 0.0, 0.0, 0.0);
        let v = validate_inertia(&link);
        assert!(v.has_errors());
        assert!(v.issues.iter().any(|i| i.message.contains("negative")));
    }

    #[test]
    fn zero_mass_is_warning() {
        let link = make_link("dummy", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        let v = validate_inertia(&link);
        assert!(v.has_warnings());
        assert!(!v.has_errors());
    }

    #[test]
    fn negative_diagonal_is_error() {
        let link = make_link("bad_ixx", 1.0, -0.01, 0.01, 0.01, 0.0, 0.0, 0.0);
        let v = validate_inertia(&link);
        assert!(v.has_errors());
        assert!(v.issues.iter().any(|i| i.message.contains("Ixx") && i.message.contains("negative")));
    }

    #[test]
    fn triangle_inequality_violation() {
        // Izz much larger than Ixx + Iyy
        let link = make_link("bad_tri", 1.0, 0.001, 0.001, 1.0, 0.0, 0.0, 0.0);
        let v = validate_inertia(&link);
        assert!(v.has_errors());
        assert!(v.issues.iter().any(|i| i.message.contains("Triangle inequality")));
    }

    #[test]
    fn triangle_inequality_satisfied() {
        // Uniform sphere: all diagonal elements equal
        let link = make_link("sphere", 1.0, 0.01, 0.01, 0.01, 0.0, 0.0, 0.0);
        let v = validate_inertia(&link);
        assert!(!v.has_errors(), "Unexpected errors: {:?}", v.issues);
    }

    #[test]
    fn not_positive_semi_definite() {
        // Large off-diagonal elements make the matrix non-PSD
        let link = make_link("bad_psd", 1.0, 0.001, 0.001, 0.001, 0.1, 0.1, 0.1);
        let v = validate_inertia(&link);
        assert!(v.has_errors());
        assert!(v.issues.iter().any(|i| i.message.contains("positive semi-definite")));
    }

    #[test]
    fn large_inertia_for_mass_warns() {
        // Mass 1 kg but Ixx = 200 → equivalent radius > 10 m
        let link = make_link("huge", 1.0, 200.0, 200.0, 200.0, 0.0, 0.0, 0.0);
        let v = validate_inertia(&link);
        assert!(v.has_warnings());
        assert!(v.issues.iter().any(|i| i.message.contains("very large")));
    }

    #[test]
    fn validate_all_returns_per_link() {
        let mut model = RobotModel::new_empty("test");
        // new_empty creates "base_link" with mass=1.0 and valid inertia

        // Add a child link
        model.add_child(
            "base_link", "bad_link", "j1",
            "revolute",
            nalgebra::Isometry3::identity(),
            nalgebra::Vector3::z(),
            GeomData::Box { hx: 0.1, hy: 0.1, hz: 0.1 },
            [0.5, 0.5, 0.5, 1.0],
            -1.0, 1.0,
        ).unwrap();
        // Make the child link have bad inertia (negative mass)
        if let Some(idx) = model.link_map.get("bad_link") {
            model.links[*idx].inertial.mass = -1.0;
        }
        let results = validate_all_inertia(&model);
        assert_eq!(results.len(), 2);
        let bad_result = results.iter().find(|r| r.link_name == "bad_link").unwrap();
        assert!(bad_result.has_errors());
        let good_result = results.iter().find(|r| r.link_name == "base_link").unwrap();
        assert!(!good_result.has_errors());
    }

    #[test]
    fn valid_nonzero_offdiag_passes() {
        // A physically valid tensor with small off-diagonal elements
        // that still satisfy PSD
        let link = make_link("tilted", 2.0, 0.05, 0.04, 0.06, 0.001, 0.001, 0.001);
        let v = validate_inertia(&link);
        assert!(!v.has_errors(), "Unexpected errors: {:?}", v.issues);
    }
}

// ============================================================
// serde — JSON serialisation round-trip tests
// ============================================================
#[cfg(feature = "serde")]
mod test_serde {
    use super::*;
    use articara::dynamics;
    use articara::robot::RobotModel;
    use std::collections::HashSet;

    /// RobotModel round-trips through JSON without data loss.
    #[test]
    fn robot_model_json_roundtrip() {
        let model = RobotModel::from_urdf(&namiashi_urdf()).unwrap();
        let json = serde_json::to_string(&model).expect("serialize RobotModel");
        let deser: RobotModel = serde_json::from_str(&json).expect("deserialize RobotModel");

        assert_eq!(deser.name, model.name);
        assert_eq!(deser.links.len(), model.links.len());
        assert_eq!(deser.joints.len(), model.joints.len());
        assert_eq!(deser.root_link, model.root_link);
        assert_eq!(deser.joint_positions.len(), model.joint_positions.len());
        for (a, b) in deser.joint_positions.iter().zip(model.joint_positions.iter()) {
            assert!((a - b).abs() < 1e-6, "joint position mismatch");
        }
    }

    /// SimGraphData round-trips through JSON.
    #[test]
    fn sim_graph_data_json_roundtrip() {
        let gd = dynamics::SimGraphData {
            time: vec![0.0, 0.001, 0.002],
            pos_x: vec![0.1, 0.2, 0.3],
            pos_y: vec![0.0, 0.0, 0.0],
            pos_z: vec![0.15, 0.20, 0.25],
            vel_x: vec![1.0, 1.1, 1.2],
            vel_y: vec![0.0, 0.0, 0.0],
            vel_z: vec![0.5, 0.6, 0.7],
            acc_x: vec![0.0, 0.0, 0.0],
            acc_y: vec![0.0, 0.0, 0.0],
            acc_z: vec![-9.8, -9.8, -9.8],
            link_name: "trunk".into(),
        };
        let json = serde_json::to_string(&gd).unwrap();
        let deser: dynamics::SimGraphData = serde_json::from_str(&json).unwrap();
        assert_eq!(deser.time.len(), gd.time.len());
        assert_eq!(deser.link_name, gd.link_name);
    }

    /// JumpSimResult round-trips through JSON.
    #[test]
    fn jump_result_json_roundtrip() {
        let jr = dynamics::JumpSimResult {
            max_height: 0.135,
            extension_duration: 0.3,
            joint_peaks: vec![dynamics::JointPeakInfo {
                joint_idx: 0,
                joint_name: "hip".into(),
                peak_torque: 5.0,
                peak_torque_angle: 0.3,
                peak_velocity: 2.5,
                peak_velocity_angle: 0.8,
                contributes: true,
            }],
            graph_data: dynamics::SimGraphData::default(),
        };
        let json = serde_json::to_string(&jr).unwrap();
        let deser: dynamics::JumpSimResult = serde_json::from_str(&json).unwrap();
        assert!((deser.max_height - jr.max_height).abs() < 1e-6);
        assert_eq!(deser.joint_peaks.len(), 1);
        assert_eq!(deser.joint_peaks[0].joint_name, "hip");
    }

    /// Full jump simulation with serialised input/output (native, no WASM).
    /// This validates the same code path used by the WASM plugin.
    #[test]
    fn native_jump_sim_serde_roundtrip() {
        let model = RobotModel::from_urdf(&namiashi_urdf()).unwrap();

        // Serialise the model → JSON → deserialise (simulates host→WASM transfer)
        let json = serde_json::to_string(&model).expect("serialize model");
        let mut deser_model: RobotModel = serde_json::from_str(&json).expect("deserialize model");

        let ground_links: Vec<String> = vec![
            "RL_foot".into(), "FL_foot".into(),
            "RR_foot".into(), "FR_foot".into(),
        ];
        let locked = HashSet::new();
        let sim = dynamics::start_jump_sim(
            &mut deser_model,
            &ground_links,
            Some("trunk"),
            1.0,
            &locked,
            [false, false, true],
            None,
            false,
            true,
            Some("trunk"),
            500.0,
            20.0,
        );
        assert!(sim.is_some(), "jump sim should initialise");
        let mut sim = sim.unwrap();

        let dt = 1.0 / 60.0_f32;
        for _ in 0..600 {
            if !dynamics::step_jump_sim(&mut sim, &mut deser_model, dt) {
                break;
            }
        }

        let result = dynamics::extract_jump_result(&sim, &deser_model);
        assert!(result.max_height > 0.0, "should have positive jump height");

        // Result round-trips through JSON
        let result_json = serde_json::to_string(&result).expect("serialize result");
        let deser_result: dynamics::JumpSimResult =
            serde_json::from_str(&result_json).expect("deserialize result");
        assert!((deser_result.max_height - result.max_height).abs() < 1e-6);
        assert_eq!(deser_result.joint_peaks.len(), result.joint_peaks.len());
        assert!(!deser_result.graph_data.time.is_empty(), "graph data should have samples");
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

/// Sidecar `.misarta.toml` round-trip — make sure mutated actuator settings
/// (mode + Kp + Kv) survive a save / load cycle. This guards against the
/// regression where loaded actuator entries appeared in the TOML but didn't
/// reach `JointData` because the joint-name lookup failed.
#[cfg(test)]
mod test_sidecar {
    use super::*;
    use articara::rbd::model::ActuatorMode;
    use articara::robot::RobotModel;

    #[test]
    fn actuator_settings_roundtrip_via_sidecar() {
        let mut model = RobotModel::from_file(&fixture_urdf()).unwrap();
        let target_idx = model
            .joints
            .iter()
            .position(|j| j.joint_type != "fixed")
            .expect("fixture should have at least one movable joint");
        let target_name = model.joints[target_idx].name.clone();

        model.joints[target_idx].actuator_mode = ActuatorMode::Velocity;
        model.joints[target_idx].actuator_kp = 123.0;
        model.joints[target_idx].actuator_kv = 7.5;

        let cfg = model.to_misarta_config();
        let tmp = std::env::temp_dir().join("articara_actuator_roundtrip.misarta.toml");
        cfg.save(&tmp).unwrap();

        let mut model2 = RobotModel::from_file(&fixture_urdf()).unwrap();
        let cfg2 = misarta::config::MisartaConfig::load(&tmp).unwrap();
        model2.load_misarta_config(&cfg2);

        let restored = &model2.joints[target_idx];
        assert_eq!(restored.name, target_name);
        assert_eq!(restored.actuator_mode, ActuatorMode::Velocity);
        assert!((restored.actuator_kp - 123.0).abs() < 1e-9, "Kp not restored: got {}", restored.actuator_kp);
        assert!((restored.actuator_kv - 7.5).abs() < 1e-9, "Kv not restored: got {}", restored.actuator_kv);

        std::fs::remove_file(&tmp).ok();
    }

    #[test]
    fn urdf_export_emits_mimic_for_master_format_entries() {
        let mut model = RobotModel::from_file(&fixture_urdf()).unwrap();
        // Find two non-fixed joints to wire up.
        let movable: Vec<String> = model
            .joints
            .iter()
            .filter(|j| j.joint_type != "fixed")
            .map(|j| j.name.clone())
            .take(2)
            .collect();
        if movable.len() < 2 {
            return;
        }
        model.mimics.push(articara::rbd::model::Mimic {
            joint: movable[1].clone(),
            source: movable[0].clone(),
            multiplier: 0.5,
            offset: 0.1,
        });
        let urdf_xml = model.export_urdf().unwrap();
        assert!(
            urdf_xml.contains("<mimic"),
            "URDF should contain <mimic> tag:\n{}",
            urdf_xml,
        );
        assert!(urdf_xml.contains(&format!("joint=\"{}\"", movable[0])));
    }

    #[test]
    fn mjcf_export_emits_equality_and_sensor() {
        let mut model = RobotModel::from_file(&fixture_urdf()).unwrap();
        let movable: Vec<String> = model
            .joints
            .iter()
            .filter(|j| j.joint_type != "fixed")
            .map(|j| j.name.clone())
            .take(2)
            .collect();
        if movable.len() < 2 {
            return;
        }
        model.mimics.push(articara::rbd::model::Mimic {
            joint: movable[1].clone(),
            source: movable[0].clone(),
            multiplier: 2.0,
            offset: 0.0,
        });
        model.sensors.push(articara::rbd::model::Sensor {
            name: "imu0".into(),
            link: model.links[0].name.clone(),
            origin: nalgebra::Isometry3::identity(),
            update_rate: 100.0,
            kind: articara::rbd::model::SensorKind::Imu {
                gyro_noise: 0.0,
                accel_noise: 0.0,
            },
        });
        let xml = articara::mjcf::export_mjcf(&model);
        assert!(xml.contains("<equality>"), "expected <equality> block:\n{}", xml);
        assert!(
            xml.contains(&format!("joint1=\"{}\"", movable[1])),
            "expected mimic joint1 in MJCF:\n{}",
            xml,
        );
        assert!(xml.contains("polycoef=\"0 2 0 0 0\""));
        assert!(xml.contains("<sensor>"));
        assert!(xml.contains("accelerometer"));
        assert!(xml.contains("gyro"));
    }

    #[test]
    fn sdf_export_emits_mimic_and_sensor() {
        let mut model = RobotModel::from_file(&fixture_urdf()).unwrap();
        let movable: Vec<String> = model
            .joints
            .iter()
            .filter(|j| j.joint_type != "fixed")
            .map(|j| j.name.clone())
            .take(2)
            .collect();
        if movable.len() < 2 {
            return;
        }
        model.mimics.push(articara::rbd::model::Mimic {
            joint: movable[1].clone(),
            source: movable[0].clone(),
            multiplier: -1.0,
            offset: 0.0,
        });
        model.sensors.push(articara::rbd::model::Sensor {
            name: "front_lidar".into(),
            link: model.links[0].name.clone(),
            origin: nalgebra::Isometry3::translation(0.1, 0.0, 0.05),
            update_rate: 10.0,
            kind: articara::rbd::model::SensorKind::Lidar {
                range_min: 0.05,
                range_max: 30.0,
                h_fov: std::f64::consts::TAU,
                h_samples: 360,
                v_fov: 0.0,
                v_samples: 1,
            },
        });
        let xml = articara::sdf::export_sdf(&model);
        assert!(
            xml.contains("<mimic joint=\""),
            "expected SDF <mimic>:\n{}",
            xml,
        );
        assert!(xml.contains(&format!("multiplier=\"{}\"", -1.0_f64)));
        assert!(xml.contains("<sensor name=\"front_lidar\""));
        assert!(xml.contains("<ray>"));
    }

    #[test]
    fn mimic_and_sensor_roundtrip_via_sidecar() {
        // Mimic + Sensor are now first-class master-format entries.
        // Round-trip a model carrying one of each through TOML.
        let mut model = RobotModel::from_file(&fixture_urdf()).unwrap();
        model.mimics.push(articara::rbd::model::Mimic {
            joint: "j2".into(),
            source: "j1".into(),
            multiplier: 0.5,
            offset: 0.1,
        });
        model.sensors.push(articara::rbd::model::Sensor {
            name: "front_cam".into(),
            link: model.links[0].name.clone(),
            origin: nalgebra::Isometry3::translation(0.1, 0.0, 0.05),
            update_rate: 30.0,
            kind: articara::rbd::model::SensorKind::Camera {
                fov: 1.2,
                width: 320,
                height: 240,
                near: 0.05,
                far: 50.0,
            },
        });
        let cfg = model.to_misarta_config();
        let toml = cfg.to_toml().unwrap();
        assert!(toml.contains("[[mimic]]"));
        assert!(toml.contains("[[sensor]]"));
        let cfg2 = misarta::config::MisartaConfig::from_toml(&toml).unwrap();
        let mut model2 = RobotModel::from_file(&fixture_urdf()).unwrap();
        model2.load_misarta_config(&cfg2);
        assert_eq!(model2.mimics.len(), 1);
        assert_eq!(model2.mimics[0].joint, "j2");
        assert!((model2.mimics[0].multiplier - 0.5).abs() < 1e-9);
        assert_eq!(model2.sensors.len(), 1);
        match &model2.sensors[0].kind {
            articara::rbd::model::SensorKind::Camera { width, height, .. } => {
                assert_eq!(*width, 320);
                assert_eq!(*height, 240);
            }
            _ => panic!("expected Camera"),
        }
    }

    #[test]
    fn format_registry_dispatches_correctly() {
        // The registry should pick the right handler for each extension.
        use articara::format::FormatRegistry;
        let reg = FormatRegistry::default_registry();
        let urdf_h = reg.handler_for(std::path::Path::new("/x/foo.urdf"));
        assert!(urdf_h.is_some());
        assert_eq!(urdf_h.unwrap().name(), "URDF");
        let sdf_h = reg.handler_for(std::path::Path::new("/x/foo.sdf"));
        assert_eq!(sdf_h.unwrap().name(), "SDF");
        let usd_h = reg.handler_for(std::path::Path::new("/x/foo.usda"));
        assert_eq!(usd_h.unwrap().name(), "Isaac USD");
        // Capabilities are honest about what each format can express.
        let urdf_caps = urdf_h.unwrap().capabilities();
        assert!(urdf_caps.mimic);
        assert!(!urdf_caps.sensors);
        assert!(!urdf_caps.collision_pairs);
        let sdf_caps = sdf_h.unwrap().capabilities();
        assert!(sdf_caps.sensors);
    }

    #[test]
    fn format_registry_can_import_fixture_urdf() {
        use articara::format::FormatRegistry;
        let reg = FormatRegistry::default_registry();
        let model = reg.import(&fixture_urdf()).unwrap();
        assert!(!model.links.is_empty());
    }

    #[test]
    fn loop_closure_capture_from_pose_satisfies_constraint() {
        // The "📍 Capture from current pose" UI button uses the same
        // midpoint-of-origins formula tested here. We assert that after
        // capturing offsets, the loop-closure error at the current pose is
        // ~0 — i.e. the constraint is exactly satisfied.
        let mut model = RobotModel::from_file(&fixture_urdf()).unwrap();
        if model.links.len() < 2 {
            return;
        }
        let a_name = model.links[0].name.clone();
        let b_name = model.links[1].name.clone();

        // Move some joints so the two links are at non-trivial poses.
        for q in model.joint_positions.iter_mut().take(2) {
            *q = 0.3;
        }
        model.rebuild_misarta_model();

        let transforms = model.compute_transforms();
        let pa = transforms[&a_name].translation.vector;
        let pb = transforms[&b_name].translation.vector;
        let mid = (pa + pb) * 0.5;
        let mid_pt = nalgebra::Point3::from(mid);
        let oa = transforms[&a_name].inverse().transform_point(&mid_pt);
        let ob = transforms[&b_name].inverse().transform_point(&mid_pt);

        model.loop_closures.push(
            articara::rbd::model::LoopClosure::position(
                "captured".to_string(),
                a_name,
                nalgebra::Vector3::new(oa.x as f64, oa.y as f64, oa.z as f64),
                b_name,
                nalgebra::Vector3::new(ob.x as f64, ob.y as f64, ob.z as f64),
            ),
        );
        model.rebuild_misarta_model();

        let err = model.loop_closure_error();
        assert!(
            err < 1e-5,
            "Captured offsets should satisfy the constraint at current pose, got err = {}",
            err,
        );
    }

    #[test]
    fn loop_closure_rotation_roundtrips_via_sidecar() {
        // 6-DoF pose loop closures must round-trip rotation as well.
        let mut model = RobotModel::from_file(&fixture_urdf()).unwrap();
        let q_a = nalgebra::UnitQuaternion::from_axis_angle(
            &nalgebra::Vector3::z_axis(),
            0.5,
        );
        let q_b = nalgebra::UnitQuaternion::from_axis_angle(
            &nalgebra::Vector3::x_axis(),
            -0.3,
        );
        model.loop_closures.push(
            articara::rbd::model::LoopClosure::pose(
                "weld".to_string(),
                model.links[0].name.clone(),
                nalgebra::Isometry3::from_parts(
                    nalgebra::Translation3::new(0.1, 0.0, 0.0),
                    q_a,
                ),
                model.links[1].name.clone(),
                nalgebra::Isometry3::from_parts(
                    nalgebra::Translation3::new(0.0, 0.1, 0.0),
                    q_b,
                ),
            ),
        );
        let cfg = model.to_misarta_config();
        let toml = cfg.to_toml().unwrap();
        let cfg2 = misarta::config::MisartaConfig::from_toml(&toml).unwrap();
        let mut model2 = RobotModel::from_file(&fixture_urdf()).unwrap();
        model2.load_misarta_config(&cfg2);
        assert_eq!(model2.loop_closures.len(), 1);
        let lc = &model2.loop_closures[0];
        // Rotation preserved
        let r_a = lc.offset_a.rotation;
        let r_b = lc.offset_b.rotation;
        let dq_a = (r_a.inverse() * q_a).angle();
        let dq_b = (r_b.inverse() * q_b).angle();
        assert!(dq_a.abs() < 1e-6, "rot_a not preserved: angle diff = {}", dq_a);
        assert!(dq_b.abs() < 1e-6, "rot_b not preserved: angle diff = {}", dq_b);
        assert!(lc.pose_6dof);
    }

    #[test]
    fn sequence_roundtrip_and_animation_build() {
        let mut model = RobotModel::from_file(&fixture_urdf()).unwrap();
        // Need at least one pose for the sequence step to reference.
        let snap = articara::rbd::model::NamedPose::snapshot(
            "rest", &model, 1.0, misarta::trajectory::InterpolationKind::Linear,
        );
        model.poses.push(snap);
        model.sequences.push(articara::rbd::model::Sequence {
            name: "demo".into(),
            steps: vec![
                articara::rbd::model::SequenceStep {
                    pose_name: "rest".into(),
                    duration: 0.5,
                    kind: misarta::trajectory::InterpolationKind::QuinticSmooth,
                },
                articara::rbd::model::SequenceStep {
                    pose_name: "rest".into(),
                    duration: 0.3,
                    kind: misarta::trajectory::InterpolationKind::Linear,
                },
            ],
        });

        // Roundtrip
        let cfg = model.to_misarta_config();
        let toml = cfg.to_toml().unwrap();
        assert!(toml.contains("[[sequence]]"));
        let cfg2 = misarta::config::MisartaConfig::from_toml(&toml).unwrap();
        let mut model2 = RobotModel::from_file(&fixture_urdf()).unwrap();
        // Pose must exist so build_sequence_animation can resolve the step.
        let snap2 = articara::rbd::model::NamedPose::snapshot(
            "rest", &model2, 1.0, misarta::trajectory::InterpolationKind::Linear,
        );
        model2.poses.push(snap2);
        model2.load_misarta_config(&cfg2);
        assert_eq!(model2.sequences.len(), 1);
        assert_eq!(model2.sequences[0].steps.len(), 2);
        assert!((model2.sequences[0].steps[0].duration - 0.5).abs() < 1e-9);

        // Animation construction
        let anim = model2.build_sequence_animation("demo").unwrap();
        // 1 anchor + 2 steps = 3 keyframes
        assert_eq!(anim.len(), 3);
        // Total duration should be 0.5 + 0.3 = 0.8
        assert!((anim.duration() - 0.8).abs() < 1e-9);
    }

    #[test]
    fn collision_pairs_roundtrip_via_sidecar() {
        let mut model = RobotModel::from_file(&fixture_urdf()).unwrap();
        // Pick two link names from the fixture so the test isn't fragile to
        // joint count.
        assert!(model.links.len() >= 2);
        let a = model.links[0].name.clone();
        let b = model.links[1].name.clone();
        model.collision_pairs.push(
            articara::rbd::model::CollisionPair::new(a.clone(), b.clone(), false),
        );

        let cfg = model.to_misarta_config();
        let toml = cfg.to_toml().unwrap();
        assert!(toml.contains("[[collision_pair]]"),
            "TOML should contain collision_pair entries:\n{}", toml);

        let mut model2 = RobotModel::from_file(&fixture_urdf()).unwrap();
        let cfg2 = misarta::config::MisartaConfig::from_toml(&toml).unwrap();
        model2.load_misarta_config(&cfg2);
        assert_eq!(model2.collision_pairs.len(), 1);
        let cp = &model2.collision_pairs[0];
        assert!(cp.matches(&a, &b));
        assert!(!cp.enabled);
    }

    #[test]
    fn mjcf_export_separates_visual_and_collision_geoms() {
        // Regression for the bug where MJCF only emitted visuals — meaning
        // MuJoCo did collision detection on the high-detail visual meshes
        // and produced spurious self-collision penalties at joint
        // boundaries. The fix is to emit both with the standard
        // contype/conaffinity/group convention.
        let mut model = RobotModel::from_file(&fixture_urdf()).unwrap();
        // Synthesize a collision shape on link0 if it has none, so the
        // generic test_robot fixture exercises both paths.
        if let Some(li) = model.links.iter().position(|l| l.collisions.is_empty()) {
            model.links[li].collisions.push(articara::robot::CollisionData {
                origin: nalgebra::Isometry3::identity(),
                geometry: articara::robot::GeomData::Box {
                    hx: 0.05, hy: 0.05, hz: 0.05,
                },
            });
        }
        let xml = articara::mjcf::export_mjcf(&model);
        // Visual geoms should be no-collide (group 1).
        assert!(xml.contains("contype=\"0\""),
            "MJCF should mark visuals as contype=\"0\":\n{}", xml);
        assert!(xml.contains("conaffinity=\"0\""),
            "MJCF should mark visuals as conaffinity=\"0\":\n{}", xml);
        assert!(xml.contains("group=\"1\""),
            "MJCF should put visuals in group=\"1\":\n{}", xml);
        // Collision geoms should be physics-enabled (group 3).
        assert!(xml.contains("contype=\"1\""),
            "MJCF should mark collisions as contype=\"1\":\n{}", xml);
        assert!(xml.contains("conaffinity=\"1\""),
            "MJCF should mark collisions as conaffinity=\"1\":\n{}", xml);
        assert!(xml.contains("group=\"3\""),
            "MJCF should put collisions in group=\"3\":\n{}", xml);
    }

    #[test]
    fn mjcf_export_emits_contact_exclude_for_disabled_pairs() {
        let mut model = RobotModel::from_file(&fixture_urdf()).unwrap();
        let a = model.links[0].name.clone();
        let b = model.links[1].name.clone();
        model.collision_pairs.push(
            articara::rbd::model::CollisionPair::new(a.clone(), b.clone(), false),
        );
        let xml = articara::mjcf::export_mjcf(&model);
        assert!(xml.contains("<contact>"),
            "expected <contact> block:\n{}", xml);
        assert!(xml.contains("<exclude"),
            "expected <exclude/> entry:\n{}", xml);
        assert!(xml.contains(&a) && xml.contains(&b));
    }

    #[test]
    fn mjcf_export_omits_contact_when_no_disabled_pairs() {
        let model = RobotModel::from_file(&fixture_urdf()).unwrap();
        let xml = articara::mjcf::export_mjcf(&model);
        assert!(!xml.contains("<exclude"),
            "no excludes should appear when collision_pairs is empty");
    }

    #[test]
    fn actuator_load_via_load_sidecar_path() {
        let urdf_src = fixture_urdf();
        let tmp_dir = std::env::temp_dir().join("articara_sidecar_path_test");
        std::fs::create_dir_all(&tmp_dir).unwrap();
        let urdf_dst = tmp_dir.join("test_robot.urdf");
        std::fs::copy(&urdf_src, &urdf_dst).unwrap();

        let mut model = RobotModel::from_file(&urdf_dst).unwrap();
        let target_idx = model
            .joints
            .iter()
            .position(|j| j.joint_type != "fixed")
            .unwrap();
        model.joints[target_idx].actuator_mode = ActuatorMode::Torque;
        model.joints[target_idx].actuator_kp = 999.0;
        model.joints[target_idx].actuator_kv = 42.0;

        model.save_sidecar_config(&urdf_dst).unwrap();

        let mut model2 = RobotModel::from_file(&urdf_dst).unwrap();
        let report = model2.load_sidecar_config();
        let report = report.expect("load_sidecar_config should return Some after save");
        assert!(report.n_actuators_total > 0, "saved sidecar should contain actuator entries");
        assert_eq!(report.n_actuators_applied, report.n_actuators_total,
            "all actuator entries should match the model: unmatched={:?}",
            report.unmatched_actuators);
        let restored = &model2.joints[target_idx];
        assert_eq!(restored.actuator_mode, ActuatorMode::Torque,
            "actuator mode not restored via load_sidecar_config");
        assert!((restored.actuator_kp - 999.0).abs() < 1e-9);
        assert!((restored.actuator_kv - 42.0).abs() < 1e-9);

        std::fs::remove_dir_all(&tmp_dir).ok();
    }
}

