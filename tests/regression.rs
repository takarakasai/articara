//! Regression tests for roboview.
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
        .parent()
        .unwrap()
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
    use roboview::robot::*;

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
        model.joint_positions[ji] = std::f32::consts::FRAC_PI_2;

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
    use roboview::format::RobotFormat;
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
        assert!(!RobotFormat::IsaacUsd.supports_import());
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
    use roboview::robot::*;
    use roboview::sdf;

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
        let tmp = std::env::temp_dir().join("roboview_test_sdf_roundtrip.sdf");
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
}

// ============================================================
// mjcf.rs — MJCF import and export
// ============================================================
mod test_mjcf {
    use super::*;
    use roboview::mjcf;
    use roboview::robot::*;

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

        let tmp = std::env::temp_dir().join("roboview_test_mjcf_roundtrip.xml");
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
    use roboview::isaac;
    use roboview::robot::RobotModel;

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
        let tmp = std::env::temp_dir().join("roboview_isaac_test");
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
// camera.rs — camera math
// ============================================================
mod test_camera {
    use nalgebra as na;
    use roboview::camera::OrbitCamera;

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
// ik.rs — inverse kinematics
// ============================================================
mod test_ik {
    use super::*;
    use nalgebra as na;
    use roboview::ik;
    use roboview::robot::RobotModel;

    #[test]
    fn build_chain_two_joints() {
        let model = RobotModel::from_urdf(&fixture_urdf()).unwrap();
        let chain = ik::build_chain(&model, "link2");
        assert_eq!(chain.len(), 2);
        assert_eq!(model.joints[chain[0].joint_idx].name, "joint1");
        assert_eq!(model.joints[chain[1].joint_idx].name, "joint2");
    }

    #[test]
    fn build_chain_one_joint() {
        let model = RobotModel::from_urdf(&fixture_urdf()).unwrap();
        let chain = ik::build_chain(&model, "link1");
        assert_eq!(chain.len(), 1);
        assert_eq!(model.joints[chain[0].joint_idx].name, "joint1");
    }

    #[test]
    fn build_chain_root_is_empty() {
        let model = RobotModel::from_urdf(&fixture_urdf()).unwrap();
        let chain = ik::build_chain(&model, "base_link");
        assert_eq!(chain.len(), 0);
    }

    #[test]
    fn build_chain_fixed_joint_skipped() {
        let model = RobotModel::from_urdf(&fixture_urdf()).unwrap();
        let chain = ik::build_chain(&model, "fixed_part");
        assert_eq!(chain.len(), 0); // fixed joints are not in chain
    }

    #[test]
    fn jacobian_dimensions() {
        let model = RobotModel::from_urdf(&fixture_urdf()).unwrap();
        let chain = ik::build_chain(&model, "link2");
        let tf = model.compute_transforms();
        let ee_pos = na::Point3::new(0.0, 0.0, 0.25_f32);
        let jac = ik::compute_jacobian(&model, &chain, &tf, &ee_pos);
        assert_eq!(jac.nrows(), 3);
        assert_eq!(jac.ncols(), 2);
    }

    #[test]
    fn ik_step_reduces_error() {
        let mut model = RobotModel::from_urdf(&fixture_urdf()).unwrap();
        let chain = ik::build_chain(&model, "link2");
        let tf = model.compute_transforms();
        let li = model.link_map["link2"];
        let ee_pos = ik::get_ee_world_pos(&model, li, &tf);
        let target = na::Point3::new(0.1, 0.0, 0.2_f32);
        let initial_error = na::distance(&ee_pos, &target);

        let deltas = ik::solve_ik_step(&model, &chain, &tf, &ee_pos, &target, 0.05, 0.1);
        assert_eq!(deltas.len(), 2);

        ik::apply_ik_deltas(&mut model, &chain, &deltas);
        let tf2 = model.compute_transforms();
        let new_pos = ik::get_ee_world_pos(&model, li, &tf2);
        let new_error = na::distance(&new_pos, &target);

        assert!(new_error < initial_error,
            "IK should reduce error: {initial_error} -> {new_error}");
    }

    #[test]
    fn apply_ik_deltas_respects_limits() {
        let mut model = RobotModel::from_urdf(&fixture_urdf()).unwrap();
        let chain = ik::build_chain(&model, "link2");

        // Try to apply huge deltas
        let deltas = vec![100.0, 100.0];
        ik::apply_ik_deltas(&mut model, &chain, &deltas);

        let ji1 = chain[0].joint_idx;
        let ji2 = chain[1].joint_idx;
        assert!(model.joint_positions[ji1] <= model.joints[ji1].upper as f32 + 1e-6);
        assert!(model.joint_positions[ji2] <= model.joints[ji2].upper as f32 + 1e-6);
    }
}

// ============================================================
// primitives.rs — geometry generation
// ============================================================
mod test_primitives {
    use roboview::primitives;

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
    use roboview::robot::RobotModel;

    /// Load URDF, export to SDF, re-import → verify data preservation.
    #[test]
    fn urdf_to_sdf_roundtrip() {
        let model = RobotModel::from_urdf(&fixture_urdf()).unwrap();
        let sdf_xml = roboview::sdf::export_sdf(&model);

        let tmp = std::env::temp_dir().join("roboview_urdf2sdf.sdf");
        std::fs::write(&tmp, &sdf_xml).unwrap();
        let model2 = roboview::sdf::import_sdf(&tmp).expect("SDF re-import failed");
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
        let mjcf_xml = roboview::mjcf::export_mjcf(&model);

        let tmp = std::env::temp_dir().join("roboview_urdf2mjcf.xml");
        std::fs::write(&tmp, &mjcf_xml).unwrap();
        let model2 = roboview::mjcf::import_mjcf(&tmp).expect("MJCF re-import failed");
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
    use roboview::robot::*;

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
        use roboview::sdf::export_sdf;
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
        use roboview::mjcf::export_mjcf;
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
    use roboview::robot::{self, GeomData, RobotModel};

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
}
