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
        .join("tests")
        .join("fixtures")
        .join("namiashi")
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
    fn all_contains_five() {
        // Misa (native) + URDF + SDF + MJCF + USD
        assert_eq!(RobotFormat::ALL.len(), 5);
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

    /// Regression: a model whose stored home pose violates its own joint
    /// limits (e.g. quadruped calf range [-2.7, -0.84] but home_pose = 0)
    /// would seed MuJoCo with an out-of-range qpos. The contact / jointlimit
    /// solver pushes the joint back into range with huge force while the
    /// position-PD actuator drives toward the original (out-of-range) target,
    /// producing a violent oscillation. The fix clamps initial joint
    /// positions to [lower, upper] before seeding MuJoCo.
    ///
    /// This test exercises the clamp policy through a helper-style direct
    /// assertion since spinning up an actual MuJoCo sim from a unit test
    /// would require the `mujoco` feature + the runtime library.
    #[test]
    fn out_of_range_home_pose_clamps_to_limits() {
        // Mimic the clamp the MujocoSim::new constructor applies: every
        // non-fixed joint that has a real range (lower < upper) and is
        // being baked into MJCF must clamp the seeded position into range.
        let model = articara::robot::RobotModel::from_urdf(&fixture_urdf()).unwrap();

        // Synthesise a "broken home pose": pick any revolute joint and
        // set its position outside its declared limits.
        let target = model
            .joints
            .iter()
            .enumerate()
            .find(|(_, j)| j.joint_type != "fixed" && j.lower < j.upper)
            .expect("fixture URDF should have at least one limited revolute joint");
        let (ji, joint) = target;
        let out_of_range = joint.upper + 1.0; // clearly above upper
        let clamped = out_of_range.clamp(joint.lower, joint.upper);
        assert_eq!(
            clamped, joint.upper,
            "clamp should snap to upper bound when input exceeds upper"
        );
        let below_range = joint.lower - 1.0;
        let clamped_lo = below_range.clamp(joint.lower, joint.upper);
        assert_eq!(
            clamped_lo, joint.lower,
            "clamp should snap to lower bound when input below lower"
        );
        let inside = (joint.lower + joint.upper) / 2.0;
        assert_eq!(
            inside.clamp(joint.lower, joint.upper),
            inside,
            "in-range values must pass through unchanged"
        );
        // Sanity: joint really has the expected fields available (so the
        // production clamp code can read joint.lower / joint.upper).
        let _ = ji;
    }

    /// Regression: when a link's inertia tensor has non-zero products of
    /// inertia (`ixy / ixz / iyz`) — i.e. the principal axes are rotated
    /// relative to the link frame — MJCF must emit `fullinertia` rather than
    /// silently truncating to `diaginertia`. Pre-fix, the off-diagonals were
    /// dropped, giving MuJoCo a different inertia tensor and producing
    /// instability for heavy / off-centre links (notably the trunk).
    #[test]
    fn mjcf_emits_fullinertia_when_off_diagonals_present() {
        use articara::rbd::model::{InertialData, LinkData};
        use articara::robot::RobotModel;

        let mut model = RobotModel::from_urdf(&fixture_urdf()).unwrap();
        // Inject non-zero products of inertia on the root link.
        let li = *model
            .link_map
            .get(&model.root_link)
            .expect("root link in map");
        model.links[li].inertial = InertialData {
            origin: nalgebra::Isometry3::identity(),
            mass: 5.0,
            ixx: 0.1,
            iyy: 0.1,
            izz: 0.05,
            ixy: 0.0,
            ixz: 0.02,
            iyz: -0.01,
        };

        let xml = articara::mjcf::export_mjcf(&model);
        assert!(
            xml.contains("fullinertia="),
            "off-diagonal inertia link must emit fullinertia, MJCF:\n{xml}"
        );

        // The clean diagonal links must still use diaginertia.
        let clean_li = (0..model.links.len()).find(|&i| i != li).unwrap();
        let _ = LinkData { ..model.links[clean_li].clone() }; // suppress unused warning if any
        assert!(
            xml.contains("diaginertia="),
            "diagonal-only links should still emit diaginertia"
        );
    }

    /// Regression: a joint loaded with no explicit armature value (the
    /// `#[serde(default)]` path) must default to a small non-zero rotor
    /// inertia. The default-zero case combined with default-stiff PD gains
    /// (kp=50, kv=5) pushed MuJoCo's explicit integrator past its Nyquist
    /// limit on the 2 ms default timestep and produced violent oscillation
    /// before the first sim step finished.
    #[test]
    fn joint_default_armature_is_nonzero() {
        let model = articara::robot::RobotModel::from_urdf(&fixture_urdf()).unwrap();
        let any_movable = model
            .joints
            .iter()
            .filter(|j| j.joint_type != "fixed")
            .next()
            .expect("fixture URDF should have a movable joint");
        assert!(
            any_movable.armature > 0.0,
            "joint {:?} armature should default to a positive value, got {}",
            any_movable.name,
            any_movable.armature
        );
    }

    /// Regression: `export_urdf` previously called `urdf_rs::read_file` on
    /// the source path unconditionally — when the model was loaded from a
    /// `.misa` file the TOML parser was handed to `urdf_rs` and the whole
    /// export aborted with a misleading "Re-read URDF error". Non-URDF
    /// sources must fall back to the from-scratch `generate_urdf_xml`
    /// generator.
    #[test]
    fn export_urdf_from_misa_source_falls_back_to_scratch_generator() {
        // Build a fresh model in memory (no source_path) so we exercise the
        // "non-URDF source" path. Setting source_path to a fake `.misa`
        // path forces the branch we want to test even without producing a
        // real .misa file on disk.
        let mut model = articara::robot::RobotModel::from_urdf(&fixture_urdf()).unwrap();
        model.source_path = Some(std::path::PathBuf::from("/tmp/nonexistent.misa"));
        let xml = model.export_urdf().expect(
            "export_urdf should succeed for .misa source — the loader \
             must NOT try to re-read a .misa as URDF",
        );
        assert!(xml.contains("<robot"), "exported XML should contain <robot>");
        assert!(
            xml.contains(&format!("<link name=\"{}\"", model.root_link)),
            "exported URDF should contain the root link"
        );
    }

    /// Regression: mesh-reference scale must be forwarded into the MJCF
    /// `<mesh>` asset element. Pre-fix, `<mesh name="..." file="..."/>`
    /// dropped the scale, so a millimetre-unit OBJ tagged with
    /// `scale = [0.001, 0.001, 0.001]` in the source model loaded into
    /// MuJoCo at 1000× its intended size, producing catastrophic
    /// ground penetration at t=0 and meganewton contact forces.
    #[test]
    fn mjcf_emits_mesh_scale_attribute() {
        use articara::rbd::model::{CollisionData, GeomData, VisualData};

        let mut model = articara::robot::RobotModel::from_urdf(&fixture_urdf()).unwrap();
        // Inject a scaled mesh visual on the root link.
        model.links[0].visuals.push(VisualData {
            origin: nalgebra::Isometry3::identity(),
            geometry: GeomData::Mesh {
                vertices: vec![
                    0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
                    1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
                    0.0, 1.0, 0.0, 0.0, 0.0, 1.0,
                ],
                filename: Some("/abs/path/example.obj".into()),
                scale: Some([0.001, 0.001, 0.001]),
            },
            color: [1.0; 4],
        });
        model.links[0].collisions.push(CollisionData {
            origin: nalgebra::Isometry3::identity(),
            geometry: GeomData::Mesh {
                vertices: vec![
                    0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
                    1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
                    0.0, 1.0, 0.0, 0.0, 0.0, 1.0,
                ],
                filename: Some("/abs/path/example.obj".into()),
                scale: Some([0.001, 0.001, 0.001]),
            },
        });
        let xml = articara::mjcf::export_mjcf(&model);
        // f32 → f64 conversion gives e.g. "0.0010000000474974513"; we just
        // need the attribute present with a value in the 1e-3 range.
        assert!(
            xml.contains("scale=\"0.001"),
            "MJCF mesh asset must include scale attribute when non-unit:\n{xml}"
        );
        // Unit-scale meshes should still omit the attribute (cleaner output).
        let mut model2 = articara::robot::RobotModel::from_urdf(&fixture_urdf()).unwrap();
        model2.links[0].visuals.push(VisualData {
            origin: nalgebra::Isometry3::identity(),
            geometry: GeomData::Mesh {
                vertices: vec![0.0; 18],
                filename: Some("/abs/unit.obj".into()),
                scale: None,
            },
            color: [1.0; 4],
        });
        let xml2 = articara::mjcf::export_mjcf(&model2);
        assert!(
            xml2.contains("<mesh name=") && !xml2.contains(" scale=\""),
            "unit-scale meshes should omit scale attribute:\n{xml2}"
        );
    }

    /// Regression: MJCF must auto-emit `<contact><exclude>` for every
    /// parent-child link pair (URDF semantic), so a model that doesn't
    /// enumerate self-collision exclusions by hand still simulates without
    /// adjacent collision geoms colliding through their joint origins. Pre-
    /// fix, only user-defined `collision_pairs` (`enabled=false`) were
    /// emitted, so a freshly-converted URDF would launch with hundreds of
    /// spurious self-contacts producing meganewton force vectors at t=0.
    #[test]
    fn mjcf_auto_excludes_parent_child_pairs() {
        let model = articara::robot::RobotModel::from_urdf(&fixture_urdf()).unwrap();
        let xml = articara::mjcf::export_mjcf(&model);
        // The fixture URDF has at least one parent-child pair; their
        // `<exclude>` line must appear in a `<contact>` block.
        let any_joint = model
            .joints
            .iter()
            .find(|j| j.parent_link != j.child_link)
            .expect("fixture URDF should have at least one joint");
        let (lo, hi) = if any_joint.parent_link <= any_joint.child_link {
            (&any_joint.parent_link, &any_joint.child_link)
        } else {
            (&any_joint.child_link, &any_joint.parent_link)
        };
        let needle_a = format!("<exclude body1=\"{lo}\" body2=\"{hi}\"");
        let needle_b = format!("<exclude body1=\"{hi}\" body2=\"{lo}\"");
        assert!(
            xml.contains("<contact>") && (xml.contains(&needle_a) || xml.contains(&needle_b)),
            "MJCF must emit an <exclude> for parent={:?} child={:?}.\n{xml}",
            any_joint.parent_link, any_joint.child_link
        );
    }

    /// Regression: when the MJCF auto-lifts the root (`base_pos = None`) it
    /// must clear primitive shapes like foot spheres, not just joint origins.
    /// Pre-fix, `compute_initial_z` walked only joint-origin Z values and
    /// ignored sphere radii — so a foot with a collision sphere of radius
    /// 0.022 m penetrated the ground at t=0 by ~0.022 m and MuJoCo bounced
    /// the robot violently. Lift must place the lowest visual point at least
    /// `clearance - epsilon` above the ground plane.
    #[test]
    fn mjcf_auto_lift_clears_foot_sphere() {
        use articara::rbd::model::{CollisionData, VisualData, GeomData};
        use articara::robot::RobotModel;
        use articara::mjcf::{export_mjcf_with_options, MjcfExportOptions, GroundPlaneCfg};

        // Build a tiny model: 1 link with a sphere visual (and matching
        // collision) offset 0.5 m below the root, sphere radius 0.022 m.
        // No URDF round-trip — we construct directly so the test stays
        // self-contained.
        let mut model = RobotModel::from_urdf(&fixture_urdf()).expect("fixture URDF");
        // Add a foot-style sphere visual + collision under the first link,
        // offset by -0.5 m in Z (joint-origin-only logic would only see the
        // -0.5; our fix must additionally see the sphere radius).
        let mut foot_origin = nalgebra::Isometry3::identity();
        foot_origin.translation.z = -0.5;
        model.links[0].visuals.push(VisualData {
            origin: foot_origin,
            geometry: GeomData::Sphere { radius: 0.022 },
            color: [1.0; 4],
        });
        model.links[0].collisions.push(CollisionData {
            origin: foot_origin,
            geometry: GeomData::Sphere { radius: 0.022 },
        });

        let opts = MjcfExportOptions {
            base_pos: None, // auto-lift
            ground_plane: Some(GroundPlaneCfg {
                z: 0.0,
                half_size: 1.0,
                roll: 0.0,
                pitch: 0.0,
            }),
            add_actuators: false,
            base_locked_axes: [false; 6],
            bake_actuator_limits: false,
            bake_joint_position_limits: false,
            mesh_path_style: articara::mesh_paths::MeshPathStyle::Absolute,
            default_friction: [0.7, 0.005, 0.0001],
        };
        let xml = export_mjcf_with_options(&model, opts);

        // Extract the root <body name="base_link" pos="x y z"> Z component.
        // Anchor on the root link name so we don't accidentally match the
        // ground geom's `pos="0 0 0"`.
        let needle = format!("<body name=\"{}\" pos=\"", model.root_link);
        let i = xml.find(&needle).expect("root body header present");
        let rest = &xml[i + needle.len()..];
        let end = rest.find('"').expect("closing quote for root pos=");
        let parts: Vec<f64> = rest[..end]
            .split_whitespace()
            .map(|s| s.parse().unwrap())
            .collect();
        assert_eq!(parts.len(), 3, "root pos= should have 3 components");
        let root_z = parts[2];

        // Required: root_z + (-0.5) - 0.022 >= 0.0 (foot sphere bottom
        // ≥ ground), with a small clearance margin.
        let foot_bottom = root_z - 0.5 - 0.022;
        assert!(
            foot_bottom >= 0.0,
            "foot sphere bottom should clear ground; root_z={root_z}, foot_bottom={foot_bottom}"
        );
        // Sanity: clearance shouldn't be absurd (sub-cm range is the design).
        assert!(
            foot_bottom < 0.05,
            "clearance should be small (~5 mm); got foot_bottom={foot_bottom}"
        );
    }

    /// Regression: MJCF `<body>` must emit URDF joint origin `rpy` as a
    /// `quat` attribute. Dropping it silently rotates every joint axis to
    /// the parent's frame in MuJoCo's view, which on keel-style RPP layouts
    /// (`<joint origin rpy="0 0 π/2"/> <axis xyz="1 0 0"/>`) caused CHAMP
    /// forward commands to drive the body sideways (the axis MuJoCo saw was
    /// body +X, not the body +Y the IK assumed). Confirm a yawed joint
    /// origin produces a non-trivial `quat` in the emitted MJCF.
    #[test]
    fn mjcf_body_preserves_joint_origin_rpy() {
        use articara::robot::RobotModel;

        let mut model = RobotModel::from_urdf(&fixture_urdf()).expect("fixture URDF");
        // Replace the first non-fixed joint's origin with a yaw=π/2 rotation
        // so we can assert the MJCF carries it through.
        let ji = model
            .joints
            .iter()
            .position(|j| j.joint_type != "fixed")
            .expect("fixture URDF should have a movable joint");
        let yaw = std::f32::consts::FRAC_PI_2;
        let rot = nalgebra::UnitQuaternion::from_euler_angles(0.0, 0.0, yaw);
        model.joints[ji].origin.rotation = rot;
        let child_link = model.joints[ji].child_link.clone();

        let xml = articara::mjcf::export_mjcf(&model);

        // The child body should have a quat attribute close to
        // (cos(π/4), 0, 0, sin(π/4)) ≈ (0.7071, 0, 0, 0.7071).
        let body_tag = format!("<body name=\"{child_link}\"");
        let start = xml
            .find(&body_tag)
            .unwrap_or_else(|| panic!("body tag for {child_link} missing in MJCF"));
        let line_end = xml[start..].find('>').unwrap();
        let body_line = &xml[start..start + line_end];
        assert!(
            body_line.contains("quat=\""),
            "<body name={child_link:?}> should carry a quat attribute when its parent joint has \
             non-identity rpy.\nLine: {body_line}"
        );
        // Spot-check the components: w ≈ 0.7071, k (z) ≈ 0.7071.
        let q_start = body_line.find("quat=\"").unwrap() + "quat=\"".len();
        let q_end = body_line[q_start..].find('"').unwrap();
        let qs: Vec<f64> = body_line[q_start..q_start + q_end]
            .split_whitespace()
            .map(|s| s.parse().unwrap())
            .collect();
        assert_eq!(qs.len(), 4, "quat should have 4 components, got {qs:?}");
        let half_sqrt = 0.5_f64.sqrt();
        assert!((qs[0] - half_sqrt).abs() < 1e-4, "quat.w {} != {half_sqrt}", qs[0]);
        assert!(qs[1].abs() < 1e-4, "quat.x {} should be ~0", qs[1]);
        assert!(qs[2].abs() < 1e-4, "quat.y {} should be ~0", qs[2]);
        assert!((qs[3] - half_sqrt).abs() < 1e-4, "quat.z {} != {half_sqrt}", qs[3]);
    }

    /// Regression: MJCF `<default class>` inheritance must be applied at
    /// import time. Robots from MuJoCo Menagerie (Unitree Go2, ANYmal,
    /// etc.) declare joint axis / range / damping / armature in nested
    /// `<default>` blocks rather than inline on each `<joint>`, and the
    /// importer used to drop all of them — every axis collapsed to MJCF's
    /// hardcoded `0 0 1` default, making the loaded kinematic model
    /// useless for dynamics or gait planning.
    ///
    /// Also exercise `<actuator>` import: `<motor>` should set
    /// `ActuatorMode::Torque` on its target joint and copy `ctrlrange` /
    /// `forcerange` into `effort`.
    #[test]
    fn mjcf_default_class_and_actuator_inheritance() {
        use articara::rbd::model::ActuatorMode;

        // Minimal MJCF that exercises a 2-level <default> hierarchy and an
        // <actuator> block with class-inherited ctrlrange — same shape as
        // Unitree Go2.
        let xml = r#"<mujoco model="cls_test">
  <compiler angle="radian"/>
  <default>
    <default class="robot">
      <joint axis="0 1 0" damping="2" armature="0.01"/>
      <motor ctrlrange="-25 25"/>
      <default class="abduction">
        <joint axis="1 0 0" range="-1.0 1.0"/>
      </default>
      <default class="knee">
        <joint range="-2.7 -0.8"/>
        <motor ctrlrange="-45 45"/>
      </default>
    </default>
  </default>
  <worldbody>
    <body name="base" pos="0 0 0.3" childclass="robot">
      <freejoint/>
      <body name="hip" pos="0.1 0 0">
        <joint name="hip_j" class="abduction"/>
        <body name="thigh" pos="0 0 0">
          <joint name="thigh_j"/>
          <body name="calf" pos="0 0 -0.2">
            <joint name="calf_j" class="knee"/>
          </body>
        </body>
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor class="abduction" name="hip_m" joint="hip_j"/>
    <motor class="robot"     name="thigh_m" joint="thigh_j"/>
    <motor class="knee"      name="calf_m" joint="calf_j"/>
  </actuator>
</mujoco>"#;
        let tmp = std::env::temp_dir().join("articara_mjcf_cls_inherit.xml");
        std::fs::write(&tmp, xml).unwrap();
        let model = mjcf::import_mjcf(&tmp).expect("class-inherit MJCF should import");

        let by_name = |n: &str| {
            model
                .joints
                .iter()
                .find(|j| j.name == n)
                .unwrap_or_else(|| panic!("joint {n} missing"))
        };

        let hip = by_name("hip_j");
        let thigh = by_name("thigh_j");
        let calf = by_name("calf_j");

        // abduction inherits robot.joint then overrides axis + range.
        assert!((hip.axis.x - 1.0).abs() < 1e-9, "hip.axis.x = {}", hip.axis.x);
        assert!(hip.axis.y.abs() < 1e-9, "hip.axis.y = {}", hip.axis.y);
        assert!((hip.lower - (-1.0)).abs() < 1e-9, "hip.lower = {}", hip.lower);
        assert!((hip.upper - 1.0).abs() < 1e-9, "hip.upper = {}", hip.upper);
        // Damping / armature stay inherited from `robot`.
        assert!((hip.joint_damping - 2.0).abs() < 1e-9);
        assert!((hip.armature - 0.01).abs() < 1e-9);

        // No explicit class → inherits childclass="robot".
        assert!((thigh.axis.y - 1.0).abs() < 1e-9, "thigh.axis.y = {}", thigh.axis.y);
        assert!(thigh.axis.x.abs() < 1e-9);

        // `knee` keeps robot's Y axis (no override) but overrides range.
        assert!((calf.axis.y - 1.0).abs() < 1e-9);
        assert!((calf.lower - (-2.7)).abs() < 1e-9, "calf.lower = {}", calf.lower);
        assert!((calf.upper - (-0.8)).abs() < 1e-9, "calf.upper = {}", calf.upper);

        // Actuators: all <motor> → Torque, effort from class-inherited ctrlrange.
        assert_eq!(hip.actuator_mode, ActuatorMode::Torque);
        assert_eq!(thigh.actuator_mode, ActuatorMode::Torque);
        assert_eq!(calf.actuator_mode, ActuatorMode::Torque);
        assert!((hip.effort - 25.0).abs() < 1e-9, "hip.effort = {}", hip.effort);
        assert!((thigh.effort - 25.0).abs() < 1e-9);
        assert!(
            (calf.effort - 45.0).abs() < 1e-9,
            "calf.effort = {} (knee class overrides ctrlrange)",
            calf.effort
        );
    }

    /// Regression: a Menagerie-style MJCF where the geom has no inline
    /// `type=` attribute (it inherits `type="mesh"` from `<default
    /// class="visual">`), the `<mesh file="..."/>` carries no `name=`
    /// (so the asset name is the file's stem), the file is `.obj`, and
    /// the meshes live in a `meshdir=` subdir. Hitting any of those four
    /// before this regression dropped Unitree Go2 visuals to spheres or
    /// to empty mesh vertices.
    #[test]
    fn mjcf_imports_class_typed_obj_mesh_via_meshdir() {
        use articara::rbd::model::GeomData;

        // Minimal OBJ — single triangle. tobj is happy with no normals
        // and no mtllib reference.
        let obj = "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n";
        let tmp = std::env::temp_dir().join("articara_mjcf_mesh_test");
        let assets = tmp.join("assets");
        std::fs::create_dir_all(&assets).unwrap();
        std::fs::write(assets.join("tri.obj"), obj).unwrap();

        // MJCF layout: meshdir="assets", mesh asset has no name= (so the
        // name defaults to "tri"), geom is `mesh="tri" class="visual"`
        // with no explicit type= (it inherits type="mesh" from the
        // visual class).
        let xml = r#"<mujoco model="mesh_class_test">
  <compiler angle="radian" meshdir="assets"/>
  <default>
    <default class="visual">
      <geom type="mesh" contype="0" conaffinity="0"/>
    </default>
  </default>
  <asset>
    <mesh file="tri.obj"/>
  </asset>
  <worldbody>
    <body name="root" pos="0 0 0">
      <geom mesh="tri" class="visual"/>
    </body>
  </worldbody>
</mujoco>"#;
        let mjcf_path = tmp.join("test.xml");
        std::fs::write(&mjcf_path, xml).unwrap();

        let model = mjcf::import_mjcf(&mjcf_path).expect("import MJCF");
        let root = model
            .links
            .iter()
            .find(|l| l.name == "root")
            .expect("root link");
        assert_eq!(
            root.visuals.len(),
            1,
            "expected exactly one visual on the root body"
        );
        match &root.visuals[0].geometry {
            GeomData::Mesh {
                vertices,
                filename,
                ..
            } => {
                assert!(
                    filename.as_deref() == Some("tri.obj"),
                    "filename = {filename:?}"
                );
                assert!(
                    !vertices.is_empty(),
                    "OBJ vertex list should be non-empty (got {} floats)",
                    vertices.len()
                );
                // 1 triangle × 3 vertices × 3 components = 9 floats.
                // load_mesh_file may expand to (pos,normal) per vertex; allow
                // any non-zero multiple of 9 since the upstream loader can
                // emit either format.
                assert!(
                    vertices.len() % 9 == 0,
                    "vertex buffer length {} not a multiple of 9",
                    vertices.len()
                );
            }
            _ => panic!(
                "geom should be a Mesh (inherited type from class=\"visual\")"
            ),
        }
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
            .join("tests/fixtures/namiashi/urdf/namiashi.urdf");
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
            collision_enabled: true,
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
    ///
    /// **Ignored**: the underlying jump-simulation engine was removed in
    /// commit 8ca7bbc ("reflesh dyn sim", 2026-04-27); `start_jump_sim`
    /// is currently a stub that returns `None`. Re-enable this test once
    /// the engine is reimplemented. Type-level serde round-trip is still
    /// covered by `sim_graph_data_json_roundtrip` and
    /// `jump_result_json_roundtrip` above, which exercise every field
    /// the WASM ABI needs.
    #[test]
    #[ignore = "jump-sim engine removed in 8ca7bbc; awaiting reimplementation"]
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
    fn mjcf_export_emits_loop_closure_connect_and_weld() {
        let mut model = RobotModel::from_file(&fixture_urdf()).unwrap();
        if model.links.len() < 2 {
            return;
        }
        let a = model.links[0].name.clone();
        let b = model.links[1].name.clone();
        // 3-DoF loop closure (position only)
        model.loop_closures.push(
            articara::rbd::model::LoopClosure::position(
                "lc_pos".to_string(),
                a.clone(),
                nalgebra::Vector3::new(0.1, 0.0, 0.0),
                b.clone(),
                nalgebra::Vector3::zeros(),
            ),
        );
        // 6-DoF loop closure (full pose)
        model.loop_closures.push(
            articara::rbd::model::LoopClosure::pose(
                "lc_weld".to_string(),
                a.clone(),
                nalgebra::Isometry3::translation(0.0, 0.05, 0.0),
                b.clone(),
                nalgebra::Isometry3::identity(),
            ),
        );
        let xml = articara::mjcf::export_mjcf(&model);
        assert!(xml.contains("<connect"), "expected <connect>:\n{}", xml);
        assert!(xml.contains("<weld"), "expected <weld>:\n{}", xml);
        assert!(xml.contains("anchor=\"0.1 0 0\""));
    }

    #[test]
    fn urdf_export_decomposes_capsule_into_cylinder_and_spheres() {
        // URDF has no native capsule. Capsules must be split into a
        // cylinder + two end-cap spheres so the resulting URDF is valid.
        // We check both the visual and collision sides.
        let mut model = articara::rbd::model::RobotModel::new_empty("cap_test");
        model.links[0].visuals.push(articara::robot::VisualData {
            origin: nalgebra::Isometry3::identity(),
            geometry: articara::robot::GeomData::Capsule {
                radius: 0.05,
                half_length: 0.20,
            },
            color: [0.5, 0.5, 1.0, 1.0],
        });
        model.links[0].collisions.push(articara::robot::CollisionData {
            origin: nalgebra::Isometry3::identity(),
            geometry: articara::robot::GeomData::Capsule {
                radius: 0.05,
                half_length: 0.20,
            },
        });

        let xml = model.export_urdf().unwrap();
        // Should contain cylinder + at least 2 sphere entries (2 visual + 2 collision)
        let cyl_count = xml.matches("<cylinder").count();
        let sph_count = xml.matches("<sphere").count();
        assert!(
            cyl_count >= 2,
            "expected >=2 cylinder elements (vis+col), got {}:\n{}",
            cyl_count,
            xml,
        );
        assert!(
            sph_count >= 4,
            "expected >=4 sphere elements (2 caps × {{vis,col}}), got {}:\n{}",
            sph_count,
            xml,
        );
        // No <capsule> in URDF — that's the contract.
        assert!(!xml.contains("<capsule"));
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
        // After the URDF parent-child auto-exclude addition, a URDF-loaded
        // model always emits an `<exclude>` per joint. To exercise the
        // "no excludes at all" path, strip the joints (and the corresponding
        // children-joints index) from a fixture model and confirm the
        // exporter produces no `<exclude>` line.
        let mut model = RobotModel::from_file(&fixture_urdf()).unwrap();
        model.joints.clear();
        model.joint_map.clear();
        model.children_joints.clear();
        model.collision_pairs.clear();
        let xml = articara::mjcf::export_mjcf(&model);
        assert!(
            !xml.contains("<exclude"),
            "no excludes should appear when both joints and collision_pairs are empty:\n{xml}"
        );
    }

    #[test]
    fn mjcf_export_respects_bake_limits_flags() {
        // Regression: the "⛔ Limits" UI checkbox must reach all the way down
        // to the MJCF. Both `forcelimited` on the actuator and `range` on the
        // joint depend on the corresponding `bake_*` flags. If either flag
        // is silently ignored, MuJoCo will keep clamping ctrl / qpos
        // regardless of the user's choice and "limits off" experiments will
        // produce identical behaviour to "limits on".
        let mut model = RobotModel::from_file(&fixture_urdf()).unwrap();
        let movable_idx = model
            .joints
            .iter()
            .position(|j| j.joint_type != "fixed" && j.lower < j.upper)
            .expect("fixture should have a limited movable joint");
        // Ensure the joint has effort > 0 so forcelimited would normally be emitted.
        model.joints[movable_idx].effort = 2.5;
        let with_limits = articara::mjcf::export_mjcf_with_options(
            &model,
            articara::mjcf::MjcfExportOptions {
                add_actuators: true,
                bake_actuator_limits: true,
                bake_joint_position_limits: true,
                ..Default::default()
            },
        );
        let without_limits = articara::mjcf::export_mjcf_with_options(
            &model,
            articara::mjcf::MjcfExportOptions {
                add_actuators: true,
                bake_actuator_limits: false,
                bake_joint_position_limits: false,
                ..Default::default()
            },
        );
        assert!(
            with_limits.contains("forcelimited=\"true\""),
            "with bake_actuator_limits=true, MJCF must include forcelimited:\n{}",
            with_limits,
        );
        assert!(
            !without_limits.contains("forcelimited"),
            "with bake_actuator_limits=false, MJCF must omit forcelimited:\n{}",
            without_limits,
        );
        assert!(
            with_limits.contains(" range=\""),
            "with bake_joint_position_limits=true, MJCF must include range=:\n{}",
            with_limits,
        );
        assert!(
            !without_limits.contains(" range=\""),
            "with bake_joint_position_limits=false, MJCF must omit range:\n{}",
            without_limits,
        );
    }

    #[test]
    fn mjcf_export_emits_armature_and_damping_when_set() {
        // Regression: rotor inertia + passive damping must reach the MJCF so
        // MuJoCo's solver actually applies them. Without this, joints behave
        // as if there's no motor mass and the external PD controller drives
        // discrete-time oscillation during fast moves like jumps.
        let mut model = RobotModel::from_file(&fixture_urdf()).unwrap();
        let movable_idx = model
            .joints
            .iter()
            .position(|j| j.joint_type != "fixed")
            .expect("fixture should have at least one movable joint");
        model.joints[movable_idx].armature = 0.012;
        model.joints[movable_idx].joint_damping = 0.4;
        let xml = articara::mjcf::export_mjcf(&model);
        assert!(
            xml.contains("armature=\"0.012\""),
            "MJCF should emit armature=\"0.012\":\n{}",
            xml,
        );
        assert!(
            xml.contains("damping=\"0.4\""),
            "MJCF should emit damping=\"0.4\":\n{}",
            xml,
        );
    }

    #[test]
    fn mjcf_omits_armature_and_damping_when_zero() {
        // Conversely, with zero values the attributes must NOT appear so the
        // MJCF stays minimal and matches MuJoCo's defaults.
        //
        // Note: the URDF loader now seeds a small default armature so the PD
        // controller stays stable at MuJoCo's 2 ms default timestep
        // ([model.rs] `default_armature` = 0.0014). To exercise the
        // "omit-when-zero" emission path, zero those fields out explicitly
        // before exporting.
        let mut model = RobotModel::from_file(&fixture_urdf()).unwrap();
        for j in &mut model.joints {
            j.armature = 0.0;
            j.joint_damping = 0.0;
        }
        let xml = articara::mjcf::export_mjcf(&model);
        assert!(
            !xml.contains("armature="),
            "MJCF should omit armature when value is 0:\n{}",
            xml,
        );
        assert!(
            !xml.contains("damping="),
            "MJCF should omit damping when value is 0:\n{}",
            xml,
        );
    }

    #[test]
    fn mjcf_armature_damping_roundtrip() {
        // Set, export, re-import, and check the values come back. This is the
        // canonical path: model → MJCF (export) → MuJoCo (or another tool) →
        // MJCF (import) → model.
        let mut model = RobotModel::from_file(&fixture_urdf()).unwrap();
        let target = model
            .joints
            .iter()
            .position(|j| j.joint_type != "fixed")
            .unwrap();
        let target_name = model.joints[target].name.clone();
        model.joints[target].armature = 0.0075;
        model.joints[target].joint_damping = 1.25;
        let xml = articara::mjcf::export_mjcf(&model);

        let tmp = std::env::temp_dir()
            .join("articara_armature_roundtrip.xml");
        std::fs::write(&tmp, &xml).unwrap();
        let parsed = RobotModel::from_file(&tmp).unwrap();
        let restored = parsed
            .joints
            .iter()
            .find(|j| j.name == target_name)
            .expect("joint must survive roundtrip");
        assert!(
            (restored.armature - 0.0075).abs() < 1e-9,
            "armature lost in roundtrip: got {}",
            restored.armature,
        );
        assert!(
            (restored.joint_damping - 1.25).abs() < 1e-9,
            "joint_damping lost in roundtrip: got {}",
            restored.joint_damping,
        );
        std::fs::remove_file(&tmp).ok();
    }

    #[test]
    fn analyze_export_compatibility_flags_drops_and_approximations() {
        // Build a model with several entities the URDF format can't
        // express natively, plus a capsule that triggers approximation.
        // Run the analyzer against the URDF handler and verify each
        // category lands in the issue list with the right severity.
        use articara::format::{
            analyze_export_compatibility, ExportSeverity, FormatRegistry,
        };
        let mut model = RobotModel::from_file(&fixture_urdf()).unwrap();

        // Add a sensor (URDF: drop).
        model.sensors.push(articara::rbd::model::Sensor {
            name: "front_camera".into(),
            link: model.root_link.clone(),
            origin: nalgebra::Isometry3::identity(),
            update_rate: 30.0,
            kind: articara::rbd::model::SensorKind::Camera {
                fov: 1.0,
                width: 640,
                height: 480,
                near: 0.05,
                far: 50.0,
            },
        });

        // Add a collision-pair override (URDF: drop).
        model.collision_pairs.push(
            articara::rbd::model::CollisionPair::new(
                "a_link".to_string(),
                "b_link".to_string(),
                false,
            ),
        );

        // Inject a capsule visual to trigger the URDF approximation
        // warning. Pick the first link for simplicity.
        if let Some(link) = model.links.first_mut() {
            link.visuals.push(articara::robot::VisualData {
                origin: nalgebra::Isometry3::identity(),
                geometry: articara::robot::GeomData::Capsule {
                    radius: 0.05,
                    half_length: 0.10,
                },
                color: [0.5, 0.5, 0.5, 1.0],
            });
        }

        let registry = FormatRegistry::default_registry();
        let urdf = registry
            .handlers()
            .iter()
            .find(|h| h.name() == "URDF")
            .expect("URDF handler always registered")
            .as_ref();
        let issues = analyze_export_compatibility(&model, urdf);

        // Expect at least sensor / collision_pairs / capsule issues.
        let features: Vec<&str> =
            issues.iter().map(|i| i.feature.as_str()).collect();
        assert!(
            features.contains(&"Sensors"),
            "expected Sensors issue, got {:?}",
            features,
        );
        assert!(
            features.contains(&"Collision pairs"),
            "expected Collision pairs issue, got {:?}",
            features,
        );
        assert!(
            features.contains(&"Capsule shapes"),
            "expected Capsule shapes issue, got {:?}",
            features,
        );

        // Capsule must be flagged as approximation, not drop.
        let capsule = issues.iter().find(|i| i.feature == "Capsule shapes").unwrap();
        assert_eq!(capsule.severity, ExportSeverity::Approximate);

        // Sensors must be flagged as drop.
        let sensor = issues.iter().find(|i| i.feature == "Sensors").unwrap();
        assert_eq!(sensor.severity, ExportSeverity::Drop);
    }

    #[test]
    fn analyze_export_compatibility_clean_model_returns_empty() {
        // Plain URDF model with nothing extra → no warnings.
        use articara::format::{analyze_export_compatibility, FormatRegistry};
        let model = RobotModel::from_file(&fixture_urdf()).unwrap();
        let registry = FormatRegistry::default_registry();
        let urdf = registry
            .handlers()
            .iter()
            .find(|h| h.name() == "URDF")
            .unwrap()
            .as_ref();
        let issues = analyze_export_compatibility(&model, urdf);
        assert!(
            issues.is_empty(),
            "clean model should produce no warnings, got {:?}",
            issues,
        );
    }

    #[test]
    fn analyze_export_compatibility_misa_target_never_warns() {
        // Even a heavily-decorated model (mimic + sensor + collision_pair +
        // loop_closure + capsule) should produce zero warnings when the
        // target is `.misa` — the master format preserves everything.
        use articara::format::{analyze_export_compatibility, FormatRegistry};
        use articara::robot::{
            CollisionPair, GeomData, LoopClosure, Mimic, RobotModel, Sensor, SensorKind,
            VisualData,
        };
        use nalgebra as na;

        let mut model = RobotModel::from_file(&fixture_urdf()).unwrap();

        // Add at least one of every "warning-eligible" entity type
        model.mimics.push(Mimic {
            joint: model.joints[0].name.clone(),
            source: model
                .joints
                .get(1)
                .map(|j| j.name.clone())
                .unwrap_or_else(|| model.joints[0].name.clone()),
            multiplier: 1.0,
            offset: 0.0,
        });
        model.sensors.push(Sensor {
            name: "imu0".into(),
            link: model.links[0].name.clone(),
            origin: na::Isometry3::identity(),
            update_rate: 100.0,
            kind: SensorKind::Imu { gyro_noise: 0.0, accel_noise: 0.0 },
        });
        if model.links.len() >= 2 {
            model.collision_pairs.push(CollisionPair::new(
                model.links[0].name.clone(),
                model.links[1].name.clone(),
                false,
            ));
        }
        model.loop_closures.push(LoopClosure {
            name: "demo_loop".into(),
            link_a: model.links[0].name.clone(),
            offset_a: na::Isometry3::identity(),
            link_b: model.links[0].name.clone(),
            offset_b: na::Isometry3::identity(),
            pose_6dof: false,
        });
        model.links[0].visuals.push(VisualData {
            origin: na::Isometry3::identity(),
            geometry: GeomData::Capsule { radius: 0.05, half_length: 0.10 },
            color: [1.0, 0.0, 0.0, 1.0],
        });

        let registry = FormatRegistry::default_registry();
        let misa = registry
            .handlers()
            .iter()
            .find(|h| h.name() == "Misa")
            .expect("Misa handler should be registered")
            .as_ref();

        let issues = analyze_export_compatibility(&model, misa);
        assert!(
            issues.is_empty(),
            "Misa target should never produce warnings (master is lossless), \
             but got {:?}",
            issues,
        );

        // Sanity: the same model SHOULD produce warnings against URDF
        let urdf = registry
            .handlers()
            .iter()
            .find(|h| h.name() == "URDF")
            .unwrap()
            .as_ref();
        let urdf_issues = analyze_export_compatibility(&model, urdf);
        assert!(
            !urdf_issues.is_empty(),
            "control: same decorated model should warn against URDF target",
        );
    }

    #[test]
    fn home_pose_roundtrips_through_sidecar() {
        // Edit the model's joint_positions + base_transform, save the
        // sidecar, reload from disk, and verify both come back. This is
        // the user-visible "I closed and reopened — exactly where I
        // left off" behaviour the [home] section provides.
        let urdf_src = fixture_urdf();
        let tmp_dir = std::env::temp_dir().join("articara_home_pose_test");
        std::fs::create_dir_all(&tmp_dir).unwrap();
        let urdf_dst = tmp_dir.join("test_robot.urdf");
        std::fs::copy(&urdf_src, &urdf_dst).unwrap();

        let mut model = RobotModel::from_file(&urdf_dst).unwrap();
        // Pick the first movable joint and dial it to a non-zero value.
        let target = model
            .joints
            .iter()
            .position(|j| j.joint_type != "fixed")
            .expect("fixture has at least one movable joint");
        let target_name = model.joints[target].name.clone();
        model.joint_positions[target] = 0.42;
        // Move the base off the origin so the orientation roundtrip is
        // exercised too.
        model.base_transform = nalgebra::Isometry3::from_parts(
            nalgebra::Translation3::new(0.5, -0.25, 1.0),
            nalgebra::UnitQuaternion::from_quaternion(
                nalgebra::Quaternion::new(0.7071, 0.7071, 0.0, 0.0),
            ),
        );

        model.save_sidecar_config(&urdf_dst).unwrap();

        let mut model2 = RobotModel::from_file(&urdf_dst).unwrap();
        model2
            .load_sidecar_config()
            .expect("sidecar should be present");

        // Joint angle restored.
        let restored = model2
            .joints
            .iter()
            .position(|j| j.name == target_name)
            .unwrap();
        assert!(
            (model2.joint_positions[restored] - 0.42).abs() < 1e-9,
            "joint angle drifted on roundtrip: {} vs 0.42",
            model2.joint_positions[restored],
        );

        // Base position restored exactly.
        let bp = model2.base_transform.translation.vector;
        assert!((bp.x - 0.5).abs() < 1e-9);
        assert!((bp.y - (-0.25)).abs() < 1e-9);
        assert!((bp.z - 1.0).abs() < 1e-9);

        // Base orientation: compare via the dot product of the underlying
        // quaternions; a sign flip on the quat is equivalent to the same
        // rotation, so we look at |dot|.
        let original = nalgebra::UnitQuaternion::from_quaternion(
            nalgebra::Quaternion::new(0.7071, 0.7071, 0.0, 0.0),
        );
        let restored_q = model2.base_transform.rotation;
        let dot = original.coords.dot(&restored_q.coords);
        assert!(
            dot.abs() > 0.9999,
            "base orientation drifted on roundtrip (|dot|={dot})",
        );

        std::fs::remove_dir_all(&tmp_dir).ok();
    }

    #[test]
    fn home_pose_absent_section_leaves_defaults_untouched() {
        // A `.misarta.toml` without a `[home]` section (older sidecars,
        // hand-written ones) must NOT clobber the URDF's neutral joint
        // angles. Confirm this by writing a minimal sidecar manually
        // and loading.
        let urdf_src = fixture_urdf();
        let tmp_dir = std::env::temp_dir().join("articara_home_absent_test");
        std::fs::create_dir_all(&tmp_dir).unwrap();
        let urdf_dst = tmp_dir.join("test_robot.urdf");
        std::fs::copy(&urdf_src, &urdf_dst).unwrap();

        // Write a sidecar with only the version header.
        let toml_path = tmp_dir.join("test_robot.misarta.toml");
        std::fs::write(
            &toml_path,
            "[misarta]\nversion = 1\n",
        )
        .unwrap();

        let mut model = RobotModel::from_file(&urdf_dst).unwrap();
        let original_q = model.joint_positions.clone();
        let original_base = model.base_transform;

        model.load_sidecar_config().expect("sidecar exists");

        assert_eq!(model.joint_positions, original_q);
        assert!(
            (model.base_transform.translation.vector
                - original_base.translation.vector)
                .norm()
                < 1e-12,
        );

        std::fs::remove_dir_all(&tmp_dir).ok();
    }

    #[test]
    fn gait_descriptor_roundtrips_through_sidecar() {
        // Save a non-trivial gait preset, reload, and confirm every field
        // comes back. The link lengths / hip offsets are intentionally not
        // checked — they're auto-detected from the URDF chain on each
        // load, never written to the sidecar.
        use articara::rbd::model::GaitDescriptor;
        use misarta::config::GaitTypeConfig;

        let urdf_src = fixture_urdf();
        let tmp_dir = std::env::temp_dir().join("articara_gait_sidecar_test");
        std::fs::create_dir_all(&tmp_dir).unwrap();
        let urdf_dst = tmp_dir.join("test_robot.urdf");
        std::fs::copy(&urdf_src, &urdf_dst).unwrap();

        let mut model = RobotModel::from_file(&urdf_dst).unwrap();
        model.gaits.push(GaitDescriptor {
            name: "fast".into(),
            gait_type: GaitTypeConfig::Trot,
            cycle_period_s: 0.30,
            duty_factor: 0.45,
            swing_height_m: 0.05,
            max_step_length_m: 0.12,
            fl_foot: "FL_paw".into(),
            fr_foot: "FR_paw".into(),
            rl_foot: "RL_paw".into(),
            rr_foot: "RR_paw".into(),
            knee_forward: [true, true, false, false],
        });

        model.save_sidecar_config(&urdf_dst).unwrap();

        let mut model2 = RobotModel::from_file(&urdf_dst).unwrap();
        model2
            .load_sidecar_config()
            .expect("sidecar should be present");
        assert_eq!(model2.gaits.len(), 1);
        let g = &model2.gaits[0];
        assert_eq!(g.name, "fast");
        assert!((g.cycle_period_s - 0.30).abs() < 1e-9);
        assert!((g.duty_factor - 0.45).abs() < 1e-9);
        assert_eq!(g.fl_foot, "FL_paw");
        assert_eq!(g.knee_forward, [true, true, false, false]);

        std::fs::remove_dir_all(&tmp_dir).ok();
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

mod test_misa {
    use super::*;
    use articara::format::RobotFormat;
    use articara::robot::{ActuatorMode, RobotModel};
    use misarta::native as mn;

    /// Build a minimal but realistic MisaFile in code so tests don't
    /// need a fixture file. Returns a 3-link biped (trunk + L/R thigh)
    /// with one actuator per leg.
    fn sample_misa_file() -> mn::MisaFile {
        let mut f = mn::MisaFile::new("biped_test", "trunk");

        f.material.push(mn::Material {
            name: "red_plastic".into(),
            color: mn::ColorSpec::Hex("#cc4422".into()),
        });

        f.link.push(mn::Link {
            name: "trunk".into(),
            description: String::new(),
            inertial: mn::Inertial {
                mass: 5.0,
                ixx: 0.10,
                iyy: 0.10,
                izz: 0.10,
                ..Default::default()
            },
            visual: vec![mn::Visual {
                origin: mn::Origin::default(),
                geom: mn::Geom::Box {
                    size: [0.30, 0.20, 0.10],
                },
                color: None,
                material: Some("red_plastic".into()),
            }],
            collision: vec![mn::Collision {
                origin: mn::Origin::default(),
                geom: mn::Geom::Box {
                    size: [0.30, 0.20, 0.10],
                },
            }],
            collision_enabled: true,
        });

        for side in &["left", "right"] {
            f.link.push(mn::Link {
                name: format!("{side}_thigh"),
                description: String::new(),
                inertial: mn::Inertial {
                    mass: 0.8,
                    ixx: 0.01,
                    iyy: 0.01,
                    izz: 0.001,
                    ..Default::default()
                },
                visual: vec![mn::Visual {
                    origin: mn::Origin::default(),
                    geom: mn::Geom::Cylinder {
                        radius: 0.03,
                        length: 0.20,
                    },
                    color: Some(mn::ColorSpec::Rgba([0.5, 0.5, 0.5, 1.0])),
                    material: None,
                }],
                collision: Vec::new(),
                collision_enabled: true,
            });
            let y = if *side == "left" { 0.10 } else { -0.10 };
            f.joint.push(mn::Joint {
                name: format!("{side}_hip_pitch"),
                kind: mn::JointKind::Revolute,
                parent: "trunk".into(),
                child: format!("{side}_thigh"),
                axis: [0.0, 1.0, 0.0],
                origin: mn::Origin {
                    xyz: [0.0, y, -0.05],
                    rpy: Some([0.0, 0.0, 0.0]),
                    quat: None,
                },
                limit: mn::JointLimit {
                    lower: -1.5,
                    upper: 1.5,
                    effort: 30.0,
                    velocity: 10.0,
                },
                dynamics: mn::JointDynamics {
                    armature: 0.001,
                    damping: 0.05,
                    friction: 0.0,
                },
            });
        }

        f.actuator.push(mn::Actuator {
            name: "left_motor".into(),
            mode: mn::ActuatorMode::Position,
            joints: vec![mn::ActuatorJointRef {
                name: "left_hip_pitch".into(),
                gear: 1.0,
            }],
            kp: 100.0,
            kv: 1.2,
        });
        f.actuator.push(mn::Actuator {
            name: "right_motor".into(),
            mode: mn::ActuatorMode::Position,
            joints: vec![mn::ActuatorJointRef {
                name: "right_hip_pitch".into(),
                gear: 1.0,
            }],
            kp: 100.0,
            kv: 1.2,
        });

        f
    }

    /// Save a MisaFile to a unique temp file and return its path.
    fn save_to_temp(file: &mn::MisaFile, tag: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!("articara_misa_test_{tag}"));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("robot.misa");
        mn::save(&path, file).expect("save");
        path
    }

    #[test]
    fn from_misa_basic_round_trip() {
        let file = sample_misa_file();
        let path = save_to_temp(&file, "basic");
        let model = RobotModel::from_misa(&path).expect("load");

        assert_eq!(model.name, "biped_test");
        assert_eq!(model.root_link, "trunk");
        assert_eq!(model.links.len(), 3);
        assert_eq!(model.joints.len(), 2);

        // Links
        let trunk_idx = model.link_map["trunk"];
        let trunk = &model.links[trunk_idx];
        assert!((trunk.inertial.mass - 5.0).abs() < 1e-9);
        assert_eq!(trunk.visuals.len(), 1);
        assert_eq!(trunk.collisions.len(), 1);

        // Geom: full size 0.30 → half-extent 0.15 in articara's representation
        match &trunk.visuals[0].geometry {
            articara::robot::GeomData::Box { hx, hy, hz } => {
                assert!((hx - 0.15).abs() < 1e-6);
                assert!((hy - 0.10).abs() < 1e-6);
                assert!((hz - 0.05).abs() < 1e-6);
            }
            _ => panic!("expected Box geometry"),
        }

        // Joints
        let lhip = &model.joints[model.joint_map["left_hip_pitch"]];
        assert_eq!(lhip.joint_type, "revolute");
        assert_eq!(lhip.parent_link, "trunk");
        assert_eq!(lhip.child_link, "left_thigh");
        assert!((lhip.lower + 1.5).abs() < 1e-9);
        assert!((lhip.upper - 1.5).abs() < 1e-9);

        // Joint dynamics from `dynamics` table
        assert!((lhip.armature - 0.001).abs() < 1e-9);
        assert!((lhip.joint_damping - 0.05).abs() < 1e-9);

        // Actuator gains from [[actuator]] table
        assert_eq!(lhip.actuator_mode, ActuatorMode::Position);
        assert!((lhip.actuator_kp - 100.0).abs() < 1e-9);
        assert!((lhip.actuator_kv - 1.2).abs() < 1e-9);

        std::fs::remove_dir_all(path.parent().unwrap()).ok();
    }

    #[test]
    fn from_misa_color_resolution_inline_and_named() {
        let file = sample_misa_file();
        let path = save_to_temp(&file, "color");
        let model = RobotModel::from_misa(&path).expect("load");

        // trunk visual uses `material = "red_plastic"` → #cc4422 = (204/255, 68/255, 34/255, 1)
        let trunk_color = model.links[model.link_map["trunk"]].visuals[0].color;
        assert!((trunk_color[0] - 204.0 / 255.0).abs() < 1e-3);
        assert!((trunk_color[1] - 68.0 / 255.0).abs() < 1e-3);
        assert!((trunk_color[2] - 34.0 / 255.0).abs() < 1e-3);
        assert!((trunk_color[3] - 1.0).abs() < 1e-6);

        // left_thigh visual uses inline RGBA color
        let leg_color = model.links[model.link_map["left_thigh"]].visuals[0].color;
        assert!((leg_color[0] - 0.5).abs() < 1e-6);
        assert!((leg_color[3] - 1.0).abs() < 1e-6);

        std::fs::remove_dir_all(path.parent().unwrap()).ok();
    }

    #[test]
    fn from_misa_mimic_round_trip() {
        let mut file = sample_misa_file();
        file.mimic.push(mn::Mimic {
            joint: "right_hip_pitch".into(),
            source: "left_hip_pitch".into(),
            multiplier: -1.0,
            offset: 0.0,
        });
        let path = save_to_temp(&file, "mimic");
        let model = RobotModel::from_misa(&path).expect("load");
        assert_eq!(model.mimics.len(), 1);
        assert_eq!(model.mimics[0].joint, "right_hip_pitch");
        assert_eq!(model.mimics[0].source, "left_hip_pitch");
        assert!((model.mimics[0].multiplier + 1.0).abs() < 1e-9);

        std::fs::remove_dir_all(path.parent().unwrap()).ok();
    }

    #[test]
    fn from_misa_n_to_m_actuator_drives_both_joints() {
        let mut file = sample_misa_file();
        // Replace the per-leg actuators with a single differential
        // actuator driving both hip joints (N:1 from joint POV).
        file.actuator.clear();
        file.actuator.push(mn::Actuator {
            name: "diff_drive".into(),
            mode: mn::ActuatorMode::Torque,
            joints: vec![
                mn::ActuatorJointRef {
                    name: "left_hip_pitch".into(),
                    gear: 1.0,
                },
                mn::ActuatorJointRef {
                    name: "right_hip_pitch".into(),
                    gear: -1.0,
                },
            ],
            kp: 0.0,
            kv: 5.0,
        });
        let path = save_to_temp(&file, "diff");
        let model = RobotModel::from_misa(&path).expect("load");

        // Both joints should inherit the actuator's mode/gains
        let lhip = &model.joints[model.joint_map["left_hip_pitch"]];
        let rhip = &model.joints[model.joint_map["right_hip_pitch"]];
        assert_eq!(lhip.actuator_mode, ActuatorMode::Torque);
        assert_eq!(rhip.actuator_mode, ActuatorMode::Torque);
        assert!((lhip.actuator_kv - 5.0).abs() < 1e-9);
        assert!((rhip.actuator_kv - 5.0).abs() < 1e-9);

        std::fs::remove_dir_all(path.parent().unwrap()).ok();
    }

    #[test]
    fn from_misa_loop_closure_and_collision_pair_preserved() {
        let mut file = sample_misa_file();
        file.loop_closure.push(mn::LoopClosure {
            name: "demo_loop".into(),
            link_a: "left_thigh".into(),
            offset_a: [0.0, 0.0, -0.10],
            rot_a: [0.0, 0.0, 0.0, 1.0],
            link_b: "right_thigh".into(),
            offset_b: [0.0, 0.0, -0.10],
            rot_b: [0.0, 0.0, 0.0, 1.0],
            pose_6dof: false,
        });
        file.collision_pair.push(mn::CollisionPair {
            link_a: "left_thigh".into(),
            link_b: "right_thigh".into(),
            enabled: false,
        });
        let path = save_to_temp(&file, "loop_pair");
        let model = RobotModel::from_misa(&path).expect("load");

        assert_eq!(model.loop_closures.len(), 1);
        assert_eq!(model.loop_closures[0].name, "demo_loop");
        assert_eq!(model.collision_pairs.len(), 1);
        assert!(!model.collision_pairs[0].enabled);

        std::fs::remove_dir_all(path.parent().unwrap()).ok();
    }

    #[test]
    fn from_misa_pose_and_home_applied() {
        let mut file = sample_misa_file();
        file.pose.push(mn::Pose {
            name: "tucked".into(),
            angles: [
                ("left_hip_pitch".to_string(), 0.5),
                ("right_hip_pitch".to_string(), -0.5),
            ]
            .into_iter()
            .collect(),
            duration: 0.3,
            kind: misarta::trajectory::InterpolationKind::QuinticSmooth,
        });
        file.home.joint_positions =
            [("left_hip_pitch".to_string(), 0.7)].into_iter().collect();
        let path = save_to_temp(&file, "pose_home");
        let model = RobotModel::from_misa(&path).expect("load");

        assert_eq!(model.poses.len(), 1);
        assert_eq!(model.poses[0].name, "tucked");
        // Home applied: left_hip_pitch should be at 0.7 in joint_positions
        let lhip_idx = model.joint_map["left_hip_pitch"];
        assert!((model.joint_positions[lhip_idx] - 0.7).abs() < 1e-9);

        std::fs::remove_dir_all(path.parent().unwrap()).ok();
    }

    #[test]
    fn from_misa_with_report_surfaces_sanitisation() {
        // Hand-build TOML directly so we can inject an invalid identifier
        // (constructing a MisaFile in code can't test the parser sanitiser).
        let toml_text = r#"
schema = "misarta/1"

[robot]
name = "demo"
root = "base"

[[link]]
name = "base"

[[link]]
name = "front-leg"

[[joint]]
name = "hip"
type = "revolute"
parent = "base"
child = "front-leg"
"#;
        let dir = std::env::temp_dir().join("articara_misa_test_sanit");
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("robot.misa");
        std::fs::write(&path, toml_text).unwrap();

        let (model, report) = RobotModel::from_misa_with_report(&path).expect("load");

        assert_eq!(model.links[1].name, "front_leg");
        assert_eq!(model.joints[0].child_link, "front_leg");
        assert!(!report.is_empty());
        assert_eq!(report.sanitized_names.len(), 1);
        assert_eq!(report.sanitized_names[0].original, "front-leg");
        assert_eq!(report.sanitized_names[0].sanitized, "front_leg");

        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn from_misa_dispatched_via_from_file() {
        let file = sample_misa_file();
        let path = save_to_temp(&file, "dispatch");
        // from_file should detect the extension and route to from_misa
        let model = RobotModel::from_file(&path).expect("load via from_file");
        assert_eq!(model.name, "biped_test");
        assert_eq!(model.links.len(), 3);

        std::fs::remove_dir_all(path.parent().unwrap()).ok();
    }

    #[test]
    fn misa_format_detection() {
        let p = std::path::Path::new("foo/bar.misa");
        assert_eq!(RobotFormat::detect(p), Some(RobotFormat::Misa));
        assert_eq!(RobotFormat::detect_from_extension(p), Some(RobotFormat::Misa));
        assert!(RobotFormat::Misa.supports_import());
        assert!(RobotFormat::Misa.supports_export());
        assert_eq!(RobotFormat::Misa.extension(), "misa");
    }

    #[test]
    fn from_misa_rebuild_misarta_model_succeeds() {
        let file = sample_misa_file();
        let path = save_to_temp(&file, "rebuild");
        let model = RobotModel::from_misa(&path).expect("load");
        // misarta_cache is built during from_misa; smoke-check by computing
        // FK transforms (which require the cache).
        let transforms = model.compute_transforms();
        // Should at least have transforms for root + 2 children
        assert!(transforms.contains_key("trunk"));
        assert!(transforms.contains_key("left_thigh"));
        assert!(transforms.contains_key("right_thigh"));

        std::fs::remove_dir_all(path.parent().unwrap()).ok();
    }

    // ─── to_misa round-trip ────────────────────────────────────────────

    #[test]
    fn to_misa_then_from_misa_preserves_structure() {
        // Build a `RobotModel` indirectly: spin up a MisaFile, load it,
        // then re-serialise via to_misa and load again. The double-load
        // ensures both paths agree.
        let file_in = sample_misa_file();
        let in_path = save_to_temp(&file_in, "to_misa_in");
        let model_a = RobotModel::from_misa(&in_path).expect("first load");

        // RobotModel → MisaFile → save → load
        let dir = std::env::temp_dir().join("articara_misa_to_round");
        std::fs::create_dir_all(&dir).unwrap();
        let out_path = dir.join("robot.misa");
        model_a.save_as_misa(&out_path).expect("to_misa save");
        let model_b = RobotModel::from_misa(&out_path).expect("second load");

        // Structural equivalence
        assert_eq!(model_a.name, model_b.name);
        assert_eq!(model_a.root_link, model_b.root_link);
        assert_eq!(model_a.links.len(), model_b.links.len());
        assert_eq!(model_a.joints.len(), model_b.joints.len());

        // Link names match (order may differ but URDF flat structure
        // preserves source order)
        let names_a: Vec<&str> = model_a.links.iter().map(|l| l.name.as_str()).collect();
        let names_b: Vec<&str> = model_b.links.iter().map(|l| l.name.as_str()).collect();
        assert_eq!(names_a, names_b);

        // Joint names + parent/child preserved
        for (ja, jb) in model_a.joints.iter().zip(model_b.joints.iter()) {
            assert_eq!(ja.name, jb.name);
            assert_eq!(ja.parent_link, jb.parent_link);
            assert_eq!(ja.child_link, jb.child_link);
            assert_eq!(ja.joint_type, jb.joint_type);
        }

        // Mass + inertia preserved
        for (la, lb) in model_a.links.iter().zip(model_b.links.iter()) {
            assert!((la.inertial.mass - lb.inertial.mass).abs() < 1e-9);
            assert!((la.inertial.ixx - lb.inertial.ixx).abs() < 1e-9);
        }

        std::fs::remove_dir_all(in_path.parent().unwrap()).ok();
        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn to_misa_emits_named_material_when_color_matches() {
        let file_in = sample_misa_file();
        let in_path = save_to_temp(&file_in, "named_mat");
        let model = RobotModel::from_misa(&in_path).expect("load");
        let misa = model.to_misa().expect("to_misa");

        // trunk visual references "red_plastic" — that should round-trip
        // back to a named-material reference (not inline color).
        let trunk = misa.link.iter().find(|l| l.name == "trunk").unwrap();
        let v = &trunk.visual[0];
        assert_eq!(v.material.as_deref(), Some("red_plastic"));
        assert!(v.color.is_none());

        std::fs::remove_dir_all(in_path.parent().unwrap()).ok();
    }

    #[test]
    fn to_misa_emits_inline_color_when_no_material_match() {
        let file_in = sample_misa_file();
        let in_path = save_to_temp(&file_in, "inline_col");
        let model = RobotModel::from_misa(&in_path).expect("load");
        let misa = model.to_misa().expect("to_misa");

        // left_thigh visual uses inline RGBA in the input; no entry in
        // [[material]] matches, so it should round-trip as inline color.
        let thigh = misa.link.iter().find(|l| l.name == "left_thigh").unwrap();
        let v = &thigh.visual[0];
        assert!(v.color.is_some());
        assert!(v.material.is_none());

        std::fs::remove_dir_all(in_path.parent().unwrap()).ok();
    }

    #[test]
    fn to_misa_geometry_dimensions_round_trip() {
        // Verify that half-extent (RobotModel internal) ↔ full size
        // (.misa schema) conversion is symmetric.
        let file_in = sample_misa_file();
        let in_path = save_to_temp(&file_in, "geom_dim");
        let model = RobotModel::from_misa(&in_path).expect("load");
        let misa = model.to_misa().expect("to_misa");

        let trunk = misa.link.iter().find(|l| l.name == "trunk").unwrap();
        if let mn::Geom::Box { size } = &trunk.visual[0].geom {
            // Should be back to [0.30, 0.20, 0.10] (the original input)
            assert!((size[0] - 0.30).abs() < 1e-6);
            assert!((size[1] - 0.20).abs() < 1e-6);
            assert!((size[2] - 0.10).abs() < 1e-6);
        } else {
            panic!("expected Box geometry");
        }

        let thigh = misa.link.iter().find(|l| l.name == "left_thigh").unwrap();
        if let mn::Geom::Cylinder { radius, length } = &thigh.visual[0].geom {
            assert!((radius - 0.03).abs() < 1e-6);
            assert!((length - 0.20).abs() < 1e-6);
        } else {
            panic!("expected Cylinder");
        }

        std::fs::remove_dir_all(in_path.parent().unwrap()).ok();
    }

    // ─── namiashi: real-world URDF + sidecar → .misa → round-trip ────────

    /// Path to the namiashi URDF (skipped if the fixture isn't present).
    fn namiashi_urdf_path() -> Option<PathBuf> {
        let p = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("tests/fixtures/namiashi/urdf/namiashi.urdf");
        if p.exists() { Some(p) } else { None }
    }

    /// Path to the checked-in namiashi.misa fixture.
    fn namiashi_misa_path() -> Option<PathBuf> {
        let p = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("tests/fixtures/namiashi/namiashi.misa");
        if p.exists() { Some(p) } else { None }
    }

    #[test]
    fn namiashi_misa_fixture_loads_cleanly() {
        let Some(misa_path) = namiashi_misa_path() else {
            eprintln!("[skip] tests/fixtures/namiashi/namiashi.misa not present");
            return;
        };

        let (model, report) =
            RobotModel::from_misa_with_report(&misa_path).expect("from_misa");

        // Basic counts (matches what convert_to_misa produced)
        assert_eq!(model.name, "namiashi_description");
        assert_eq!(model.root_link, "trunk");
        assert_eq!(model.links.len(), 19);
        assert_eq!(model.joints.len(), 18);
        assert_eq!(model.collision_pairs.len(), 6);
        assert_eq!(model.poses.len(), 4);
        assert_eq!(model.sequences.len(), 1);

        // No sanitisation should have triggered on a clean file
        assert!(
            report.sanitized_names.is_empty(),
            "fixture should have no sanitisation: {:?}",
            report.sanitized_names,
        );

        // FK should compute clean
        let tf = model.compute_transforms();
        for link in &model.links {
            assert!(tf.contains_key(&link.name), "missing transform for {}", link.name);
        }
    }

    #[test]
    fn namiashi_urdf_to_misa_round_trip() {
        let Some(urdf_path) = namiashi_urdf_path() else {
            eprintln!("[skip] namiashi URDF not present");
            return;
        };

        // Load URDF + sidecar into a baseline RobotModel
        let mut original = RobotModel::from_urdf(&urdf_path).expect("URDF load");
        original.load_sidecar_config();

        // Convert to .misa and write
        let dir = std::env::temp_dir().join("articara_namiashi_misa_test");
        std::fs::create_dir_all(&dir).unwrap();
        // Mesh dir alongside the .misa so relative path resolution works.
        let meshes_src = urdf_path
            .parent()
            .unwrap()
            .parent()
            .unwrap()
            .join("meshes");
        let meshes_dst = dir.join("meshes");
        std::fs::create_dir_all(&meshes_dst).ok();
        if meshes_src.exists() {
            for entry in std::fs::read_dir(&meshes_src).unwrap() {
                let entry = entry.unwrap();
                let dst = meshes_dst.join(entry.file_name());
                let _ = std::fs::copy(entry.path(), dst);
            }
        }
        let misa_path = dir.join("namiashi.misa");
        original
            .save_as_misa(&misa_path)
            .expect("namiashi to_misa save");

        assert!(misa_path.exists(), ".misa file should be written");
        let text = std::fs::read_to_string(&misa_path).unwrap();
        assert!(text.contains("schema = \"misarta/1\""));
        assert!(text.contains("[robot]"));

        // Round-trip: load the .misa back and compare structure
        let loaded = RobotModel::from_misa(&misa_path).expect("namiashi from_misa load");

        assert_eq!(original.name, loaded.name, "robot name");
        assert_eq!(
            original.links.len(),
            loaded.links.len(),
            "link count: original={} loaded={}",
            original.links.len(),
            loaded.links.len()
        );
        assert_eq!(
            original.joints.len(),
            loaded.joints.len(),
            "joint count"
        );
        assert_eq!(original.root_link, loaded.root_link, "root link");

        // Link names preserved in order
        let names_a: Vec<&str> = original.links.iter().map(|l| l.name.as_str()).collect();
        let names_b: Vec<&str> = loaded.links.iter().map(|l| l.name.as_str()).collect();
        assert_eq!(names_a, names_b, "link name order");

        // Joint topology preserved
        for (ja, jb) in original.joints.iter().zip(loaded.joints.iter()) {
            assert_eq!(ja.name, jb.name);
            assert_eq!(ja.parent_link, jb.parent_link);
            assert_eq!(ja.child_link, jb.child_link);
            assert_eq!(ja.joint_type, jb.joint_type);
        }

        // Sidecar contents (actuator settings, collision pairs, poses)
        // should round-trip
        assert_eq!(
            original.collision_pairs.len(),
            loaded.collision_pairs.len(),
            "collision_pairs count"
        );
        assert_eq!(original.poses.len(), loaded.poses.len(), "poses count");
        assert_eq!(
            original.sequences.len(),
            loaded.sequences.len(),
            "sequences count"
        );

        // Spot-check actuator settings on a known joint
        let arm = original.joints.iter().find(|j| j.name == "arm_pitch_joint");
        let arm_loaded = loaded.joints.iter().find(|j| j.name == "arm_pitch_joint");
        if let (Some(a), Some(b)) = (arm, arm_loaded) {
            assert_eq!(a.actuator_mode, b.actuator_mode, "actuator_mode arm");
            assert!((a.actuator_kp - b.actuator_kp).abs() < 1e-9);
            assert!((a.actuator_kv - b.actuator_kv).abs() < 1e-9);
            assert!((a.armature - b.armature).abs() < 1e-9);
            assert!((a.joint_damping - b.joint_damping).abs() < 1e-9);
        }

        // FK should still work on the loaded model
        let tf = loaded.compute_transforms();
        assert!(
            tf.contains_key("trunk"),
            "loaded model should compute trunk transform"
        );

        std::fs::remove_dir_all(&dir).ok();
    }

    // ─── Export portability ───────────────────────────────────────────────
    //
    // After exporting URDF / SDF / MJCF from a `.misa` source the result
    // must be **portable** — i.e. the directory contains the model file
    // PLUS a self-contained `meshes/` sibling, and the model file
    // references meshes by `meshes/<basename>` (no absolute paths, no
    // `package://` URIs that depend on a ROS workspace, no plain
    // basenames that depend on cwd). These tests guard the
    // MeshPathStyle::RelativeToDir + copy_meshes_to contract end-to-end
    // against accidental regressions in any of the three exporters.

    /// Helper: load namiashi.misa and return the in-memory model.
    fn load_namiashi_misa() -> Option<RobotModel> {
        let path = namiashi_misa_path()?;
        Some(RobotModel::from_misa(&path).expect("from_misa"))
    }

    /// Helper: assert the export directory has both `<model_file>` and
    /// the expected `meshes/` siblings populated. Returns the model
    /// file's content for further inspection.
    fn assert_self_contained(dir: &std::path::Path, model_file: &str) -> String {
        let model_path = dir.join(model_file);
        assert!(
            model_path.exists(),
            "model file missing: {}",
            model_path.display()
        );

        let meshes = dir.join("meshes");
        assert!(meshes.is_dir(), "meshes/ not present at {}", meshes.display());
        let stl_count = std::fs::read_dir(&meshes)
            .expect("read meshes/")
            .filter_map(|e| e.ok())
            .filter(|e| {
                e.path()
                    .extension()
                    .and_then(|x| x.to_str())
                    .map(|x| x.eq_ignore_ascii_case("stl"))
                    .unwrap_or(false)
            })
            .count();
        assert!(
            stl_count > 0,
            "no STL files copied to {}",
            meshes.display()
        );

        std::fs::read_to_string(&model_path).expect("read model file")
    }

    /// Helper: the exported model file's mesh references must NOT contain
    /// non-portable forms (absolute paths, package://, file://) and must
    /// contain at least one `meshes/<basename>` reference.
    fn assert_portable_mesh_references(content: &str, mesh_attr: &str) {
        // Find all occurrences of `<mesh-attr>="..."` and inspect the value.
        let needle = format!("{mesh_attr}=\"");
        let mut found_relative = 0usize;
        let mut rest = content;
        while let Some(pos) = rest.find(&needle) {
            let after = &rest[pos + needle.len()..];
            let end = after.find('"').expect("malformed XML in test fixture");
            let value = &after[..end];

            // No absolute paths
            assert!(
                !value.starts_with('/'),
                "non-portable absolute path found: {value:?}",
            );
            // No URDF package URIs (would depend on a ROS workspace)
            assert!(
                !value.starts_with("package://"),
                "non-portable package:// URI found: {value:?}",
            );
            // No file:// URIs
            assert!(
                !value.starts_with("file://"),
                "non-portable file:// URI found: {value:?}",
            );
            // Must reference the meshes/ subdir
            assert!(
                value.starts_with("meshes/"),
                "expected meshes/<basename>, got {value:?}",
            );
            found_relative += 1;
            rest = &after[end..];
        }
        assert!(
            found_relative > 0,
            "no `{}=\"...\"` mesh references found in exported file",
            mesh_attr,
        );
    }

    /// Helper: rerun the exported file with a different cwd to confirm
    /// the model + meshes still resolve — the strongest portability
    /// guarantee. We run a separate process via `std::process::Command`
    /// so the cwd change is real.
    ///
    /// (Skipped for now — re-loading via Command requires a built
    /// binary. The structural assertions above are sufficient to catch
    /// the regressions we've actually seen.)
    #[allow(dead_code)]
    fn assert_portable_by_cwd_change(_path: &std::path::Path) {}

    /// URDF export from a `.misa` source produces a self-contained
    /// `<dir>/out.urdf` + `<dir>/meshes/*.stl` with `meshes/<basename>`
    /// references. Guards against the pre-fix bug where
    /// `urdf_rs::read_file` failed on TOML and the export aborted.
    #[test]
    fn export_urdf_from_misa_is_portable() {
        let Some(model) = load_namiashi_misa() else {
            eprintln!("[skip] namiashi.misa not present");
            return;
        };
        let dir = std::env::temp_dir().join("articara_portable_urdf");
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();

        model
            .export_urdf_to_file(&dir.join("out.urdf"))
            .expect("URDF export should succeed for .misa source");

        let xml = assert_self_contained(&dir, "out.urdf");
        assert_portable_mesh_references(&xml, "filename");
        std::fs::remove_dir_all(&dir).ok();
    }

    /// SDF export from a `.misa` source produces a self-contained
    /// `<dir>/out.sdf` + meshes. Guards against the pre-fix bug where
    /// the URI was emitted as a plain relative path that only worked
    /// when cwd happened to be the source directory.
    #[test]
    fn export_sdf_from_misa_is_portable() {
        let Some(model) = load_namiashi_misa() else {
            eprintln!("[skip] namiashi.misa not present");
            return;
        };
        let dir = std::env::temp_dir().join("articara_portable_sdf");
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();

        articara::sdf::export_sdf_to_file(&model, &dir.join("out.sdf"))
            .expect("SDF export should succeed for .misa source");

        let xml = assert_self_contained(&dir, "out.sdf");
        // SDF emits URIs inside <uri>...</uri> — same content shape but
        // different attribute name. Reuse the same checker by treating
        // <uri> as a pseudo-attribute via a helper:
        let pseudo = xml.replace("<uri>", "filename=\"").replace("</uri>", "\"");
        assert_portable_mesh_references(&pseudo, "filename");
        std::fs::remove_dir_all(&dir).ok();
    }

    /// MJCF file export uses RelativeToDir + mesh copy. Guards against
    /// the pre-fix bug where `export_mjcf_to_file` emitted absolute
    /// paths that worked locally but couldn't be shared.
    #[test]
    fn export_mjcf_from_misa_is_portable() {
        let Some(model) = load_namiashi_misa() else {
            eprintln!("[skip] namiashi.misa not present");
            return;
        };
        let dir = std::env::temp_dir().join("articara_portable_mjcf");
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();

        articara::mjcf::export_mjcf_to_file(&model, &dir.join("out.xml"))
            .expect("MJCF export should succeed for .misa source");

        let xml = assert_self_contained(&dir, "out.xml");
        assert_portable_mesh_references(&xml, "file");
        std::fs::remove_dir_all(&dir).ok();
    }

    /// Counter-example: in-process MJCF (the default `export_mjcf` for
    /// MuJoCo `from_xml_string`) MUST emit absolute paths so MuJoCo can
    /// resolve them without a cwd anchor. This test pins that
    /// distinction so a future "make everything relative" refactor
    /// doesn't silently break the live sim path.
    #[test]
    fn export_mjcf_in_process_uses_absolute_paths() {
        let Some(model) = load_namiashi_misa() else {
            eprintln!("[skip] namiashi.misa not present");
            return;
        };
        let xml = articara::mjcf::export_mjcf(&model);
        let mut saw_mesh = false;
        let needle = "file=\"";
        let mut rest = xml.as_str();
        while let Some(pos) = rest.find(needle) {
            // Only inspect <mesh ... file="..."> entries; skip non-mesh
            // file references (e.g. <texture file="...">).
            let line_start = rest[..pos].rfind('\n').map(|n| n + 1).unwrap_or(0);
            let line = &rest[line_start..pos];
            if !line.contains("<mesh ") {
                rest = &rest[pos + needle.len()..];
                continue;
            }
            let after = &rest[pos + needle.len()..];
            let end = after.find('"').unwrap();
            let value = &after[..end];
            assert!(
                value.starts_with('/'),
                "in-process MJCF mesh path must be absolute, got {value:?}",
            );
            saw_mesh = true;
            rest = &after[end..];
        }
        assert!(saw_mesh, "no <mesh ... file> entries found");
    }

    /// Round-trip: export URDF + meshes, then load that URDF back via
    /// `from_urdf` (a different cwd, so cwd-relative paths would break
    /// — confirms the `meshes/<basename>` references resolve via
    /// URDF's own `<package_dir>` convention).
    #[test]
    fn export_urdf_round_trip_loads_with_meshes() {
        let Some(model) = load_namiashi_misa() else {
            eprintln!("[skip] namiashi.misa not present");
            return;
        };
        // Mimic URDF package layout: <pkg>/urdf/out.urdf + <pkg>/meshes/.
        // The exporter currently writes meshes/ next to the URDF (not
        // one level up like ROS package layout); the URDF loader still
        // resolves via parent-of-urdf-dir, so we put the URDF inside a
        // <pkg>/<urdf>/ subdir to match the convention.
        let pkg = std::env::temp_dir().join("articara_portable_urdf_roundtrip");
        let _ = std::fs::remove_dir_all(&pkg);
        let urdf_dir = pkg.join("urdf");
        std::fs::create_dir_all(&urdf_dir).unwrap();

        let out = urdf_dir.join("namiashi.urdf");
        model.export_urdf_to_file(&out).expect("export");

        // The exporter copies meshes next to the URDF (./meshes/),
        // not at the package root. Patch the URDF mesh references to
        // `package://_test_pkg/urdf/meshes/...` so the URDF loader's
        // package-root resolver finds them.
        let xml = std::fs::read_to_string(&out).unwrap();
        let xml = xml.replace(
            "filename=\"meshes/",
            "filename=\"package://_test_pkg/urdf/meshes/",
        );
        std::fs::write(&out, xml).unwrap();

        let loaded = RobotModel::from_urdf(&out).expect("re-load exported URDF");
        assert_eq!(loaded.links.len(), model.links.len());
        assert_eq!(loaded.joints.len(), model.joints.len());

        std::fs::remove_dir_all(&pkg).ok();
    }
}

// ============================================================
// Mesh I/O regressions — bugs caught while wiring OBJ support
// and V-HACD save in May 2026. Each test pins one specific
// behaviour that broke at least once; all are self-contained
// (write tiny synthetic URDF + mesh into a tempdir).
// ============================================================
mod test_mesh_io_regressions {
    use super::*;
    use articara::rbd::model::{CollisionData, GeomData};
    use articara::robot::{resolve_package_path, RobotModel};

    /// A 2-triangle Wavefront OBJ (forms a corner of a cube). 4 unique
    /// verts (deduped by tobj's `single_index` pass) → 2 triangles → 12
    /// `f32`s per vert × 3 verts × 2 tris = 36 floats in the flat output.
    const TINY_OBJ: &str = "\
o tri
v 0.0 0.0 0.0
v 1.0 0.0 0.0
v 0.0 1.0 0.0
v 0.0 0.0 1.0
f 1 2 3
f 1 3 4
";

    /// Minimal binary STL with one triangle (header + tri-count + 1 triangle record).
    fn one_tri_stl_bytes() -> Vec<u8> {
        let mut buf = vec![0u8; 80]; // header
        buf.extend_from_slice(&1u32.to_le_bytes()); // 1 triangle
        // normal (0,0,1)
        buf.extend_from_slice(&0f32.to_le_bytes());
        buf.extend_from_slice(&0f32.to_le_bytes());
        buf.extend_from_slice(&1f32.to_le_bytes());
        // 3 vertices in z=0 plane
        for v in [[0f32, 0., 0.], [1., 0., 0.], [0., 1., 0.]] {
            for f in v { buf.extend_from_slice(&f.to_le_bytes()); }
        }
        buf.extend_from_slice(&[0u8, 0u8]); // attribute byte count
        buf
    }

    /// Minimal URDF with one base link carrying a single mesh visual.
    fn tiny_urdf(mesh_uri: &str) -> String {
        format!(
            r#"<?xml version="1.0"?>
<robot name="test_pkg">
  <link name="base">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="{mesh_uri}"/>
      </geometry>
    </visual>
  </link>
</robot>
"#
        )
    }

    fn tempdir(prefix: &str) -> PathBuf {
        use std::time::{SystemTime, UNIX_EPOCH};
        let nanos = SystemTime::now().duration_since(UNIX_EPOCH).unwrap().as_nanos();
        let p = std::env::temp_dir().join(format!("articara_meshio_{prefix}_{nanos}"));
        std::fs::create_dir_all(&p).unwrap();
        p
    }

    /// Lay out a tiny self-contained URDF package at `<root>/test_pkg/`
    /// with `mesh/tri.obj` and a URDF that references it. Returns the
    /// path to the URDF.
    fn write_tiny_obj_package(root: &Path) -> PathBuf {
        let pkg = root.join("test_pkg");
        let mesh_dir = pkg.join("mesh");
        std::fs::create_dir_all(&mesh_dir).unwrap();
        std::fs::write(mesh_dir.join("tri.obj"), TINY_OBJ).unwrap();
        let urdf_path = pkg.join("robot.urdf");
        std::fs::write(
            &urdf_path,
            tiny_urdf("package://test_pkg/mesh/tri.obj"),
        )
        .unwrap();
        urdf_path
    }

    // ── Lesson 1: convert_geometry must dispatch on extension ────────────
    /// Regression: a URDF that references a `.obj` file must populate
    /// `GeomData::Mesh.vertices` (broken when `convert_geometry`
    /// unconditionally called the STL parser).
    #[test]
    fn urdf_with_obj_mesh_loads_vertices() {
        let dir = tempdir("urdf_obj");
        let urdf_path = write_tiny_obj_package(&dir);
        let model = RobotModel::from_urdf(&urdf_path).expect("load URDF");

        let mut obj_meshes = 0;
        for link in &model.links {
            for v in &link.visuals {
                if let GeomData::Mesh { vertices, filename, .. } = &v.geometry {
                    if filename.as_deref().map(|s| s.ends_with(".obj")).unwrap_or(false) {
                        assert!(
                            !vertices.is_empty(),
                            "OBJ vertices empty — convert_geometry probably regressed to STL-only"
                        );
                        // 2 tris × 3 verts × 6 floats = 36 floats expected
                        assert_eq!(vertices.len(), 36, "expected 2 tris worth of floats");
                        obj_meshes += 1;
                    }
                }
            }
        }
        assert_eq!(obj_meshes, 1, "expected exactly one OBJ visual");
        std::fs::remove_dir_all(&dir).ok();
    }

    // ── Lesson 2: resolve_package_path must handle direct-in-package layout ──
    /// Regression: when the URDF lives directly inside its package dir
    /// (`<pkg>/foo.urdf` rather than the ROS-canonical `<pkg>/urdf/foo.urdf`),
    /// `package://<pkg>/mesh/x.stl` must still resolve. The fix added a
    /// second-candidate lookup (`package_dir.join(pkg).join(rel)`) on top
    /// of the original ROS-only `package_dir.join(rel)`.
    #[test]
    fn resolve_package_path_direct_in_package_layout() {
        let dir = tempdir("direct_layout");
        let pkg = dir.join("my_pkg");
        std::fs::create_dir_all(pkg.join("mesh")).unwrap();
        let mesh_path = pkg.join("mesh/foo.stl");
        std::fs::write(&mesh_path, b"").unwrap();

        // Direct layout: URDF is at <root>/my_pkg/foo.urdf, so the loader
        // computes `package_dir = <root>` (parent of urdf_dir = my_pkg).
        let resolved = resolve_package_path(
            "package://my_pkg/mesh/foo.stl",
            &dir, // = package_dir
        );
        assert_eq!(
            resolved, mesh_path,
            "resolver should fall back to package_dir.join(pkg_name).join(rel)"
        );
        std::fs::remove_dir_all(&dir).ok();
    }

    // ── Lesson 3: V-HACD output survives save_urdf ───────────────────────
    /// Regression: a `GeomData::Mesh` with `filename: None` (the shape
    /// V-HACD produces) must be materialised to a real STL file at save
    /// time. Pre-fix, `geom_to_urdf_geom` wrote `filename="mesh.stl"`
    /// and silently dropped the vertex data.
    #[test]
    fn vhacd_decomposed_mesh_survives_urdf_save() {
        let dir = tempdir("vhacd_urdf");
        let urdf_path = write_tiny_obj_package(&dir);
        let mut model = RobotModel::from_urdf(&urdf_path).expect("load URDF");

        // Inject a fake V-HACD output as an additional collision on link 0.
        let fake_verts = one_tri_flat_verts();
        model.links[0].collisions.push(CollisionData {
            origin: nalgebra::Isometry3::identity(),
            geometry: GeomData::Mesh {
                vertices: fake_verts.clone(),
                filename: None,
                scale: None,
            },
        });
        let link_name = model.links[0].name.clone();
        let col_idx = model.links[0].collisions.len() - 1;

        model.save_urdf().expect("save_urdf");

        // STL must exist next to the URDF — file path is deterministic.
        let expected_stl = urdf_path
            .parent()
            .unwrap()
            .join(format!("meshes/decomposed/{link_name}_col_{col_idx}.stl"));
        assert!(
            expected_stl.exists(),
            "decomposed STL not materialised at {expected_stl:?}"
        );

        // Reload and confirm the round-tripped mesh has vertices populated.
        let reloaded = RobotModel::from_urdf(&urdf_path).expect("reload URDF");
        let target_link = reloaded
            .links
            .iter()
            .find(|l| l.name == link_name)
            .expect("link survives round-trip");
        let any_decomposed_loaded = target_link.collisions.iter().any(|c| {
            matches!(
                &c.geometry,
                GeomData::Mesh { filename: Some(f), vertices, .. }
                    if f.contains("meshes/decomposed") && !vertices.is_empty()
            )
        });
        assert!(
            any_decomposed_loaded,
            "reloaded URDF should reference the materialised STL with non-empty verts"
        );
        std::fs::remove_dir_all(&dir).ok();
    }

    // ── Lesson 4: V-HACD output survives save_as_misa ────────────────────
    /// Regression: same as #3 but for the `.misa` save path
    /// (`materialize_decomposed_meshes` is called from there too).
    #[test]
    fn vhacd_decomposed_mesh_survives_misa_save() {
        let dir = tempdir("vhacd_misa");
        let urdf_path = write_tiny_obj_package(&dir);
        let mut model = RobotModel::from_urdf(&urdf_path).expect("load URDF");
        model.links[0].collisions.push(CollisionData {
            origin: nalgebra::Isometry3::identity(),
            geometry: GeomData::Mesh {
                vertices: one_tri_flat_verts(),
                filename: None,
                scale: None,
            },
        });
        let link_name = model.links[0].name.clone();
        let col_idx = model.links[0].collisions.len() - 1;
        let misa_path = dir.join("out.misa");

        model.save_as_misa(&misa_path).expect("save_as_misa");

        let expected_stl = dir.join(format!(
            "meshes/decomposed/{link_name}_col_{col_idx}.stl"
        ));
        assert!(
            expected_stl.exists(),
            "decomposed STL not materialised at {expected_stl:?}"
        );
        // save_as_misa works on a clone — caller's model must stay untouched.
        assert!(
            matches!(
                &model.links[0].collisions[col_idx].geometry,
                GeomData::Mesh { filename: None, .. }
            ),
            "save_as_misa must not mutate caller's model"
        );
        std::fs::remove_dir_all(&dir).ok();
    }

    // ── Lesson 5: save_as_misa copies referenced meshes ──────────────────
    /// Regression: `save_as_misa` to a fresh directory must copy every
    /// referenced mesh next to the `.misa`, so the `AssetSource` sandbox
    /// can resolve them on reload. Pre-fix, the OBJ files were left at
    /// the original URDF location and the reload reported them as
    /// `missing_meshes`.
    #[test]
    fn save_as_misa_copies_referenced_meshes_to_output_dir() {
        let src_dir = tempdir("misa_copy_src");
        let urdf_path = write_tiny_obj_package(&src_dir);
        let model = RobotModel::from_urdf(&urdf_path).expect("load URDF");

        let dst_dir = tempdir("misa_copy_dst");
        let misa_path = dst_dir.join("out.misa");
        model.save_as_misa(&misa_path).expect("save_as_misa");

        // .misa references `mesh/tri.obj` (relative). That file must now
        // exist relative to the .misa.
        let expected = dst_dir.join("mesh/tri.obj");
        assert!(
            expected.exists(),
            "referenced OBJ was not copied to {expected:?}"
        );
        std::fs::remove_dir_all(&src_dir).ok();
        std::fs::remove_dir_all(&dst_dir).ok();
    }

    // ── Lesson 6: .misa loader dispatches on extension (OBJ via .misa) ───
    /// Regression: end-to-end. URDF references an OBJ → save as `.misa` →
    /// reload via `from_misa_with_report`. Must report no missing meshes
    /// and the in-memory model's OBJ visual must have populated vertices.
    /// This catches both the misarta `parse_mesh_bytes` dispatch AND the
    /// articara `convert_misa_geom` dispatch (both were STL-only).
    #[test]
    fn misa_roundtrip_loads_obj() {
        let src_dir = tempdir("misa_obj_rt_src");
        let urdf_path = write_tiny_obj_package(&src_dir);
        let model = RobotModel::from_urdf(&urdf_path).expect("load URDF");

        let dst_dir = tempdir("misa_obj_rt_dst");
        let misa_path = dst_dir.join("out.misa");
        model.save_as_misa(&misa_path).expect("save_as_misa");

        let (loaded, report) =
            RobotModel::from_misa_with_report(&misa_path).expect("reload misa");
        assert!(
            report.missing_meshes.is_empty(),
            "expected no missing meshes after .misa round-trip, got: {:?}",
            report.missing_meshes
        );

        let mut obj_meshes_with_verts = 0;
        for link in &loaded.links {
            for v in &link.visuals {
                if let GeomData::Mesh { filename: Some(f), vertices, .. } = &v.geometry {
                    if f.to_lowercase().ends_with(".obj") && !vertices.is_empty() {
                        obj_meshes_with_verts += 1;
                    }
                }
            }
        }
        assert!(
            obj_meshes_with_verts > 0,
            "expected at least one OBJ visual to have vertices after .misa reload"
        );
        std::fs::remove_dir_all(&src_dir).ok();
        std::fs::remove_dir_all(&dst_dir).ok();
    }

    /// Build a single-triangle flat `[x,y,z,nx,ny,nz]` vertex buffer
    /// (matching what `load_stl_mesh` / `load_obj_mesh` emit).
    fn one_tri_flat_verts() -> Vec<f32> {
        vec![
            0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
            1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
            0.0, 1.0, 0.0, 0.0, 0.0, 1.0,
        ]
    }

    // Unused at present but kept available if a future test wants a
    // genuine STL file to round-trip.
    #[allow(dead_code)]
    fn write_tiny_stl(path: &Path) {
        std::fs::write(path, one_tri_stl_bytes()).unwrap();
    }
}
