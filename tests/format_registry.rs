//! Format detection / registry dispatch, cross-format round trips and serde smoke tests (split from regression.rs).
//!
//! Shared fixture paths live in `common::fixtures`.

mod common;

#[allow(unused_imports)]
use common::fixtures::*;
#[allow(unused_imports)]
use std::path::{Path, PathBuf};

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
    fn detect_gltf_extensions() {
        for name in ["robot.glb", "robot.gltf", "ROBOT.GLB"] {
            let fmt = RobotFormat::detect_from_extension(Path::new(name));
            assert_eq!(fmt, Some(RobotFormat::Gltf), "{name}");
        }
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
        assert!(RobotFormat::Gltf.supports_import());
    }

    #[test]
    fn supports_export() {
        for &fmt in RobotFormat::ALL {
            assert!(fmt.supports_export());
        }
    }

    #[test]
    fn all_contains_six() {
        // Misa (native) + URDF + SDF + MJCF + USD + glTF
        assert_eq!(RobotFormat::ALL.len(), 6);
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

    /// Load URDF, export to GLB through the registry, re-import → the
    /// kinematics and the posed link transforms survive unchanged.
    #[test]
    fn urdf_to_glb_roundtrip() {
        let mut model = RobotModel::from_urdf(&fixture_urdf()).unwrap();
        for (q, j) in model.joint_positions.iter_mut().zip(&model.joints) {
            if j.joint_type != "fixed" {
                *q = 0.3;
            }
        }
        let registry = articara::format::FormatRegistry::default_registry();
        let tmp = std::env::temp_dir().join("articara_urdf2glb.glb");
        registry.export(&model, &tmp).expect("GLB export failed");
        let model2 = registry.import(&tmp).expect("GLB re-import failed");
        std::fs::remove_file(&tmp).ok();

        assert_eq!(model.root_link, model2.root_link);
        assert_eq!(model.links.len(), model2.links.len());
        assert_eq!(model.joints.len(), model2.joints.len());
        for a in &model.joints {
            let b = model2.joints.iter().find(|j| j.name == a.name).unwrap();
            assert_eq!(a.joint_type, b.joint_type);
            assert_eq!((a.parent_link.as_str(), a.child_link.as_str()), (b.parent_link.as_str(), b.child_link.as_str()));
            assert!((a.lower - b.lower).abs() < 1e-9 && (a.upper - b.upper).abs() < 1e-9);
        }
        let (tf1, tf2) = (model.compute_transforms(), model2.compute_transforms());
        for (name, t1) in &tf1 {
            let t2 = tf2[name];
            assert!((t1.translation.vector - t2.translation.vector).norm() < 1e-5, "{name}");
            assert!(t1.rotation.angle_to(&t2.rotation) < 1e-5, "{name}");
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
// serde — JSON serialisation round-trip tests
// ============================================================
#[cfg(feature = "serde")]
mod test_serde {
    use super::*;
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

}
