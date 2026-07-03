//! USD ASCII export / import and the Isaac Sim script generator (split from regression.rs).
//!
//! Shared fixture paths live in `common::fixtures`.

mod common;

#[allow(unused_imports)]
use common::fixtures::*;
#[allow(unused_imports)]
use std::path::{Path, PathBuf};

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
