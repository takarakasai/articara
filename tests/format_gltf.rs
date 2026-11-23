//! glTF import of files written by real exporters (fixtures under
//! `tests/fixtures/gltf/`).

use std::path::PathBuf;

use articara::robot::{GeomData, RobotModel};

fn fixture(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/gltf").join(name)
}

/// `blender_arm.glb`: built in Blender 5.3 with `tools/blender/articara_joints.py`
/// and exported by its "Export for articara" button — base (mass 3 kg,
/// collision) → `shoulder_pitch` revolute about the arm's local Y
/// (−30°…120°) → `slider` prismatic along local −Z (0…0.1 m).
#[test]
fn blender_addon_export_imports_with_joints() {
    let model = RobotModel::from_file(&fixture("blender_arm.glb")).unwrap();

    let shoulder = &model.joints[model.joint_map["shoulder_pitch"]];
    assert_eq!(shoulder.joint_type, "revolute");
    assert_eq!((shoulder.parent_link.as_str(), shoulder.child_link.as_str()), ("base", "upper_arm"));
    assert!((shoulder.axis - nalgebra::Vector3::y()).norm() < 1e-6, "{:?}", shoulder.axis);
    assert!((shoulder.lower - (-30f64).to_radians()).abs() < 1e-9);
    assert!((shoulder.upper - 120f64.to_radians()).abs() < 1e-9);

    let slider = &model.joints[model.joint_map["slider_joint"]];
    assert_eq!(slider.joint_type, "prismatic");
    assert!((slider.axis + nalgebra::Vector3::z()).norm() < 1e-6, "{:?}", slider.axis);
    assert_eq!((slider.lower, slider.upper), (0.0, 0.1));

    // Blender placed the arm at z = 0.35 and the slider at z = 0.55.
    let tf = model.compute_transforms();
    assert!((tf["upper_arm"].translation.z - 0.35).abs() < 1e-5);
    assert!((tf["slider"].translation.z - 0.55).abs() < 1e-5);

    let base = &model.links[model.link_map["base"]];
    assert_eq!(base.inertial.mass, 3.0);
    assert!(matches!(base.collisions.as_slice(), [c] if matches!(c.geometry, GeomData::Mesh { .. })));
    assert!(model.links[model.link_map["upper_arm"]].collisions.is_empty());
}

/// `blender_arm_animated.glb`: a `shoulder` revolute joint (local Y)
/// keyframed in Blender 5.3 at 0° (frame 1), 60° (frame 25) and −20°
/// (frame 49), which Blender's exporter bakes to one key per frame at
/// t = frame / 24 s.
#[test]
fn blender_animation_imports_as_sequence() {
    let model = RobotModel::from_file(&fixture("blender_arm_animated.glb")).unwrap();

    let seq = &model.sequences[0];
    assert_eq!(seq.name, "upper_armAction");
    assert_eq!(seq.steps.len(), 49);
    let shoulder_deg = |k: usize| {
        let pose = model.poses.iter().find(|p| p.name == seq.steps[k].pose_name).unwrap();
        pose.angles["shoulder"].to_degrees()
    };
    assert!(shoulder_deg(0).abs() < 1e-3, "{}", shoulder_deg(0));
    assert!((shoulder_deg(24) - 60.0).abs() < 1e-3, "{}", shoulder_deg(24));
    assert!((shoulder_deg(48) + 20.0).abs() < 1e-3, "{}", shoulder_deg(48));
    let total: f64 = seq.steps.iter().map(|s| s.duration).sum();
    assert!((total - 49.0 / 24.0).abs() < 1e-5, "{total}");
}
