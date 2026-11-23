//! glTF import of files written by real exporters (fixtures under
//! `tests/fixtures/gltf/`).

use std::path::PathBuf;

use articara::robot::RobotModel;

fn fixture(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/gltf").join(name)
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
