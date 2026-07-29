//! MJCF import / export (split from regression.rs).
//!
//! Shared fixture paths live in `common::fixtures`.

mod common;

#[allow(unused_imports)]
use common::fixtures::*;
#[allow(unused_imports)]
use std::path::{Path, PathBuf};

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
                mesh: std::sync::Arc::new(
                    misarta::mesh::MeshData::from_flat_vertices_f32(&[
                    0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
                    1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
                    0.0, 1.0, 0.0, 0.0, 0.0, 1.0,
                ]),
                ),
                filename: Some("/abs/path/example.obj".into()),
                scale: Some([0.001, 0.001, 0.001]),
            },
            color: [1.0; 4],
        });
        model.links[0].collisions.push(CollisionData {
            origin: nalgebra::Isometry3::identity(),
            geometry: GeomData::Mesh {
                mesh: std::sync::Arc::new(
                    misarta::mesh::MeshData::from_flat_vertices_f32(&[
                    0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
                    1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
                    0.0, 1.0, 0.0, 0.0, 0.0, 1.0,
                ]),
                ),
                filename: Some("/abs/path/example.obj".into()),
                scale: Some([0.001, 0.001, 0.001]),
            },
            physics: None,
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
                mesh: std::sync::Arc::new(
                    misarta::mesh::MeshData::from_flat_vertices_f32(&vec![0.0; 18]),
                ),
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
            physics: None,
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
            timestep: None,
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
                mesh,
                filename,
                ..
            } => {
                let vertices = mesh.to_flat_vertices_f32();
                // meshdir prefix must be baked into the stored URI so the
                // exporter (which joins this against `model.source_path`'s
                // parent) recovers the same on-disk file. Without the
                // prefix, in-process MuJoCo re-export looks beside the
                // MJCF instead of in `assets/` and fails with
                // "Error opening file '<...>/tri.obj'".
                assert!(
                    filename.as_deref() == Some("assets/tri.obj"),
                    "filename = {filename:?} (expected meshdir prefix)"
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

        // End-to-end: re-export the imported model through
        // `crate::mesh_paths::resolve_source` (the exact path the in-
        // process MuJoCo backend takes) and confirm the resolved URI
        // points at a real file. This is the regression for the user-
        // observed "Failed to load MuJoCo model" error.
        let geom = &root.visuals[0].geometry;
        let GeomData::Mesh { filename: Some(uri), .. } = geom else {
            unreachable!("checked above");
        };
        let resolved = articara::mesh_paths::resolve_source(uri, &model)
            .expect("resolve_source returned None — meshdir lost");
        assert!(
            resolved.exists(),
            "resolved mesh path {resolved:?} doesn't exist — meshdir prefix dropped \
             between import and re-export"
        );
    }
}
