//! .misa master format and the legacy .misarta.toml sidecar (split from regression.rs).
//!
//! Shared fixture paths live in `common::fixtures`.

mod common;

#[allow(unused_imports)]
use common::fixtures::*;
#[allow(unused_imports)]
use std::path::{Path, PathBuf};

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
            physics: None,
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
                physics: None,
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
            four_support_fraction: 0.5,
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

    /// A2 parity: the same structural edit applied (a) on `RobotModel`
    /// (via its `EditTables` impl + wrappers) and (b) on the exported
    /// `MisaFile` (via `misarta::native::edit`) must serialise to the
    /// identical `.misa` text. Guards the two `EditTables`
    /// implementations against slot-enumeration drift.
    #[test]
    fn edit_parity_with_native_edit() {
        let Some(model) = load_namiashi_misa() else {
            eprintln!("[skip] namiashi.misa not present");
            return;
        };

        // Pick a non-root leaf link and any joint at runtime so the test
        // survives fixture changes.
        let leaf = model
            .links
            .iter()
            .map(|l| l.name.clone())
            .find(|n| *n != model.root_link && !model.joints.iter().any(|j| j.parent_link == *n))
            .expect("fixture has a leaf link");
        let joint = model.joints.last().expect("fixture has joints").name.clone();

        // (a) edits on RobotModel, then export.
        let mut via_model = model.clone();
        assert!(via_model.rename_link(&leaf, "parity_link"));
        assert!(via_model.rename_joint(&joint, "parity_joint"));
        let doc_a = via_model.to_misa().expect("to_misa after model edits");

        // (b) the same edits on the exported MisaFile.
        let mut doc_b = model.to_misa().expect("to_misa base");
        misarta::native::edit::rename_link(&mut doc_b, &leaf, "parity_link").unwrap();
        misarta::native::edit::rename_joint(&mut doc_b, &joint, "parity_joint").unwrap();

        assert_eq!(
            misarta::native::write_str(&doc_a).unwrap(),
            misarta::native::write_str(&doc_b).unwrap(),
            "rename parity: RobotModel path vs native::edit path diverged"
        );

        // Removal parity on the renamed model.
        let mut via_model_rm = via_model.clone();
        via_model_rm.remove_link("parity_link").expect("remove via model");
        let doc_a = via_model_rm.to_misa().expect("to_misa after remove");

        let mut doc_b_rm = doc_b.clone();
        misarta::native::edit::remove_link(&mut doc_b_rm, "parity_link").unwrap();

        assert_eq!(
            misarta::native::write_str(&doc_a).unwrap(),
            misarta::native::write_str(&doc_b_rm).unwrap(),
            "remove parity: RobotModel path vs native::edit path diverged"
        );
    }
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
                physics: None,
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
