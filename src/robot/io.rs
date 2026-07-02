//! Format I/O for [`RobotModel`]: URDF import / export, `.misa` load /
//! save (including the sidecar-era helpers), mesh loading and
//! materialisation of in-memory decomposed meshes.
//!
//! Structural editing lives in [`super::edit`]; ray picking in
//! [`super::pick`].

use nalgebra as na;
use std::collections::{HashMap, HashSet};
use std::path::{Path, PathBuf};

use crate::rbd::model::*;

// ========== URDF Loading ==========

impl RobotModel {
    pub fn from_urdf(path: &Path) -> Result<Self, String> {
        let robot = urdf_rs::read_file(path).map_err(|e| format!("URDF parse error: {e}"))?;

        let urdf_dir = path.parent().unwrap_or(Path::new("."));
        let package_dir = urdf_dir.parent().unwrap_or(urdf_dir);

        // Materials
        let mut materials: HashMap<String, [f32; 4]> = HashMap::new();
        for mat in &robot.materials {
            if let Some(ref color) = mat.color {
                materials.insert(
                    mat.name.clone(),
                    [
                        color.rgba.0[0] as f32,
                        color.rgba.0[1] as f32,
                        color.rgba.0[2] as f32,
                        color.rgba.0[3] as f32,
                    ],
                );
            }
        }

        // Links
        let mut links = Vec::new();
        let mut link_map = HashMap::new();
        for (i, link) in robot.links.iter().enumerate() {
            let visuals = link
                .visual
                .iter()
                .map(|vis| {
                    let origin = pose_to_isometry(&vis.origin);
                    let color = vis
                        .material
                        .as_ref()
                        .and_then(|m| {
                            m.color
                                .as_ref()
                                .map(|c| {
                                    [
                                        c.rgba.0[0] as f32,
                                        c.rgba.0[1] as f32,
                                        c.rgba.0[2] as f32,
                                        c.rgba.0[3] as f32,
                                    ]
                                })
                                .or_else(|| materials.get(&m.name).copied())
                        })
                        .unwrap_or([0.8, 0.8, 0.8, 1.0]);
                    let geometry = convert_geometry(&vis.geometry, package_dir);
                    VisualData {
                        origin,
                        geometry,
                        color,
                    }
                })
                .collect();

            let collisions = link
                .collision
                .iter()
                .map(|col| CollisionData {
                    origin: pose_to_isometry(&col.origin),
                    geometry: convert_geometry(&col.geometry, package_dir),
                
                    physics: None,
                })
                .collect();

            let inertial = InertialData {
                origin: pose_to_isometry(&link.inertial.origin),
                mass: link.inertial.mass.value,
                ixx: link.inertial.inertia.ixx,
                ixy: link.inertial.inertia.ixy,
                ixz: link.inertial.inertia.ixz,
                iyy: link.inertial.inertia.iyy,
                iyz: link.inertial.inertia.iyz,
                izz: link.inertial.inertia.izz,
            };

            link_map.insert(link.name.clone(), i);
            links.push(LinkData {
                name: link.name.clone(),
                visuals,
                collisions,
                inertial,
                collision_enabled: true,
            });
        }

        // Joints
        let mut joints = Vec::new();
        let mut joint_map = HashMap::new();
        let mut children_joints: HashMap<String, Vec<usize>> = HashMap::new();
        let mut child_links: HashSet<String> = HashSet::new();
        // Collect URDF <mimic> entries as the master format's `mimics` list.
        let mut mimics: Vec<crate::rbd::model::Mimic> = Vec::new();

        for (i, joint) in robot.joints.iter().enumerate() {
            let jtype = format!("{:?}", joint.joint_type).to_lowercase();
            let origin = pose_to_isometry(&joint.origin);
            let axis = na::Vector3::new(
                joint.axis.xyz.0[0] as f32,
                joint.axis.xyz.0[1] as f32,
                joint.axis.xyz.0[2] as f32,
            );

            joint_map.insert(joint.name.clone(), i);
            children_joints
                .entry(joint.parent.link.clone())
                .or_default()
                .push(i);
            child_links.insert(joint.child.link.clone());

            joints.push(JointData {
                name: joint.name.clone(),
                joint_type: jtype,
                parent_link: joint.parent.link.clone(),
                child_link: joint.child.link.clone(),
                origin,
                axis,
                lower: joint.limit.lower,
                upper: joint.limit.upper,
                effort: joint.limit.effort,
                velocity: joint.limit.velocity,
                actuator_mode: crate::rbd::model::ActuatorMode::default(),
                actuator_kp: 50.0,
                actuator_kv: 5.0,
                // URDF has no native armature field, but most real motors do —
                // a small default (matches `default_armature()`) keeps the PD
                // controller stable at MuJoCo's default 2 ms timestep.
                armature: 0.0014,
                joint_damping: 0.0,
            });

            // Capture <mimic> if present. URDF uses linear coupling; we
            // store it as a master-format Mimic that other exporters can
            // translate into their native form.
            if let Some(ref m) = joint.mimic {
                mimics.push(crate::rbd::model::Mimic {
                    joint: joint.name.clone(),
                    source: m.joint.clone(),
                    multiplier: m.multiplier.unwrap_or(1.0),
                    offset: m.offset.unwrap_or(0.0),
                });
            }
        }

        // Root link = not a child of any joint
        let root_link = links
            .iter()
            .find(|l| !child_links.contains(&l.name))
            .map(|l| l.name.clone())
            .unwrap_or_default();

        let joint_positions = vec![0.0_f64; joints.len()];

        log::info!(
            "Loaded robot '{}': {} links, {} joints, root='{}'",
            robot.name,
            links.len(),
            joints.len(),
            root_link
        );

        let mut model = Self {
            name: robot.name.clone(),
            links,
            joints,
            link_map,
            joint_map,
            root_link,
            children_joints,
            materials,
            joint_positions,
            source_path: Some(path.to_path_buf()),
            base_transform: na::Isometry3::identity(),
            misarta_cache: None,
            loop_closures: Vec::new(),
            poses: Vec::new(),
            collision_pairs: Vec::new(),
            sequences: Vec::new(),
            mimics,
            sensors: Vec::new(),
            gaits: Vec::new(),
        };
        model.rebuild_misarta_model();
        Ok(model)
    }

    /// Load a robot model from any supported format (auto-detected by
    /// extension, with content sniffing for ambiguous `.xml`). Dispatches
    /// through the [`crate::format::FormatRegistry`], so a newly
    /// registered format is picked up here without further wiring.
    pub fn from_file(path: &Path) -> Result<Self, String> {
        crate::format::FormatRegistry::default_registry().import(path)
    }

    /// Load a `.misa` master-format file. Convenience wrapper that
    /// discards the [`misarta::native::LoadReport`]; use
    /// [`Self::from_misa_with_report`] when the GUI needs to surface
    /// sanitisations / missing meshes.
    pub fn from_misa(path: &Path) -> Result<Self, String> {
        let (model, _report) = Self::from_misa_with_report(path)?;
        Ok(model)
    }

    /// Load a `.misa` master-format file along with the load report.
    ///
    /// The report carries identifier sanitisations, material renames,
    /// and unresolved mesh references — surface it in the editor's
    /// post-load dialog so the user can confirm the changes.
    pub fn from_misa_with_report(
        path: &Path,
    ) -> Result<(Self, misarta::native::LoadReport), String> {
        let out = misarta::native::load(path)
            .map_err(|e| format!(".misa load: {e}"))?;
        let report = out.report.clone();
        let model = Self::from_misa_file(&out.file, path)?;
        Ok((model, report))
    }

    /// Convert an already-parsed [`misarta::native::MisaFile`] into a
    /// `RobotModel`. Used internally by [`Self::from_misa`]; exposed so
    /// callers that produced a `MisaFile` in memory (tests, scripted
    /// generators) can skip the parse step.
    ///
    /// `path` is used to resolve relative mesh references and is stored
    /// as `source_path` on the returned model. Pass any path under the
    /// directory mesh files live in.
    pub fn from_misa_file(
        file: &misarta::native::MisaFile,
        path: &Path,
    ) -> Result<Self, String> {
        misa_load::build_robot_model(file, path)
    }

    /// Build a [`misarta::native::MisaFile`] in memory from this
    /// `RobotModel`. The inverse of [`Self::from_misa_file`].
    ///
    /// The resulting file is structurally complete — links, joints,
    /// inertials, visuals, collisions, materials, mimics, loop closures,
    /// collision pairs, sensors, actuators, poses, sequences, gaits, and
    /// home pose are all populated. Mesh references keep whatever path
    /// they had in the source format (URDF `package://…` or already-relative
    /// `meshes/…`); callers that want clean relative paths should run
    /// `normalise_mesh_paths_to_meshes_dir` before serialising.
    ///
    /// Per-joint actuator settings (mode/kp/kv) are emitted as 1:1
    /// `[[actuator]]` entries — one actuator per movable joint with
    /// `joints = [{ name = "<joint>", gear = 1.0 }]`. Multi-joint actuators
    /// (N:M) are not reconstructed because `RobotModel` doesn't carry the
    /// information needed to identify them; callers that need N:M output
    /// must build the `MisaFile` directly and skip this convenience.
    pub fn to_misa(&self) -> Result<misarta::native::MisaFile, String> {
        misa_save::build_misa_file(self)
    }

    /// Convenience wrapper: convert to a `MisaFile` and write it to disk.
    ///
    /// In-memory decomposed meshes (`GeomData::Mesh` with `filename: None`,
    /// produced by V-HACD) are materialised to STL files alongside the
    /// `.misa` so the saved file references real meshes. The materialisation
    /// is done on an internal clone so the caller's model is left untouched.
    pub fn save_as_misa(&self, path: &Path) -> Result<(), String> {
        let misa_dir = path.parent().unwrap_or(Path::new("."));
        let mut working = self.clone();
        materialize_decomposed_meshes(&mut working, misa_dir, |fname| {
            format!("meshes/decomposed/{fname}")
        })?;
        // Copy referenced (pre-existing) mesh files next to the `.misa` so
        // the `AssetSource` sandbox can find them on re-load. Without this
        // step a `.misa` saved into a fresh directory loads with empty mesh
        // visuals (`missing_meshes` in the LoadReport).
        copy_referenced_meshes_to_misa_dir(&working, self.source_path.as_deref(), misa_dir)?;
        let file = working.to_misa()?;
        misarta::native::save(path, &file).map_err(|e| format!(".misa save: {e}"))
    }
}

// ─── .misa → RobotModel conversion (internal) ──────────────────────────────

mod misa_load {
    use super::*;
    use misarta::native as mn;

    pub fn build_robot_model(
        file: &mn::MisaFile,
        path: &Path,
    ) -> Result<RobotModel, String> {
        let base_dir = path.parent().unwrap_or(Path::new("."));

        // ── Materials map: name → RGBA ──────────────────────────────────
        let mut materials: HashMap<String, [f32; 4]> = HashMap::new();
        for m in &file.material {
            materials.insert(m.name.clone(), color_spec_to_rgba(&m.color));
        }

        // ── Links ───────────────────────────────────────────────────────
        let mut links: Vec<LinkData> = Vec::with_capacity(file.link.len());
        let mut link_map: HashMap<String, usize> = HashMap::new();
        for (i, l) in file.link.iter().enumerate() {
            link_map.insert(l.name.clone(), i);

            let visuals: Vec<VisualData> = l
                .visual
                .iter()
                .map(|v| VisualData {
                    origin: misa_origin_to_isometry_f32(&v.origin),
                    geometry: convert_misa_geom(&v.geom, base_dir),
                    color: resolve_visual_color(v, &materials),
                })
                .collect();

            let collisions: Vec<CollisionData> = l
                .collision
                .iter()
                .map(|c| CollisionData {
                    origin: misa_origin_to_isometry_f32(&c.origin),
                    geometry: convert_misa_geom(&c.geom, base_dir),
                    physics: c.physics.as_ref().map(|p| {
                        crate::rbd::model::MjcfPhysics {
                            friction: p.friction,
                            condim: p.condim,
                            priority: p.priority,
                            solimp: p.solimp,
                            margin: p.margin,
                        }
                    }),
                })
                .collect();

            let inertial = InertialData {
                origin: misa_origin_to_isometry_f32(&l.inertial.origin),
                mass: l.inertial.mass,
                ixx: l.inertial.ixx,
                ixy: l.inertial.ixy,
                ixz: l.inertial.ixz,
                iyy: l.inertial.iyy,
                iyz: l.inertial.iyz,
                izz: l.inertial.izz,
            };

            links.push(LinkData {
                name: l.name.clone(),
                visuals,
                collisions,
                inertial,
                collision_enabled: l.collision_enabled,
            });
        }

        // ── Joints ──────────────────────────────────────────────────────
        // Build a per-joint actuator-config lookup. Multi-joint actuators
        // (N:M) are flattened to per-joint settings: each participating
        // joint inherits the actuator's mode/kp/kv. Multi-actuator-per-joint
        // (N:1) is collapsed to "first wins" with a log warning — the
        // current `JointData` schema can only hold one set of gains.
        let mut joint_actuator_settings: HashMap<&str, (mn::ActuatorMode, f64, f64)> =
            HashMap::new();
        for a in &file.actuator {
            for jr in &a.joints {
                if joint_actuator_settings.contains_key(jr.name.as_str()) {
                    log::warn!(
                        "joint '{}' has multiple actuators ('{}' is the additional one); \
                         only the first actuator's gains are kept in RobotModel",
                        jr.name,
                        a.name,
                    );
                    continue;
                }
                joint_actuator_settings.insert(jr.name.as_str(), (a.mode, a.kp, a.kv));
            }
        }

        let mut joints: Vec<JointData> = Vec::with_capacity(file.joint.len());
        let mut joint_map: HashMap<String, usize> = HashMap::new();
        let mut children_joints: HashMap<String, Vec<usize>> = HashMap::new();
        for (i, j) in file.joint.iter().enumerate() {
            joint_map.insert(j.name.clone(), i);
            children_joints
                .entry(j.parent.clone())
                .or_default()
                .push(i);

            let (actuator_mode, actuator_kp, actuator_kv) = joint_actuator_settings
                .get(j.name.as_str())
                .copied()
                .map(|(m, kp, kv)| (convert_actuator_mode(m), kp, kv))
                .unwrap_or((ActuatorMode::default(), 50.0, 5.0));

            joints.push(JointData {
                name: j.name.clone(),
                joint_type: joint_kind_to_string(j.kind),
                parent_link: j.parent.clone(),
                child_link: j.child.clone(),
                origin: misa_origin_to_isometry_f32(&j.origin),
                axis: na::Vector3::new(j.axis[0] as f32, j.axis[1] as f32, j.axis[2] as f32),
                lower: j.limit.lower,
                upper: j.limit.upper,
                effort: j.limit.effort,
                velocity: j.limit.velocity,
                actuator_mode,
                actuator_kp,
                actuator_kv,
                armature: j.dynamics.armature,
                joint_damping: j.dynamics.damping,
            });
        }

        // ── Mimics (direct, same shape) ─────────────────────────────────
        let mimics: Vec<crate::rbd::model::Mimic> = file
            .mimic
            .iter()
            .map(|m| crate::rbd::model::Mimic {
                joint: m.joint.clone(),
                source: m.source.clone(),
                multiplier: m.multiplier,
                offset: m.offset,
            })
            .collect();

        // ── Loop closures (use existing from_config) ────────────────────
        let loop_closures: Vec<crate::rbd::model::LoopClosure> = file
            .loop_closure
            .iter()
            .map(crate::rbd::model::LoopClosure::from_config)
            .collect();

        // ── Collision pairs (use normalising constructor) ───────────────
        let collision_pairs: Vec<crate::rbd::model::CollisionPair> = file
            .collision_pair
            .iter()
            .map(|cp| {
                crate::rbd::model::CollisionPair::new(cp.link_a.clone(), cp.link_b.clone(), cp.enabled)
            })
            .collect();

        // ── Sensors (Origin → Isometry3<f64>) ───────────────────────────
        let sensors: Vec<crate::rbd::model::Sensor> = file
            .sensor
            .iter()
            .map(|s| crate::rbd::model::Sensor {
                name: s.name.clone(),
                link: s.link.clone(),
                origin: misa_origin_to_isometry_f64(&s.origin),
                update_rate: s.update_rate,
                kind: convert_sensor_kind(&s.kind),
            })
            .collect();

        // ── Poses, sequences, gaits ─────────────────────────────────────
        // These are direct re-exports of misarta::config types in the
        // .misa schema, so we go through load_misarta_config to reuse the
        // existing application logic (joint angle filtering, etc.).
        let mut cfg = misarta::config::MisartaConfig::new();
        for p in &file.pose {
            cfg.pose.push(p.clone());
        }
        for s in &file.sequence {
            cfg.sequence.push(s.clone());
        }
        for g in &file.gait {
            cfg.gait.push(g.clone());
        }
        cfg.home = file.home.clone();

        // ── Root link, joint positions ──────────────────────────────────
        let joint_positions = vec![0.0_f64; joints.len()];
        let root_link = file.robot.root.clone();

        log::info!(
            "Loaded .misa robot '{}': {} links, {} joints, root='{}'",
            file.robot.name,
            links.len(),
            joints.len(),
            root_link
        );

        let mut model = RobotModel {
            name: file.robot.name.clone(),
            links,
            joints,
            link_map,
            joint_map,
            root_link,
            children_joints,
            materials,
            joint_positions,
            source_path: Some(path.to_path_buf()),
            base_transform: na::Isometry3::identity(),
            misarta_cache: None,
            loop_closures: Vec::new(),
            poses: Vec::new(),
            collision_pairs: Vec::new(),
            sequences: Vec::new(),
            mimics: Vec::new(),
            sensors: Vec::new(),
            gaits: Vec::new(),
        };
        // Apply the pose / sequence / gait / home subset via the existing
        // sidecar loader (it handles joint_positions for `home` and per-joint
        // actuator gains). load_misarta_config also blanks
        // mimics / loop_closures / collision_pairs / sensors from the cfg
        // contents, so we must populate those AFTER calling it (we passed
        // them empty above to make the order explicit).
        model.load_misarta_config(&cfg);
        model.mimics = mimics;
        model.loop_closures = loop_closures;
        model.collision_pairs = collision_pairs;
        model.sensors = sensors;
        model.rebuild_misarta_model();
        Ok(model)
    }

    // ─── Conversion helpers ──────────────────────────────────────────────

    pub(super) fn misa_origin_to_isometry_f32(o: &mn::Origin) -> na::Isometry3<f32> {
        let t = na::Translation3::new(o.xyz[0] as f32, o.xyz[1] as f32, o.xyz[2] as f32);
        let r = misa_origin_rotation_f32(o);
        na::Isometry3::from_parts(t, r)
    }

    fn misa_origin_to_isometry_f64(o: &mn::Origin) -> na::Isometry3<f64> {
        let t = na::Translation3::new(o.xyz[0], o.xyz[1], o.xyz[2]);
        let r = if let Some(q) = o.quat {
            na::UnitQuaternion::from_quaternion(na::Quaternion::new(q[3], q[0], q[1], q[2]))
        } else if let Some(rpy) = o.rpy {
            na::UnitQuaternion::from_euler_angles(rpy[0], rpy[1], rpy[2])
        } else {
            na::UnitQuaternion::identity()
        };
        na::Isometry3::from_parts(t, r)
    }

    fn misa_origin_rotation_f32(o: &mn::Origin) -> na::UnitQuaternion<f32> {
        if let Some(q) = o.quat {
            na::UnitQuaternion::from_quaternion(na::Quaternion::new(
                q[3] as f32,
                q[0] as f32,
                q[1] as f32,
                q[2] as f32,
            ))
        } else if let Some(rpy) = o.rpy {
            na::UnitQuaternion::from_euler_angles(rpy[0] as f32, rpy[1] as f32, rpy[2] as f32)
        } else {
            na::UnitQuaternion::identity()
        }
    }

    fn convert_misa_geom(geom: &mn::Geom, base_dir: &Path) -> GeomData {
        match geom {
            mn::Geom::Box { size } => GeomData::Box {
                hx: size[0] as f32 / 2.0,
                hy: size[1] as f32 / 2.0,
                hz: size[2] as f32 / 2.0,
            },
            mn::Geom::Cylinder { radius, length } => GeomData::Cylinder {
                radius: *radius as f32,
                half_length: *length as f32 / 2.0,
            },
            mn::Geom::Sphere { radius } => GeomData::Sphere {
                radius: *radius as f32,
            },
            mn::Geom::Capsule { radius, length } => GeomData::Capsule {
                radius: *radius as f32,
                half_length: *length as f32 / 2.0,
            },
            mn::Geom::Mesh { file, scale } => {
                let path = base_dir.join(file);
                let scale_arr = [scale[0] as f32, scale[1] as f32, scale[2] as f32];
                GeomData::Mesh {
                    mesh: super::load_mesh(&path, Some(&scale_arr)),
                    filename: Some(file.clone()),
                    scale: Some(scale_arr),
                }
            }
        }
    }


    pub(super) fn color_spec_to_rgba(c: &mn::ColorSpec) -> [f32; 4] {
        match c {
            mn::ColorSpec::Rgba(v) => *v,
            mn::ColorSpec::Hex(s) => parse_hex_color(s).unwrap_or([0.8, 0.8, 0.8, 1.0]),
        }
    }

    fn parse_hex_color(s: &str) -> Option<[f32; 4]> {
        let s = s.strip_prefix('#').unwrap_or(s);
        let byte = |i: usize| -> Option<f32> {
            let pair = s.get(i..i + 2)?;
            u8::from_str_radix(pair, 16).ok().map(|b| b as f32 / 255.0)
        };
        match s.len() {
            6 => Some([byte(0)?, byte(2)?, byte(4)?, 1.0]),
            8 => Some([byte(0)?, byte(2)?, byte(4)?, byte(6)?]),
            _ => None,
        }
    }

    fn resolve_visual_color(
        v: &mn::Visual,
        materials: &HashMap<String, [f32; 4]>,
    ) -> [f32; 4] {
        if let Some(c) = &v.color {
            return color_spec_to_rgba(c);
        }
        if let Some(name) = &v.material {
            if let Some(c) = materials.get(name) {
                return *c;
            }
        }
        [0.8, 0.8, 0.8, 1.0]
    }

    fn joint_kind_to_string(k: mn::JointKind) -> String {
        match k {
            mn::JointKind::Revolute => "revolute".into(),
            mn::JointKind::Continuous => "continuous".into(),
            mn::JointKind::Prismatic => "prismatic".into(),
            mn::JointKind::Fixed => "fixed".into(),
            mn::JointKind::Floating => "floating".into(),
            mn::JointKind::Planar => "planar".into(),
        }
    }

    fn convert_actuator_mode(m: mn::ActuatorMode) -> ActuatorMode {
        match m {
            mn::ActuatorMode::Position => ActuatorMode::Position,
            mn::ActuatorMode::Velocity => ActuatorMode::Velocity,
            mn::ActuatorMode::Torque => ActuatorMode::Torque,
            mn::ActuatorMode::ComputedTorque => ActuatorMode::ComputedTorque,
            mn::ActuatorMode::Fixed => ActuatorMode::Fixed,
        }
    }

    fn convert_sensor_kind(k: &mn::SensorKind) -> crate::rbd::model::SensorKind {
        use crate::rbd::model::SensorKind as Out;
        match k {
            mn::SensorKind::Camera { fov, width, height, near, far } => Out::Camera {
                fov: *fov, width: *width, height: *height, near: *near, far: *far,
            },
            mn::SensorKind::Lidar {
                range_min, range_max, h_fov, h_samples, v_fov, v_samples,
            } => Out::Lidar {
                range_min: *range_min,
                range_max: *range_max,
                h_fov: *h_fov,
                h_samples: *h_samples,
                v_fov: *v_fov,
                v_samples: *v_samples,
            },
            mn::SensorKind::Imu { gyro_noise, accel_noise } => Out::Imu {
                gyro_noise: *gyro_noise,
                accel_noise: *accel_noise,
            },
            mn::SensorKind::ForceTorque { joint } => Out::ForceTorque {
                joint: joint.clone(),
            },
            mn::SensorKind::Contact { partner } => Out::Contact {
                partner: partner.clone(),
            },
            mn::SensorKind::Generic { kind, params } => Out::Generic {
                kind: kind.clone(),
                params: params.clone(),
            },
        }
    }
}

// compute_transforms, parent_joint_of_link, ancestor_links, link_bounding_sphere
// are now defined in crate::rbd::model (re-exported via pub use above).


// ========== Helper Functions ==========

/// Convert an Isometry3 back to a urdf_rs Pose (xyz + rpy).
pub fn isometry_to_pose(iso: &na::Isometry3<f32>) -> urdf_rs::Pose {
    let t = iso.translation;
    let (roll, pitch, yaw) = iso.rotation.euler_angles();
    urdf_rs::Pose {
        xyz: urdf_rs::Vec3([t.x as f64, t.y as f64, t.z as f64]),
        rpy: urdf_rs::Vec3([roll as f64, pitch as f64, yaw as f64]),
    }
}

/// Extract Euler angles (roll, pitch, yaw) from an isometry.
fn euler_from_isometry(iso: &na::Isometry3<f32>) -> (f32, f32, f32) {
    iso.rotation.euler_angles()
}

/// Convert a `GeomData` to a `urdf_rs::Geometry`.
fn geom_to_urdf_geom(geom: &GeomData) -> urdf_rs::Geometry {
    match geom {
        GeomData::Box { hx, hy, hz } => urdf_rs::Geometry::Box {
            size: urdf_rs::Vec3([*hx as f64 * 2.0, *hy as f64 * 2.0, *hz as f64 * 2.0]),
        },
        GeomData::Cylinder { radius, half_length } => urdf_rs::Geometry::Cylinder {
            radius: *radius as f64,
            length: *half_length as f64 * 2.0,
        },
        GeomData::Sphere { radius } => urdf_rs::Geometry::Sphere {
            radius: *radius as f64,
        },
        GeomData::Mesh { filename, scale, .. } => urdf_rs::Geometry::Mesh {
            filename: filename.clone().unwrap_or_else(|| "mesh.stl".into()),
            scale: scale.map(|s| urdf_rs::Vec3([s[0] as f64, s[1] as f64, s[2] as f64])),
        },
        // Capsule is not supported by URDF — approximate as cylinder (caps ignored)
        GeomData::Capsule { radius, half_length } => urdf_rs::Geometry::Cylinder {
            radius: *radius as f64,
            length: (*half_length * 2.0 + *radius * 2.0) as f64,
        },
    }
}

/// Convert a `VisualData` to one or more `urdf_rs::Visual` elements.
/// Capsules are decomposed into a cylinder + 2 sphere visuals.
fn visuals_to_urdf(vis: &VisualData) -> Vec<urdf_rs::Visual> {
    let make_visual = |origin_iso: &na::Isometry3<f32>, geom: urdf_rs::Geometry| -> urdf_rs::Visual {
        urdf_rs::Visual {
            name: None,
            origin: isometry_to_pose(origin_iso),
            geometry: geom,
            material: Some(urdf_rs::Material {
                name: String::new(),
                color: Some(urdf_rs::Color {
                    rgba: urdf_rs::Vec4([
                        vis.color[0] as f64,
                        vis.color[1] as f64,
                        vis.color[2] as f64,
                        vis.color[3] as f64,
                    ]),
                }),
                texture: None,
            }),
        }
    };

    match &vis.geometry {
        GeomData::Capsule { radius, half_length } => {
            let r = *radius;
            let hl = *half_length;
            let cyl = make_visual(&vis.origin, urdf_rs::Geometry::Cylinder {
                radius: r as f64,
                length: (hl * 2.0) as f64,
            });
            let top_origin = vis.origin * na::Translation3::new(0.0, 0.0, hl);
            let top = make_visual(&na::Isometry3::from_parts(
                top_origin.translation,
                vis.origin.rotation,
            ), urdf_rs::Geometry::Sphere { radius: r as f64 });
            let bot_origin = vis.origin * na::Translation3::new(0.0, 0.0, -hl);
            let bot = make_visual(&na::Isometry3::from_parts(
                bot_origin.translation,
                vis.origin.rotation,
            ), urdf_rs::Geometry::Sphere { radius: r as f64 });
            vec![cyl, top, bot]
        }
        _ => vec![urdf_rs::Visual {
            name: None,
            origin: isometry_to_pose(&vis.origin),
            geometry: geom_to_urdf_geom(&vis.geometry),
            material: Some(urdf_rs::Material {
                name: String::new(),
                color: Some(urdf_rs::Color {
                    rgba: urdf_rs::Vec4([
                        vis.color[0] as f64,
                        vis.color[1] as f64,
                        vis.color[2] as f64,
                        vis.color[3] as f64,
                    ]),
                }),
                texture: None,
            }),
        }],
    }
}

/// Convert a `CollisionData` to one or more `urdf_rs::Collision` elements.
/// Capsules are decomposed into a cylinder + 2 sphere collisions.
fn collisions_to_urdf(col: &CollisionData) -> Vec<urdf_rs::Collision> {
    match &col.geometry {
        GeomData::Capsule { radius, half_length } => {
            let r = *radius;
            let hl = *half_length;
            let cyl = urdf_rs::Collision {
                name: None,
                origin: isometry_to_pose(&col.origin),
                geometry: urdf_rs::Geometry::Cylinder {
                    radius: r as f64,
                    length: (hl * 2.0) as f64,
                },
            };
            let top_origin = col.origin * na::Translation3::new(0.0, 0.0, hl);
            let top = urdf_rs::Collision {
                name: None,
                origin: isometry_to_pose(&na::Isometry3::from_parts(
                    top_origin.translation,
                    col.origin.rotation,
                )),
                geometry: urdf_rs::Geometry::Sphere { radius: r as f64 },
            };
            let bot_origin = col.origin * na::Translation3::new(0.0, 0.0, -hl);
            let bot = urdf_rs::Collision {
                name: None,
                origin: isometry_to_pose(&na::Isometry3::from_parts(
                    bot_origin.translation,
                    col.origin.rotation,
                )),
                geometry: urdf_rs::Geometry::Sphere { radius: r as f64 },
            };
            vec![cyl, top, bot]
        }
        _ => vec![urdf_rs::Collision {
            name: None,
            origin: isometry_to_pose(&col.origin),
            geometry: geom_to_urdf_geom(&col.geometry),
        }],
    }
}

/// Convert a GeomData to URDF XML geometry element.
fn geom_to_urdf_xml(geom: &GeomData) -> String {
    match geom {
        GeomData::Box { hx, hy, hz } => {
            let sx = hx * 2.0;
            let sy = hy * 2.0;
            let sz = hz * 2.0;
            format!("      <geometry>\n        <box size=\"{sx} {sy} {sz}\"/>\n      </geometry>\n")
        }
        GeomData::Cylinder {
            radius,
            half_length,
        } => {
            let length = half_length * 2.0;
            format!("      <geometry>\n        <cylinder radius=\"{radius}\" length=\"{length}\"/>\n      </geometry>\n")
        }
        GeomData::Sphere { radius } => {
            format!("      <geometry>\n        <sphere radius=\"{radius}\"/>\n      </geometry>\n")
        }
        GeomData::Mesh { filename, scale, .. } => {
            let fname = filename.as_deref().unwrap_or("mesh.stl");
            if let Some(s) = scale {
                format!("      <geometry>\n        <mesh filename=\"{fname}\" scale=\"{} {} {}\"/>\n      </geometry>\n", s[0], s[1], s[2])
            } else {
                format!("      <geometry>\n        <mesh filename=\"{fname}\"/>\n      </geometry>\n")
            }
        }
        GeomData::Capsule { radius, half_length } => {
            // URDF: decompose capsule into cylinder + 2 spheres
            let cyl_len = half_length * 2.0;
            let out = format!("      <geometry>\n        <cylinder radius=\"{radius}\" length=\"{cyl_len}\"/>\n      </geometry>\n");
            // Note: multi-geometry per visual/collision is not standard URDF.
            // For full fidelity, the caller should emit separate <visual>/<collision> elements.
            // Here we output the cylinder portion; spheres must be added separately.
            out
        }
    }
}


impl RobotModel {
    /// Export the current model as a URDF XML string.
    /// Generate URDF XML from scratch (for models built programmatically).
    pub fn generate_urdf_xml(&self) -> String {
        let mut xml = format!("<?xml version=\"1.0\"?>\n<robot name=\"{}\">\n", self.name);

        for link in &self.links {
            xml.push_str(&format!("  <link name=\"{}\">\n", link.name));

            // Inertial
            let inp = &link.inertial;
            let (ix, iy, iz) = (
                inp.origin.translation.x,
                inp.origin.translation.y,
                inp.origin.translation.z,
            );
            let (ir, ip, iya) = euler_from_isometry(&inp.origin);
            xml.push_str(&format!(
                "    <inertial>\n      <origin xyz=\"{ix} {iy} {iz}\" rpy=\"{ir} {ip} {iya}\"/>\n      <mass value=\"{}\"/>\n      <inertia ixx=\"{}\" ixy=\"{}\" ixz=\"{}\" iyy=\"{}\" iyz=\"{}\" izz=\"{}\"/>\n    </inertial>\n",
                inp.mass, inp.ixx, inp.ixy, inp.ixz, inp.iyy, inp.iyz, inp.izz
            ));

            // Visuals
            for vis in &link.visuals {
                let emit_visual = |xml: &mut String, origin: &na::Isometry3<f32>, geom: &GeomData, color: &[f32; 4]| {
                    let (vx, vy, vz) = (origin.translation.x, origin.translation.y, origin.translation.z);
                    let (vr, vp, vya) = euler_from_isometry(origin);
                    xml.push_str(&format!(
                        "    <visual>\n      <origin xyz=\"{vx} {vy} {vz}\" rpy=\"{vr} {vp} {vya}\"/>\n"
                    ));
                    xml.push_str(&geom_to_urdf_xml(geom));
                    xml.push_str(&format!(
                        "      <material name=\"\">\n        <color rgba=\"{} {} {} {}\"/>\n      </material>\n",
                        color[0], color[1], color[2], color[3]
                    ));
                    xml.push_str("    </visual>\n");
                };

                match &vis.geometry {
                    GeomData::Capsule { radius, half_length } => {
                        // Decompose into cylinder + 2 spheres
                        let cyl_geom = GeomData::Cylinder { radius: *radius, half_length: *half_length };
                        emit_visual(&mut xml, &vis.origin, &cyl_geom, &vis.color);
                        let top = vis.origin * na::Translation3::new(0.0, 0.0, *half_length);
                        let top_iso = na::Isometry3::from_parts(top.translation, vis.origin.rotation);
                        let sph_geom = GeomData::Sphere { radius: *radius };
                        emit_visual(&mut xml, &top_iso, &sph_geom, &vis.color);
                        let bot = vis.origin * na::Translation3::new(0.0, 0.0, -*half_length);
                        let bot_iso = na::Isometry3::from_parts(bot.translation, vis.origin.rotation);
                        emit_visual(&mut xml, &bot_iso, &sph_geom, &vis.color);
                    }
                    _ => {
                        emit_visual(&mut xml, &vis.origin, &vis.geometry, &vis.color);
                    }
                }
            }

            // Collisions
            for col in &link.collisions {
                let emit_collision = |xml: &mut String, origin: &na::Isometry3<f32>, geom: &GeomData| {
                    let (cx, cy, cz) = (origin.translation.x, origin.translation.y, origin.translation.z);
                    let (cr, cp, cya) = euler_from_isometry(origin);
                    xml.push_str(&format!(
                        "    <collision>\n      <origin xyz=\"{cx} {cy} {cz}\" rpy=\"{cr} {cp} {cya}\"/>\n"
                    ));
                    xml.push_str(&geom_to_urdf_xml(geom));
                    xml.push_str("    </collision>\n");
                };

                match &col.geometry {
                    GeomData::Capsule { radius, half_length } => {
                        let cyl_geom = GeomData::Cylinder { radius: *radius, half_length: *half_length };
                        emit_collision(&mut xml, &col.origin, &cyl_geom);
                        let top = col.origin * na::Translation3::new(0.0, 0.0, *half_length);
                        let top_iso = na::Isometry3::from_parts(top.translation, col.origin.rotation);
                        let sph_geom = GeomData::Sphere { radius: *radius };
                        emit_collision(&mut xml, &top_iso, &sph_geom);
                        let bot = col.origin * na::Translation3::new(0.0, 0.0, -*half_length);
                        let bot_iso = na::Isometry3::from_parts(bot.translation, col.origin.rotation);
                        emit_collision(&mut xml, &bot_iso, &sph_geom);
                    }
                    _ => {
                        emit_collision(&mut xml, &col.origin, &col.geometry);
                    }
                }
            }

            xml.push_str("  </link>\n");
        }

        for joint in &self.joints {
            let (jx, jy, jz) = (
                joint.origin.translation.x,
                joint.origin.translation.y,
                joint.origin.translation.z,
            );
            let (jr, jp, jya) = euler_from_isometry(&joint.origin);
            xml.push_str(&format!(
                "  <joint name=\"{}\" type=\"{}\">\n    <origin xyz=\"{jx} {jy} {jz}\" rpy=\"{jr} {jp} {jya}\"/>\n    <parent link=\"{}\"/>\n    <child link=\"{}\"/>\n    <axis xyz=\"{} {} {}\"/>\n    <limit lower=\"{}\" upper=\"{}\" effort=\"{}\" velocity=\"{}\"/>\n  </joint>\n",
                joint.name, joint.joint_type,
                joint.parent_link, joint.child_link,
                joint.axis.x, joint.axis.y, joint.axis.z,
                joint.lower, joint.upper, joint.effort, joint.velocity
            ));
        }

        xml.push_str("</robot>\n");
        xml
    }

    /// Re-reads the original URDF, patches editable fields (mass, inertia,
    /// joint limits, joint origin, joint axis), and serializes.
    /// For models created from scratch (no source_path), generates URDF XML directly.
    pub fn export_urdf(&self) -> Result<String, String> {
        // The "re-read source, patch fields, re-serialise" path is only
        // valid when the source actually IS a URDF — otherwise `urdf_rs`
        // chokes on whatever it finds (`.misa` TOML, etc.) and the whole
        // export aborts with a misleading "Re-read URDF error". For non-
        // URDF sources (and for models built in memory) fall back to the
        // from-scratch XML generator.
        let is_urdf_source = self
            .source_path
            .as_ref()
            .and_then(|p| p.extension())
            .and_then(|e| e.to_str())
            .map(|ext| {
                let lc = ext.to_ascii_lowercase();
                lc == "urdf" || lc == "xacro"
            })
            .unwrap_or(false);
        if !is_urdf_source {
            return Ok(self.generate_urdf_xml());
        }
        let source = self
            .source_path
            .as_ref()
            .ok_or("No source URDF path stored")?;
        let mut robot =
            urdf_rs::read_file(source).map_err(|e| format!("Re-read URDF error: {e}"))?;

        // Patch link inertial data
        for our_link in &self.links {
            if let Some(urdf_link) = robot.links.iter_mut().find(|l| l.name == our_link.name) {
                urdf_link.inertial.mass.value = our_link.inertial.mass;
                urdf_link.inertial.inertia.ixx = our_link.inertial.ixx;
                urdf_link.inertial.inertia.ixy = our_link.inertial.ixy;
                urdf_link.inertial.inertia.ixz = our_link.inertial.ixz;
                urdf_link.inertial.inertia.iyy = our_link.inertial.iyy;
                urdf_link.inertial.inertia.iyz = our_link.inertial.iyz;
                urdf_link.inertial.inertia.izz = our_link.inertial.izz;
                urdf_link.inertial.origin = isometry_to_pose(&our_link.inertial.origin);
            }
        }

        // Patch visual and collision data
        for our_link in &self.links {
            if let Some(urdf_link) = robot.links.iter_mut().find(|l| l.name == our_link.name) {
                urdf_link.visual = our_link.visuals.iter().flat_map(visuals_to_urdf).collect();
                urdf_link.collision = our_link.collisions.iter().flat_map(collisions_to_urdf).collect();
            }
        }

        // Patch joint data
        for our_joint in &self.joints {
            if let Some(urdf_joint) = robot.joints.iter_mut().find(|j| j.name == our_joint.name) {
                urdf_joint.origin = isometry_to_pose(&our_joint.origin);
                urdf_joint.axis.xyz = urdf_rs::Vec3([
                    our_joint.axis.x as f64,
                    our_joint.axis.y as f64,
                    our_joint.axis.z as f64,
                ]);
                urdf_joint.limit.lower = our_joint.lower;
                urdf_joint.limit.upper = our_joint.upper;
                urdf_joint.limit.effort = our_joint.effort;
                urdf_joint.limit.velocity = our_joint.velocity;
            }
        }

        // Patch / inject mimic entries from the master format. URDF uses
        // a single linear `<mimic>` per joint; we set / clear it based on
        // whether the joint appears in `self.mimics`.
        for urdf_joint in robot.joints.iter_mut() {
            urdf_joint.mimic = self
                .mimics
                .iter()
                .find(|m| m.joint == urdf_joint.name)
                .map(|m| urdf_rs::Mimic {
                    joint: m.source.clone(),
                    multiplier: Some(m.multiplier),
                    offset: Some(m.offset),
                });
        }

        urdf_rs::write_to_string(&robot).map_err(|e| format!("URDF serialize error: {e}"))
    }

    /// Save (overwrite) the original URDF file with current edits.
    /// Mesh files are not touched since they haven't changed.
    pub fn save_urdf(&self) -> Result<PathBuf, String> {
        let source = self
            .source_path
            .clone()
            .ok_or("No source URDF path stored")?;
        // Materialise any in-memory decomposed meshes (V-HACD output)
        // to STL files next to the URDF, so the saved XML references
        // real files instead of an `unwrap_or("mesh.stl")` placeholder.
        // Done on a clone so the caller's model is left untouched.
        let mut working = self.clone();
        materialize_urdf_decomposed_meshes(&mut working, &source)?;
        let xml = working.export_urdf()?;
        std::fs::write(&source, &xml).map_err(|e| format!("Save error: {e}"))?;
        Ok(source)
    }

    /// Export the current model to a URDF file at the given path.
    /// Also copies all referenced mesh files to the output directory,
    /// preserving the relative directory structure from the package root.
    pub fn export_urdf_to_file(&self, output_path: &Path) -> Result<(), String> {
        // Materialise in-memory decomposed meshes to STL files next to
        // the *output* URDF (so the exported tree is self-contained).
        // Done on a clone so the caller's model is left untouched.
        let mut working = self.clone();
        materialize_urdf_decomposed_meshes(&mut working, output_path)?;
        let xml = working.export_urdf()?;
        std::fs::write(output_path, &xml).map_err(|e| format!("Write error: {e}"))?;

        // Copy mesh files (only if loaded from an existing file)
        let source = match self.source_path.as_ref() {
            Some(s) => s,
            None => return Ok(()), // No source path — no meshes to copy
        };

        // Non-URDF sources (`.misa` etc.) can't be re-read with `urdf_rs`
        // below. Their generated XML references the model's own relative
        // mesh paths (`meshes/<file>`), so copy those next to the output
        // URDF the same way a `.misa` save does — mirrors the source-type
        // branch introduced for `export_urdf` itself.
        let is_urdf_source = source
            .extension()
            .and_then(|e| e.to_str())
            .map(|ext| {
                let lc = ext.to_ascii_lowercase();
                lc == "urdf" || lc == "xacro"
            })
            .unwrap_or(false);
        if !is_urdf_source {
            let output_dir = output_path.parent().unwrap_or(Path::new("."));
            copy_referenced_meshes_to_misa_dir(self, Some(source), output_dir)?;
            return Ok(());
        }

        let urdf_dir = source.parent().unwrap_or(Path::new("."));
        let package_dir = urdf_dir.parent().unwrap_or(urdf_dir);
        let output_dir = output_path.parent().unwrap_or(Path::new("."));
        // The output "package dir" is the parent of the output URDF dir,
        // mirroring the original structure: <package_dir>/<urdf_subdir>/file.urdf
        let output_package_dir = output_dir.parent().unwrap_or(output_dir);

        // Re-read original URDF to get mesh filenames
        let robot =
            urdf_rs::read_file(source).map_err(|e| format!("Re-read URDF for meshes: {e}"))?;

        let mut copied: std::collections::HashSet<PathBuf> = std::collections::HashSet::new();
        let mut copy_count = 0u32;

        for link in &robot.links {
            // Collect mesh geometries from both visual and collision
            let geom_iter = link
                .visual
                .iter()
                .map(|v| &v.geometry)
                .chain(link.collision.iter().map(|c| &c.geometry));

            for geom in geom_iter {
                if let urdf_rs::Geometry::Mesh { filename, .. } = geom {
                    let src_abs = resolve_package_path(filename, package_dir);
                    if copied.contains(&src_abs) {
                        continue;
                    }
                    copied.insert(src_abs.clone());

                    if !src_abs.exists() {
                        log::warn!("Mesh file not found, skipping: {:?}", src_abs);
                        continue;
                    }

                    // Determine matched destination path
                    let dst_abs = resolve_package_path(filename, output_package_dir);

                    // Create parent directory for destination
                    if let Some(dst_parent) = dst_abs.parent() {
                        std::fs::create_dir_all(dst_parent)
                            .map_err(|e| format!("Create mesh dir {:?}: {e}", dst_parent))?;
                    }

                    // Copy (skip if src == dst)
                    if src_abs != dst_abs {
                        std::fs::copy(&src_abs, &dst_abs).map_err(|e| {
                            format!(
                                "Copy mesh {:?} -> {:?}: {e}",
                                src_abs.file_name().unwrap_or_default(),
                                dst_abs
                            )
                        })?;
                        copy_count += 1;
                    }
                }
            }
        }

        log::info!(
            "Exported URDF to {:?}, copied {} mesh file(s)",
            output_path,
            copy_count
        );
        Ok(())
    }
}


pub fn pose_to_isometry(pose: &urdf_rs::Pose) -> na::Isometry3<f32> {
    let xyz = &pose.xyz.0;
    let rpy = &pose.rpy.0;
    let translation = na::Translation3::new(xyz[0] as f32, xyz[1] as f32, xyz[2] as f32);
    let rotation =
        na::UnitQuaternion::from_euler_angles(rpy[0] as f32, rpy[1] as f32, rpy[2] as f32);
    na::Isometry3::from_parts(translation, rotation)
}

fn convert_geometry(geom: &urdf_rs::Geometry, package_dir: &Path) -> GeomData {
    match geom {
        urdf_rs::Geometry::Box { size } => GeomData::Box {
            hx: size.0[0] as f32 / 2.0,
            hy: size.0[1] as f32 / 2.0,
            hz: size.0[2] as f32 / 2.0,
        },
        urdf_rs::Geometry::Cylinder { radius, length } => GeomData::Cylinder {
            radius: *radius as f32,
            half_length: *length as f32 / 2.0,
        },
        urdf_rs::Geometry::Sphere { radius } => GeomData::Sphere {
            radius: *radius as f32,
        },
        urdf_rs::Geometry::Mesh { filename, scale } => {
            let mesh_path = resolve_package_path(filename, package_dir);
            let sf = scale
                .as_ref()
                .map(|s| [s.0[0] as f32, s.0[1] as f32, s.0[2] as f32]);
            GeomData::Mesh {
                mesh: load_mesh(&mesh_path, sf.as_ref()),
                filename: Some(filename.clone()),
                scale: sf,
            }
        }
        _ => GeomData::Box {
            hx: 0.01,
            hy: 0.01,
            hz: 0.01,
        },
    }
}

pub fn resolve_package_path(filename: &str, package_dir: &Path) -> PathBuf {
    if let Some(rest) = filename.strip_prefix("package://") {
        let (pkg_name, rel_path) = match rest.find('/') {
            Some(slash_pos) => (&rest[..slash_pos], &rest[slash_pos + 1..]),
            None => (rest, ""),
        };
        // ROS layout: URDF at <pkg>/urdf/foo.urdf, so package_dir IS the package root.
        let ros_candidate = package_dir.join(rel_path);
        if ros_candidate.exists() {
            return ros_candidate;
        }
        // Direct-in-package layout: URDF at <pkg>/foo.urdf (no urdf/ subdir),
        // so package_dir is the *parent* of the named package — append pkg_name.
        if !pkg_name.is_empty() {
            let direct_candidate = package_dir.join(pkg_name).join(rel_path);
            if direct_candidate.exists() {
                return direct_candidate;
            }
        }
        // Neither exists — return ROS candidate so the caller's warn! surfaces the expected path.
        ros_candidate
    } else if filename.starts_with("file://") {
        PathBuf::from(filename.strip_prefix("file://").unwrap())
    } else {
        PathBuf::from(filename)
    }
}




/// Write a flat `[x, y, z, nx, ny, nz]` vertex array (3 verts per tri, no indexing —
/// same shape produced by `load_stl_mesh` / `load_obj_mesh`) as a **binary STL** file.
///
/// The per-vertex normal of the first vertex is used as the per-triangle (face) normal,
/// matching what the loaders produce for flat-shaded meshes.
pub fn write_stl_binary(path: &Path, vertices: &[f32]) -> std::io::Result<()> {
    use std::io::Write;
    if vertices.len() % 18 != 0 {
        return Err(std::io::Error::new(
            std::io::ErrorKind::InvalidInput,
            format!("vertex array length {} is not a multiple of 18", vertices.len()),
        ));
    }
    if let Some(parent) = path.parent() {
        std::fs::create_dir_all(parent)?;
    }
    let mut f = std::io::BufWriter::new(std::fs::File::create(path)?);
    f.write_all(&[0u8; 80])?; // 80-byte header
    let n_tris = (vertices.len() / 18) as u32;
    f.write_all(&n_tris.to_le_bytes())?;
    for tri in vertices.chunks_exact(18) {
        // face normal = first vertex's normal (loaders write the same normal
        // to all 3 verts of a flat-shaded triangle)
        for off in [3, 4, 5] {
            f.write_all(&tri[off].to_le_bytes())?;
        }
        for vi in 0..3 {
            let base = vi * 6;
            for off in 0..3 {
                f.write_all(&tri[base + off].to_le_bytes())?;
            }
        }
        f.write_all(&[0u8; 2])?; // attribute byte count
    }
    f.flush()?;
    Ok(())
}

/// Walk the model and materialise every `GeomData::Mesh` whose `filename` is
/// `None` (i.e. produced by V-HACD / other in-memory decomposition) as a
/// binary STL file under `mesh_root/meshes/decomposed/`.
///
/// `make_ref` builds the string written back into the model's `filename` —
/// for URDF it should produce `package://<pkg>/meshes/decomposed/<fname>`;
/// for `.misa` it should produce the relative `meshes/decomposed/<fname>`.
///
/// Filenames are deterministic (`<link>_(vis|col)_<idx>.stl`), so repeated
/// Save calls overwrite the same files rather than accumulating duplicates.
pub fn materialize_decomposed_meshes<F>(
    model: &mut RobotModel,
    mesh_root: &Path,
    make_ref: F,
) -> Result<usize, String>
where
    F: Fn(&str) -> String,
{
    let subdir = Path::new("meshes/decomposed");
    let abs_dir = mesh_root.join(subdir);
    let mut written = 0usize;
    let mut need_dir = true;

    for link in &mut model.links {
        let link_name = sanitize_filename(&link.name);
        for (vi, vis) in link.visuals.iter_mut().enumerate() {
            if let GeomData::Mesh { mesh, filename, .. } = &mut vis.geometry {
                if filename.is_none() && mesh.num_triangles() > 0 {
                    if need_dir {
                        std::fs::create_dir_all(&abs_dir)
                            .map_err(|e| format!("create {abs_dir:?}: {e}"))?;
                        need_dir = false;
                    }
                    let fname = format!("{link_name}_vis_{vi}.stl");
                    write_stl_binary(&abs_dir.join(&fname), &mesh.to_flat_vertices_f32())
                        .map_err(|e| format!("write {fname}: {e}"))?;
                    *filename = Some(make_ref(&fname));
                    written += 1;
                }
            }
        }
        for (ci, col) in link.collisions.iter_mut().enumerate() {
            if let GeomData::Mesh { mesh, filename, .. } = &mut col.geometry {
                if filename.is_none() && mesh.num_triangles() > 0 {
                    if need_dir {
                        std::fs::create_dir_all(&abs_dir)
                            .map_err(|e| format!("create {abs_dir:?}: {e}"))?;
                        need_dir = false;
                    }
                    let fname = format!("{link_name}_col_{ci}.stl");
                    write_stl_binary(&abs_dir.join(&fname), &mesh.to_flat_vertices_f32())
                        .map_err(|e| format!("write {fname}: {e}"))?;
                    *filename = Some(make_ref(&fname));
                    written += 1;
                }
            }
        }
    }
    if written > 0 {
        log::info!("Materialised {written} decomposed mesh(es) under {abs_dir:?}");
    }
    Ok(written)
}

/// URDF-side wrapper: pick the correct mesh root + `package://` package name
/// based on the URDF's on-disk layout (ROS vs direct-in-package), then call
/// [`materialize_decomposed_meshes`].
pub fn materialize_urdf_decomposed_meshes(
    model: &mut RobotModel,
    urdf_path: &Path,
) -> Result<usize, String> {
    let urdf_dir = urdf_path.parent().unwrap_or(Path::new("."));
    let package_dir = urdf_dir.parent().unwrap_or(urdf_dir);

    // Pick the package root based on whether the URDF lives in a `urdf/`
    // subfolder (ROS convention) or directly in the package directory.
    //   ROS layout:    <base>/<pkg>/urdf/foo.urdf  →  mesh_root = <base>/<pkg>
    //   Direct layout: <base>/<pkg>/foo.urdf      →  mesh_root = <base>/<pkg>
    let urdf_dir_name = urdf_dir
        .file_name()
        .and_then(|n| n.to_str())
        .unwrap_or("");
    let mesh_root: PathBuf = if urdf_dir_name == "urdf" {
        package_dir.to_path_buf()
    } else {
        urdf_dir.to_path_buf()
    };

    // The URI's package name must match `mesh_root`'s directory name so that
    // `resolve_package_path` on re-load picks the file back up via its
    // `package_dir.join(pkg_name).join(rel_path)` candidate.
    let pkg_name = mesh_root
        .file_name()
        .and_then(|n| n.to_str())
        .map(|s| s.to_string())
        .unwrap_or_else(|| "robot".to_string());

    materialize_decomposed_meshes(model, &mesh_root, move |fname| {
        format!("package://{pkg_name}/meshes/decomposed/{fname}")
    })
}

/// Copy every referenced mesh file ([`GeomData::Mesh`] with
/// `filename: Some(_)`) from its current on-disk location into `misa_dir`,
/// placing it at the same relative path the `.misa` will use to reference it.
///
/// `source_path` is the path the model was originally loaded from (URDF or
/// `.misa`); it's used to resolve `package://` / relative mesh references.
/// If `None`, the helper is a no-op.
///
/// Files already at the destination (same source and destination path) are
/// skipped. Files whose source can't be found are logged via `log::warn!`
/// — non-fatal so a half-broken model still saves something useful.
fn copy_referenced_meshes_to_misa_dir(
    model: &RobotModel,
    source_path: Option<&Path>,
    misa_dir: &Path,
) -> Result<(), String> {
    let Some(source) = source_path else {
        return Ok(()); // No on-disk origin — nothing to copy.
    };
    let source_dir = source.parent().unwrap_or(Path::new("."));
    let source_pkg_dir = source_dir.parent().unwrap_or(source_dir);
    let source_is_misa = source
        .extension()
        .and_then(|e| e.to_str())
        .map(|s| s.eq_ignore_ascii_case("misa"))
        .unwrap_or(false);

    let mut copied: std::collections::HashSet<PathBuf> = std::collections::HashSet::new();

    let mut visit = |filename: &str| -> Result<(), String> {
        // misa-relative path (after stripping `package://pkg/` etc).
        let rel = misa_save::normalise_mesh_path_for_misa(filename);
        if rel.is_empty() {
            return Ok(());
        }
        let src_abs = if source_is_misa {
            source_dir.join(&rel)
        } else {
            // URDF (or other) source — use the same resolver the loader uses.
            resolve_package_path(filename, source_pkg_dir)
        };
        let dst_abs = misa_dir.join(&rel);
        if !copied.insert(dst_abs.clone()) {
            return Ok(());
        }
        if src_abs == dst_abs {
            return Ok(()); // saving in place
        }
        if !src_abs.exists() {
            log::warn!(
                "Mesh source not found, .misa will reference a missing file: {:?} (resolved from {:?})",
                src_abs, filename
            );
            return Ok(());
        }
        if let Some(parent) = dst_abs.parent() {
            std::fs::create_dir_all(parent)
                .map_err(|e| format!("create {parent:?}: {e}"))?;
        }
        std::fs::copy(&src_abs, &dst_abs)
            .map_err(|e| format!("copy {src_abs:?} -> {dst_abs:?}: {e}"))?;
        Ok(())
    };

    for link in &model.links {
        for v in &link.visuals {
            if let GeomData::Mesh { filename: Some(f), .. } = &v.geometry {
                visit(f)?;
            }
        }
        for c in &link.collisions {
            if let GeomData::Mesh { filename: Some(f), .. } = &c.geometry {
                visit(f)?;
            }
        }
    }
    Ok(())
}

fn sanitize_filename(s: &str) -> String {
    s.chars()
        .map(|c| {
            if c.is_ascii_alphanumeric() || c == '_' || c == '-' {
                c
            } else {
                '_'
            }
        })
        .collect()
}


/// Load a mesh file (STL / OBJ / DAE by extension) via [`misarta::mesh::MeshData`].
/// I/O errors, parse errors and unsupported formats log a warning and
/// return an empty mesh (0 triangles). `scale` is baked into the vertices.
pub fn load_mesh(
    path: &std::path::Path,
    scale: Option<&[f32; 3]>,
) -> std::sync::Arc<misarta::mesh::MeshData> {
    let ext = path
        .extension()
        .and_then(|e| e.to_str())
        .unwrap_or("")
        .to_lowercase();
    let mesh = match ext.as_str() {
        "stl" => misarta::mesh::MeshData::from_stl(path),
        "obj" => std::fs::read(path)
            .map_err(|e| format!("read error: {e}"))
            .and_then(|bytes| misarta::mesh::MeshData::from_obj_bytes(&bytes)),
        "dae" => misarta::collada::load_dae(path),
        other => Err(format!("unsupported mesh format '.{other}'")),
    };
    let mesh = match mesh {
        Ok(m) => m,
        Err(e) => {
            log::warn!("Failed to load mesh {:?}: {}", path, e);
            return std::sync::Arc::new(misarta::mesh::MeshData::from_flat_vertices_f32(&[]));
        }
    };
    let mesh = match scale {
        Some(&[sx, sy, sz]) if [sx, sy, sz] != [1.0, 1.0, 1.0] => {
            mesh.scaled(&na::Vector3::new(sx as f64, sy as f64, sz as f64))
        }
        _ => mesh,
    };
    log::info!(
        "Loaded mesh {:?}: {} triangles",
        path.file_name().unwrap_or_default(),
        mesh.num_triangles()
    );
    std::sync::Arc::new(mesh)
}

// Inertia computation (InertiaTensor, compute_geometry_inertia, compute_link_inertia,
// compute_geometry_volume) and validation (validate_inertia, validate_all_inertia)
// are now defined in crate::rbd::model (re-exported via pub use above).

// ─── RobotModel → .misa conversion (internal) ──────────────────────────────

mod misa_save {
    use super::*;
    use misarta::native as mn;

    pub fn build_misa_file(model: &RobotModel) -> Result<mn::MisaFile, String> {
        let mut out = mn::MisaFile::new(model.name.clone(), model.root_link.clone());

        // ── Materials ───────────────────────────────────────────────────
        // Sort by name so the on-disk order is stable across edits.
        let mut mat_names: Vec<&String> = model.materials.keys().collect();
        mat_names.sort();
        for name in &mat_names {
            let rgba = model.materials[*name];
            out.material.push(mn::Material {
                name: (*name).clone(),
                color: mn::ColorSpec::Rgba(rgba),
            });
        }

        // ── Links ───────────────────────────────────────────────────────
        for link in &model.links {
            let visuals = link
                .visuals
                .iter()
                .map(|v| {
                    let (color, material) = encode_visual_material(v.color, &model.materials);
                    mn::Visual {
                        origin: isometry_f32_to_origin(&v.origin),
                        geom: geom_data_to_geom(&v.geometry),
                        color,
                        material,
                    }
                })
                .collect();

            let collisions = link
                .collisions
                .iter()
                .map(|c| mn::Collision {
                    origin: isometry_f32_to_origin(&c.origin),
                    geom: geom_data_to_geom(&c.geometry),
                    physics: c.physics.as_ref().map(|p| mn::MjcfPhysics {
                        friction: p.friction,
                        condim: p.condim,
                        priority: p.priority,
                        solimp: p.solimp,
                        margin: p.margin,
                    }),
                })
                .collect();

            let inertial = mn::Inertial {
                mass: link.inertial.mass,
                ixx: link.inertial.ixx,
                iyy: link.inertial.iyy,
                izz: link.inertial.izz,
                ixy: link.inertial.ixy,
                ixz: link.inertial.ixz,
                iyz: link.inertial.iyz,
                origin: isometry_f32_to_origin(&link.inertial.origin),
            };

            out.link.push(mn::Link {
                name: link.name.clone(),
                description: String::new(),
                inertial,
                visual: visuals,
                collision: collisions,
                collision_enabled: link.collision_enabled,
            });
        }

        // ── Joints ──────────────────────────────────────────────────────
        for j in &model.joints {
            let kind = joint_type_str_to_kind(&j.joint_type)?;
            out.joint.push(mn::Joint {
                name: j.name.clone(),
                kind,
                parent: j.parent_link.clone(),
                child: j.child_link.clone(),
                axis: [j.axis.x as f64, j.axis.y as f64, j.axis.z as f64],
                origin: isometry_f32_to_origin(&j.origin),
                limit: mn::JointLimit {
                    lower: j.lower,
                    upper: j.upper,
                    effort: j.effort,
                    velocity: j.velocity,
                },
                dynamics: mn::JointDynamics {
                    armature: j.armature,
                    damping: j.joint_damping,
                    friction: 0.0,
                },
            });
        }

        // ── Actuators (1:1 form) ────────────────────────────────────────
        // RobotModel only carries 1:1 mappings, so we emit one [[actuator]]
        // per movable joint. Authors who want N:M must hand-edit the .misa
        // afterward.
        for j in &model.joints {
            if j.joint_type == "fixed" {
                continue;
            }
            out.actuator.push(mn::Actuator {
                name: format!("{}_motor", j.name),
                mode: actuator_mode_to_native(j.actuator_mode),
                joints: vec![mn::ActuatorJointRef {
                    name: j.name.clone(),
                    gear: 1.0,
                }],
                kp: j.actuator_kp,
                kv: j.actuator_kv,
            });
        }

        // ── Mimics ──────────────────────────────────────────────────────
        for m in &model.mimics {
            out.mimic.push(mn::Mimic {
                joint: m.joint.clone(),
                source: m.source.clone(),
                multiplier: m.multiplier,
                offset: m.offset,
            });
        }

        // ── Loop closures ───────────────────────────────────────────────
        for lc in &model.loop_closures {
            out.loop_closure.push(lc.to_config());
        }

        // ── Collision pairs ─────────────────────────────────────────────
        for cp in &model.collision_pairs {
            out.collision_pair.push(misarta::config::CollisionPairConfig {
                link_a: cp.link_a.clone(),
                link_b: cp.link_b.clone(),
                enabled: cp.enabled,
            });
        }

        // ── Sensors ─────────────────────────────────────────────────────
        for s in &model.sensors {
            out.sensor.push(mn::Sensor {
                name: s.name.clone(),
                link: s.link.clone(),
                origin: isometry_f64_to_origin(&s.origin),
                update_rate: s.update_rate,
                kind: sensor_kind_to_native(&s.kind),
            });
        }

        // ── Poses, sequences, gaits, home (reuse misarta::config types) ─
        // RobotModel.{poses, sequences, gaits} use articara-side mirror
        // structs; convert via the existing to_misarta_config path which
        // already handles the mapping.
        let cfg = model.to_misarta_config();
        for p in &cfg.pose {
            out.pose.push(p.clone());
        }
        for s in &cfg.sequence {
            out.sequence.push(s.clone());
        }
        for g in &cfg.gait {
            out.gait.push(g.clone());
        }
        out.home = cfg.home;

        Ok(out)
    }

    // ─── Conversion helpers ──────────────────────────────────────────────

    fn isometry_f32_to_origin(iso: &na::Isometry3<f32>) -> mn::Origin {
        let t = iso.translation.vector;
        let (r, p, y) = iso.rotation.euler_angles();
        let xyz = [t.x as f64, t.y as f64, t.z as f64];
        let rpy = [r as f64, p as f64, y as f64];
        let is_id = xyz[0] == 0.0
            && xyz[1] == 0.0
            && xyz[2] == 0.0
            && rpy[0] == 0.0
            && rpy[1] == 0.0
            && rpy[2] == 0.0;
        mn::Origin {
            xyz,
            rpy: if is_id { None } else { Some(rpy) },
            quat: None,
        }
    }

    fn isometry_f64_to_origin(iso: &na::Isometry3<f64>) -> mn::Origin {
        let t = iso.translation.vector;
        let (r, p, y) = iso.rotation.euler_angles();
        let xyz = [t.x, t.y, t.z];
        let rpy = [r, p, y];
        let is_id = xyz == [0.0, 0.0, 0.0] && rpy == [0.0, 0.0, 0.0];
        mn::Origin {
            xyz,
            rpy: if is_id { None } else { Some(rpy) },
            quat: None,
        }
    }

    fn geom_data_to_geom(g: &GeomData) -> mn::Geom {
        match g {
            GeomData::Box { hx, hy, hz } => mn::Geom::Box {
                size: [*hx as f64 * 2.0, *hy as f64 * 2.0, *hz as f64 * 2.0],
            },
            GeomData::Cylinder { radius, half_length } => mn::Geom::Cylinder {
                radius: *radius as f64,
                length: *half_length as f64 * 2.0,
            },
            GeomData::Sphere { radius } => mn::Geom::Sphere {
                radius: *radius as f64,
            },
            GeomData::Capsule { radius, half_length } => mn::Geom::Capsule {
                radius: *radius as f64,
                length: *half_length as f64 * 2.0,
            },
            GeomData::Mesh { filename, scale, .. } => {
                let file = filename
                    .as_ref()
                    .map(|s| normalise_mesh_path(s))
                    .unwrap_or_else(|| "meshes/unnamed.stl".to_string());
                let scale_arr = scale
                    .map(|s| [s[0] as f64, s[1] as f64, s[2] as f64])
                    .unwrap_or([1.0, 1.0, 1.0]);
                mn::Geom::Mesh {
                    file,
                    scale: scale_arr,
                }
            }
        }
    }

    /// Convert a URDF-style mesh reference into a master-relative path.
    /// `package://name/sub/path.stl` → `sub/path.stl`. Leaves already-relative
    /// paths untouched (so `meshes/foo.stl` round-trips as itself).
    pub(super) fn normalise_mesh_path_for_misa(s: &str) -> String {
        normalise_mesh_path(s)
    }

    fn normalise_mesh_path(s: &str) -> String {
        if let Some(rest) = s.strip_prefix("package://") {
            // Drop the package name (everything up to the first `/`).
            if let Some(slash) = rest.find('/') {
                return rest[slash + 1..].to_string();
            }
            return rest.to_string();
        }
        if let Some(rest) = s.strip_prefix("file://") {
            return rest.to_string();
        }
        s.to_string()
    }

    /// If `color` matches an entry in `materials` exactly, emit
    /// `material = "name"`; otherwise keep the inline RGBA. Picks the
    /// alphabetically-first matching name when several materials share
    /// the same colour, so the choice is deterministic.
    fn encode_visual_material(
        color: [f32; 4],
        materials: &HashMap<String, [f32; 4]>,
    ) -> (Option<mn::ColorSpec>, Option<String>) {
        let mut matches: Vec<&String> = materials
            .iter()
            .filter(|(_, c)| **c == color)
            .map(|(n, _)| n)
            .collect();
        matches.sort();
        if let Some(name) = matches.first() {
            (None, Some((*name).clone()))
        } else {
            (Some(mn::ColorSpec::Rgba(color)), None)
        }
    }

    fn joint_type_str_to_kind(s: &str) -> Result<mn::JointKind, String> {
        match s {
            "revolute" => Ok(mn::JointKind::Revolute),
            "continuous" => Ok(mn::JointKind::Continuous),
            "prismatic" => Ok(mn::JointKind::Prismatic),
            "fixed" => Ok(mn::JointKind::Fixed),
            "floating" => Ok(mn::JointKind::Floating),
            "planar" => Ok(mn::JointKind::Planar),
            other => Err(format!(
                "to_misa: unknown joint_type '{other}' (cannot map to JointKind)"
            )),
        }
    }

    fn actuator_mode_to_native(m: ActuatorMode) -> mn::ActuatorMode {
        match m {
            ActuatorMode::Position => mn::ActuatorMode::Position,
            ActuatorMode::Velocity => mn::ActuatorMode::Velocity,
            ActuatorMode::Torque => mn::ActuatorMode::Torque,
            ActuatorMode::ComputedTorque => mn::ActuatorMode::ComputedTorque,
            ActuatorMode::Fixed => mn::ActuatorMode::Fixed,
        }
    }

    fn sensor_kind_to_native(k: &crate::rbd::model::SensorKind) -> mn::SensorKind {
        use crate::rbd::model::SensorKind as In;
        match k {
            In::Camera { fov, width, height, near, far } => mn::SensorKind::Camera {
                fov: *fov, width: *width, height: *height, near: *near, far: *far,
            },
            In::Lidar {
                range_min, range_max, h_fov, h_samples, v_fov, v_samples,
            } => mn::SensorKind::Lidar {
                range_min: *range_min,
                range_max: *range_max,
                h_fov: *h_fov,
                h_samples: *h_samples,
                v_fov: *v_fov,
                v_samples: *v_samples,
            },
            In::Imu { gyro_noise, accel_noise } => mn::SensorKind::Imu {
                gyro_noise: *gyro_noise,
                accel_noise: *accel_noise,
            },
            In::ForceTorque { joint } => mn::SensorKind::ForceTorque {
                joint: joint.clone(),
            },
            In::Contact { partner } => mn::SensorKind::Contact {
                partner: partner.clone(),
            },
            In::Generic { kind, params } => mn::SensorKind::Generic {
                kind: kind.clone(),
                params: params.clone(),
            },
        }
    }
}
