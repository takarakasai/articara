use nalgebra as na;
use std::collections::{HashMap, HashSet};
use std::io::BufReader;
use std::path::{Path, PathBuf};

// Re-export all core types from rbd::model so that `crate::robot::RobotModel`
// (and friends) continues to work everywhere.
pub use crate::rbd::model::*;

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

    /// Load a robot model from any supported format (auto-detected by extension).
    pub fn from_file(path: &Path) -> Result<Self, String> {
        use crate::format::RobotFormat;
        let fmt = RobotFormat::detect(path)
            .ok_or_else(|| format!("Unknown file format: {:?}", path.extension()))?;
        match fmt {
            RobotFormat::Urdf => Self::from_urdf(path),
            RobotFormat::Sdf => crate::sdf::import_sdf(path),
            RobotFormat::Mjcf => crate::mjcf::import_mjcf(path),
            RobotFormat::IsaacUsd => crate::usd_import::import_usda(path),
            RobotFormat::Misa => Self::from_misa(path),
        }
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
                let ext = path
                    .extension()
                    .and_then(|e| e.to_str())
                    .unwrap_or("")
                    .to_lowercase();
                let vertices = match ext.as_str() {
                    "stl" => load_stl_mesh_with_scale(&path, scale_arr),
                    "obj" => super::load_obj_mesh(&path, Some(&scale_arr)),
                    "dae" => super::load_dae_mesh(&path, Some(&scale_arr)),
                    _ => {
                        log::warn!(
                            ".misa references unsupported mesh format {:?}: {:?}",
                            ext, path
                        );
                        Vec::new()
                    }
                };
                GeomData::Mesh {
                    vertices,
                    filename: Some(file.clone()),
                    scale: Some(scale_arr),
                }
            }
        }
    }

    /// STL loader that takes a plain `[f32; 3]` scale (parallel to the
    /// URDF-side `load_stl_mesh` which takes a `urdf_rs::Vec3`). Kept
    /// separate to avoid touching the URDF code path while .misa lands;
    /// can be unified in a follow-up refactor.
    fn load_stl_mesh_with_scale(path: &PathBuf, scale: [f32; 3]) -> Vec<f32> {
        let file = match std::fs::File::open(path) {
            Ok(f) => f,
            Err(e) => {
                log::warn!("Failed to open STL file {:?}: {}", path, e);
                return Vec::new();
            }
        };
        let mut reader = BufReader::new(file);
        let mesh = match stl_io::read_stl(&mut reader) {
            Ok(m) => m,
            Err(e) => {
                log::warn!("Failed to read STL {:?}: {}", path, e);
                return Vec::new();
            }
        };

        let mut vertices = Vec::with_capacity(mesh.faces.len() * 3 * 6);
        for face in &mesh.faces {
            let nx = face.normal[0];
            let ny = face.normal[1];
            let nz = face.normal[2];
            for &vi in &face.vertices {
                let vtx = &mesh.vertices[vi];
                vertices.push(vtx[0] * scale[0]);
                vertices.push(vtx[1] * scale[1]);
                vertices.push(vtx[2] * scale[2]);
                vertices.push(nx);
                vertices.push(ny);
                vertices.push(nz);
            }
        }
        vertices
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

impl RobotModel {
    /// Pick: find the closest link hit by a ray, given current world transforms.
    /// Uses two-pass: bounding sphere (coarse) → triangle/analytic intersection (precise).
    /// Returns (link_index, distance) or None.
    pub fn pick_link(
        &self,
        ray_origin: &na::Point3<f32>,
        ray_dir: &na::Vector3<f32>,
        transforms: &std::collections::HashMap<String, na::Isometry3<f32>>,
    ) -> Option<(usize, f32)> {
        let mut best: Option<(usize, f32)> = None;

        for (li, link) in self.links.iter().enumerate() {
            let world_tf = transforms
                .get(&link.name)
                .copied()
                .unwrap_or(na::Isometry3::identity());

            // Coarse pass: bounding sphere
            let (local_center, radius) = self.link_bounding_sphere(li);
            if radius < 1e-6 {
                continue; // Skip links with no visual geometry
            }
            let world_center = world_tf * local_center;
            if ray_sphere_intersect(ray_origin, ray_dir, &world_center, radius).is_none() {
                continue; // Ray misses bounding sphere
            }

            // Precise pass: test against actual geometry of each visual
            let mut link_best_dist: Option<f32> = None;
            for vis in &link.visuals {
                let full_tf = world_tf * vis.origin;
                let dist = precise_geometry_intersect(
                    ray_origin, ray_dir, &full_tf, &vis.geometry,
                );
                if let Some(d) = dist {
                    if d > 0.0 && (link_best_dist.is_none() || d < link_best_dist.unwrap()) {
                        link_best_dist = Some(d);
                    }
                }
            }

            if let Some(d) = link_best_dist {
                if best.is_none() || d < best.unwrap().1 {
                    best = Some((li, d));
                }
            }
        }
        best
    }
}

// ========== Ray Intersection Tests ==========

/// Precise geometry intersection: transforms ray into geometry-local space and tests.
pub fn precise_geometry_intersect(
    ray_origin: &na::Point3<f32>,
    ray_dir: &na::Vector3<f32>,
    geom_tf: &na::Isometry3<f32>,
    geom: &GeomData,
) -> Option<f32> {
    // Transform ray into geometry's local frame
    let inv_tf = geom_tf.inverse();
    let local_origin = inv_tf * ray_origin;
    let local_dir = inv_tf * ray_dir;

    match geom {
        GeomData::Box { hx, hy, hz } => ray_box_intersect(&local_origin, &local_dir, *hx, *hy, *hz),
        GeomData::Cylinder { radius, half_length } => {
            ray_cylinder_intersect(&local_origin, &local_dir, *radius, *half_length)
        }
        GeomData::Sphere { radius } => {
            ray_sphere_intersect(&local_origin, &local_dir, &na::Point3::origin(), *radius)
        }
        GeomData::Capsule { radius, half_length } => {
            // Test cylinder body + two hemisphere caps
            let t_cyl = ray_cylinder_intersect(&local_origin, &local_dir, *radius, *half_length);
            let top_center = na::Point3::new(0.0, 0.0, *half_length);
            let bot_center = na::Point3::new(0.0, 0.0, -*half_length);
            let t_top = ray_sphere_intersect(&local_origin, &local_dir, &top_center, *radius);
            let t_bot = ray_sphere_intersect(&local_origin, &local_dir, &bot_center, *radius);
            [t_cyl, t_top, t_bot].iter().filter_map(|t| *t).reduce(f32::min)
        }
        GeomData::Mesh { vertices, .. } => ray_mesh_intersect(&local_origin, &local_dir, vertices),
    }
}

/// Ray-sphere intersection. Returns the nearest positive distance or None.
pub fn ray_sphere_intersect(
    origin: &na::Point3<f32>,
    dir: &na::Vector3<f32>,
    center: &na::Point3<f32>,
    radius: f32,
) -> Option<f32> {
    let oc = origin - center;
    let a = dir.dot(dir);
    let b = 2.0 * oc.dot(dir);
    let c = oc.dot(&oc) - radius * radius;
    let disc = b * b - 4.0 * a * c;
    if disc < 0.0 {
        return None;
    }
    let sqrt_disc = disc.sqrt();
    let t1 = (-b - sqrt_disc) / (2.0 * a);
    let t2 = (-b + sqrt_disc) / (2.0 * a);
    if t1 > 0.0 {
        Some(t1)
    } else if t2 > 0.0 {
        Some(t2)
    } else {
        None
    }
}

/// Ray-AABB (box) intersection using slab method.
pub fn ray_box_intersect(
    origin: &na::Point3<f32>,
    dir: &na::Vector3<f32>,
    hx: f32,
    hy: f32,
    hz: f32,
) -> Option<f32> {
    let mut tmin = f32::NEG_INFINITY;
    let mut tmax = f32::INFINITY;
    let halves = [hx, hy, hz];

    for i in 0..3 {
        if dir[i].abs() < 1e-10 {
            if origin[i] < -halves[i] || origin[i] > halves[i] {
                return None;
            }
        } else {
            let inv_d = 1.0 / dir[i];
            let mut t1 = (-halves[i] - origin[i]) * inv_d;
            let mut t2 = (halves[i] - origin[i]) * inv_d;
            if t1 > t2 {
                std::mem::swap(&mut t1, &mut t2);
            }
            tmin = tmin.max(t1);
            tmax = tmax.min(t2);
            if tmin > tmax {
                return None;
            }
        }
    }
    if tmax < 0.0 {
        None
    } else if tmin > 0.0 {
        Some(tmin)
    } else {
        Some(tmax)
    }
}

/// Ray-cylinder intersection (Z-axis aligned, centered at origin).
pub fn ray_cylinder_intersect(
    origin: &na::Point3<f32>,
    dir: &na::Vector3<f32>,
    radius: f32,
    half_length: f32,
) -> Option<f32> {
    // Infinite cylinder in XY
    let a = dir.x * dir.x + dir.y * dir.y;
    let b = 2.0 * (origin.x * dir.x + origin.y * dir.y);
    let c = origin.x * origin.x + origin.y * origin.y - radius * radius;
    let disc = b * b - 4.0 * a * c;

    let mut best: Option<f32> = None;

    if disc >= 0.0 && a > 1e-10 {
        let sqrt_disc = disc.sqrt();
        for &t in &[(-b - sqrt_disc) / (2.0 * a), (-b + sqrt_disc) / (2.0 * a)] {
            if t > 0.0 {
                let z = origin.z + t * dir.z;
                if z.abs() <= half_length {
                    if best.is_none() || t < best.unwrap() {
                        best = Some(t);
                    }
                }
            }
        }
    }

    // Cap discs (top and bottom)
    if dir.z.abs() > 1e-10 {
        for &cap_z in &[half_length, -half_length] {
            let t = (cap_z - origin.z) / dir.z;
            if t > 0.0 {
                let px = origin.x + t * dir.x;
                let py = origin.y + t * dir.y;
                if px * px + py * py <= radius * radius {
                    if best.is_none() || t < best.unwrap() {
                        best = Some(t);
                    }
                }
            }
        }
    }

    best
}

/// Ray-mesh (triangle soup) intersection using Möller–Trumbore algorithm.
/// Vertices are in flat format: [x, y, z, nx, ny, nz, x, y, z, nx, ny, nz, ...].
/// Every 3 vertices (18 floats) form one triangle.
pub fn ray_mesh_intersect(
    origin: &na::Point3<f32>,
    dir: &na::Vector3<f32>,
    vertices: &[f32],
) -> Option<f32> {
    let mut best: Option<f32> = None;
    let stride = 6; // x,y,z,nx,ny,nz per vertex
    let tri_stride = stride * 3; // 18 floats per triangle

    let mut i = 0;
    while i + tri_stride <= vertices.len() {
        let v0 = na::Point3::new(vertices[i], vertices[i + 1], vertices[i + 2]);
        let v1 = na::Point3::new(vertices[i + stride], vertices[i + stride + 1], vertices[i + stride + 2]);
        let v2 = na::Point3::new(vertices[i + stride * 2], vertices[i + stride * 2 + 1], vertices[i + stride * 2 + 2]);

        if let Some(t) = ray_triangle_intersect(origin, dir, &v0, &v1, &v2) {
            if t > 0.0 && (best.is_none() || t < best.unwrap()) {
                best = Some(t);
            }
        }
        i += tri_stride;
    }
    best
}

/// Möller–Trumbore ray-triangle intersection.
pub fn ray_triangle_intersect(
    origin: &na::Point3<f32>,
    dir: &na::Vector3<f32>,
    v0: &na::Point3<f32>,
    v1: &na::Point3<f32>,
    v2: &na::Point3<f32>,
) -> Option<f32> {
    let edge1 = v1 - v0;
    let edge2 = v2 - v0;
    let h = dir.cross(&edge2);
    let a = edge1.dot(&h);
    if a.abs() < 1e-8 {
        return None; // Ray parallel to triangle
    }
    let f = 1.0 / a;
    let s = origin - v0;
    let u = f * s.dot(&h);
    if !(0.0..=1.0).contains(&u) {
        return None;
    }
    let q = s.cross(&edge1);
    let v = f * dir.dot(&q);
    if v < 0.0 || u + v > 1.0 {
        return None;
    }
    let t = f * edge2.dot(&q);
    if t > 1e-6 {
        Some(t)
    } else {
        None
    }
}

/// Find closest approach of a ray to an axis line (infinite line through origin in given direction).
/// Returns `(t_line, distance)` where `t_line` is the parameter along the axis
/// (point on axis = `axis_origin + axis_dir * t_line`) and `distance` is the
/// closest distance between the ray and the axis line.
pub fn ray_axis_closest(
    ro: &na::Point3<f32>,
    rd: &na::Vector3<f32>,
    axis_origin: &na::Point3<f32>,
    axis_dir: &na::Vector3<f32>,
) -> (f32, f32) {
    let w = ro - axis_origin;
    let a = rd.dot(rd);
    let b = rd.dot(axis_dir);
    let c = axis_dir.dot(axis_dir);
    let d = rd.dot(&w);
    let e = axis_dir.dot(&w);
    let denom = a * c - b * b;

    if denom.abs() < 1e-10 {
        // Ray parallel to axis
        let t_line = e / c;
        let closest_on_line = axis_origin + axis_dir * t_line;
        let dist = (ro - closest_on_line).norm();
        (t_line, dist)
    } else {
        let t_ray = (b * e - c * d) / denom;
        let t_line = (a * e - b * d) / denom;
        let p_ray = ro + rd * t_ray;
        let p_line = axis_origin + axis_dir * t_line;
        let dist = (p_ray - p_line).norm();
        (t_line, dist)
    }
}

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
    // ========== Model editing: Add / Remove links and joints ==========

    /// Create a new empty model with a single root link.
    pub fn new_empty(name: &str) -> Self {
        let root_name = "base_link".to_string();
        let mut link_map = HashMap::new();
        link_map.insert(root_name.clone(), 0);
        let mut model = Self {
            name: name.to_string(),
            links: vec![LinkData {
                name: root_name.clone(),
                visuals: vec![VisualData {
                    origin: na::Isometry3::identity(),
                    geometry: GeomData::Box { hx: 0.05, hy: 0.05, hz: 0.025 },
                    color: [0.7, 0.7, 0.7, 1.0],
                }],
                collisions: Vec::new(),
                inertial: InertialData {
                    origin: na::Isometry3::identity(),
                    mass: 1.0,
                    ixx: 0.001, ixy: 0.0, ixz: 0.0,
                    iyy: 0.001, iyz: 0.0, izz: 0.001,
                },
            }],
            joints: Vec::new(),
            link_map,
            joint_map: HashMap::new(),
            root_link: root_name,
            children_joints: HashMap::new(),
            materials: HashMap::new(),
            joint_positions: Vec::new(),
            source_path: None,
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
        model.rebuild_misarta_model();
        model
    }

    /// Generate a unique link name that doesn't collide with existing ones.
    pub fn generate_link_name(&self, base: &str) -> String {
        if !self.link_map.contains_key(base) {
            return base.to_string();
        }
        for i in 1.. {
            let name = format!("{base}_{i}");
            if !self.link_map.contains_key(&name) {
                return name;
            }
        }
        unreachable!()
    }

    /// Generate a unique joint name that doesn't collide with existing ones.
    pub fn generate_joint_name(&self, base: &str) -> String {
        if !self.joint_map.contains_key(base) {
            return base.to_string();
        }
        for i in 1.. {
            let name = format!("{base}_{i}");
            if !self.joint_map.contains_key(&name) {
                return name;
            }
        }
        unreachable!()
    }

    /// Add a new link with default values. Returns the index of the new link.
    pub fn add_link(&mut self, name: &str, geometry: GeomData, color: [f32; 4]) -> usize {
        let idx = self.links.len();
        self.link_map.insert(name.to_string(), idx);
        self.links.push(LinkData {
            name: name.to_string(),
            visuals: vec![VisualData {
                origin: na::Isometry3::identity(),
                geometry,
                color,
            }],
            collisions: Vec::new(),
            inertial: InertialData {
                origin: na::Isometry3::identity(),
                mass: 0.1,
                ixx: 0.0001, ixy: 0.0, ixz: 0.0,
                iyy: 0.0001, iyz: 0.0, izz: 0.0001,
            },
        });
        self.misarta_cache = None; // invalidate stale cache
        idx
    }

    /// Add a new joint connecting parent_link to child_link.
    /// Returns the index of the new joint, or Err if parent/child not found.
    pub fn add_joint(
        &mut self,
        name: &str,
        joint_type: &str,
        parent_link: &str,
        child_link: &str,
        origin: na::Isometry3<f32>,
        axis: na::Vector3<f32>,
        lower: f64,
        upper: f64,
    ) -> Result<usize, String> {
        if !self.link_map.contains_key(parent_link) {
            return Err(format!("Parent link '{}' not found", parent_link));
        }
        if !self.link_map.contains_key(child_link) {
            return Err(format!("Child link '{}' not found", child_link));
        }
        let idx = self.joints.len();
        self.joint_map.insert(name.to_string(), idx);
        self.children_joints
            .entry(parent_link.to_string())
            .or_default()
            .push(idx);
        self.joints.push(JointData {
            name: name.to_string(),
            joint_type: joint_type.to_string(),
            parent_link: parent_link.to_string(),
            child_link: child_link.to_string(),
            origin,
            axis,
            lower,
            upper,
            effort: 10.0,
            velocity: 5.0,
            actuator_mode: crate::rbd::model::ActuatorMode::default(),
            actuator_kp: 50.0,
            actuator_kv: 5.0,
                    // Match `default_armature()` — see comment on the URDF
                    // loader path for the rationale.
                    armature: 0.0014,
                    joint_damping: 0.0,
        });
        self.joint_positions.push(0.0);
        self.misarta_cache = None; // invalidate stale cache
        Ok(idx)
    }

    /// Add a child link + joint pair in one step.
    /// Creates a new link, then a joint connecting parent → new link.
    /// Returns (link_index, joint_index).
    pub fn add_child(
        &mut self,
        parent_link: &str,
        link_name: &str,
        joint_name: &str,
        joint_type: &str,
        origin: na::Isometry3<f32>,
        axis: na::Vector3<f32>,
        geometry: GeomData,
        color: [f32; 4],
        lower: f64,
        upper: f64,
    ) -> Result<(usize, usize), String> {
        let li = self.add_link(link_name, geometry, color);
        let ji = self.add_joint(joint_name, joint_type, parent_link, link_name, origin, axis, lower, upper)?;
        Ok((li, ji))
    }

    /// Remove a link and all joints that reference it (parent or child).
    /// Also removes child links recursively. Returns the names of removed links.
    pub fn remove_link(&mut self, link_name: &str) -> Result<Vec<String>, String> {
        if link_name == self.root_link {
            return Err("Cannot remove the root link".to_string());
        }
        if !self.link_map.contains_key(link_name) {
            return Err(format!("Link '{}' not found", link_name));
        }

        // Collect all links to remove (this link + all descendants)
        let mut to_remove = Vec::new();
        self.collect_descendants(link_name, &mut to_remove);

        // Remove joints that reference any of the removed links
        let remove_set: HashSet<String> = to_remove.iter().cloned().collect();
        self.joints.retain(|j| {
            !remove_set.contains(&j.parent_link) || !remove_set.contains(&j.child_link)
        });
        // Also remove the joint whose child is link_name
        self.joints.retain(|j| !remove_set.contains(&j.child_link));

        // Remove the links themselves
        self.links.retain(|l| !remove_set.contains(&l.name));

        // Rebuild indices
        self.rebuild_indices();
        Ok(to_remove)
    }

    /// Collect a link and all its descendants.
    fn collect_descendants(&self, link_name: &str, result: &mut Vec<String>) {
        result.push(link_name.to_string());
        if let Some(child_joints) = self.children_joints.get(link_name) {
            for &ji in child_joints {
                let child = &self.joints[ji].child_link;
                self.collect_descendants(child, result);
            }
        }
    }

    /// Rebuild all index maps after structural changes (add/remove).
    pub fn rebuild_indices(&mut self) {
        self.link_map.clear();
        for (i, link) in self.links.iter().enumerate() {
            self.link_map.insert(link.name.clone(), i);
        }
        self.joint_map.clear();
        self.children_joints.clear();
        for (i, joint) in self.joints.iter().enumerate() {
            self.joint_map.insert(joint.name.clone(), i);
            self.children_joints
                .entry(joint.parent_link.clone())
                .or_default()
                .push(i);
        }
        // Fix joint_positions length
        self.joint_positions.resize(self.joints.len(), 0.0);
        self.rebuild_misarta_model();
    }

    /// Rename a link.  Updates the canonical name, all joint parent/child
    /// references, loop-closure references, and rebuilds derived indices.
    /// Returns `true` on success, `false` if `new_name` is empty or already taken.
    pub fn rename_link(&mut self, old_name: &str, new_name: &str) -> bool {
        let new_name = new_name.trim();
        if new_name.is_empty() || new_name == old_name {
            return false;
        }
        // Reject duplicates
        if self.link_map.contains_key(new_name) {
            return false;
        }
        // Find link index
        let Some(&li) = self.link_map.get(old_name) else {
            return false;
        };
        // 1. Rename the link itself
        self.links[li].name = new_name.to_string();
        // 2. Update root_link
        if self.root_link == old_name {
            self.root_link = new_name.to_string();
        }
        // 3. Update all joints referencing this link
        for joint in &mut self.joints {
            if joint.parent_link == old_name {
                joint.parent_link = new_name.to_string();
            }
            if joint.child_link == old_name {
                joint.child_link = new_name.to_string();
            }
        }
        // 4. Update loop-closure references
        for lc in &mut self.loop_closures {
            if lc.link_a == old_name {
                lc.link_a = new_name.to_string();
            }
            if lc.link_b == old_name {
                lc.link_b = new_name.to_string();
            }
        }
        // 5. Rebuild all derived maps
        self.rebuild_indices();
        true
    }

    /// Rename a joint.  Updates the canonical name and rebuilds derived indices.
    /// Returns `true` on success, `false` if `new_name` is empty or already taken.
    pub fn rename_joint(&mut self, old_name: &str, new_name: &str) -> bool {
        let new_name = new_name.trim();
        if new_name.is_empty() || new_name == old_name {
            return false;
        }
        if self.joint_map.contains_key(new_name) {
            return false;
        }
        let Some(&ji) = self.joint_map.get(old_name) else {
            return false;
        };
        self.joints[ji].name = new_name.to_string();
        self.rebuild_indices();
        true
    }

    /// Return a list of all link names (for UI combo boxes).
    pub fn link_names(&self) -> Vec<String> {
        self.links.iter().map(|l| l.name.clone()).collect()
    }

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
        if self.source_path.is_none() {
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
            let ext = mesh_path
                .extension()
                .and_then(|e| e.to_str())
                .unwrap_or("")
                .to_lowercase();
            let vertices = match ext.as_str() {
                "stl" => load_stl_mesh(&mesh_path, scale.as_ref()),
                "obj" => load_obj_mesh(&mesh_path, sf.as_ref()),
                "dae" => load_dae_mesh(&mesh_path, sf.as_ref()),
                _ => {
                    log::warn!(
                        "Unsupported mesh format {:?}: {:?}",
                        ext,
                        mesh_path
                    );
                    Vec::new()
                }
            };
            GeomData::Mesh {
                vertices,
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

fn load_stl_mesh(path: &PathBuf, scale: Option<&urdf_rs::Vec3>) -> Vec<f32> {
    let sf = scale
        .map(|s| [s.0[0] as f32, s.0[1] as f32, s.0[2] as f32])
        .unwrap_or([1.0, 1.0, 1.0]);

    let file = match std::fs::File::open(path) {
        Ok(f) => f,
        Err(e) => {
            log::warn!("Failed to open STL file {:?}: {}", path, e);
            return Vec::new();
        }
    };
    let mut reader = BufReader::new(file);
    let mesh = match stl_io::read_stl(&mut reader) {
        Ok(m) => m,
        Err(e) => {
            log::warn!("Failed to read STL {:?}: {}", path, e);
            return Vec::new();
        }
    };

    let mut vertices = Vec::with_capacity(mesh.faces.len() * 3 * 6);
    for face in &mesh.faces {
        let nx = face.normal[0];
        let ny = face.normal[1];
        let nz = face.normal[2];
        for &vi in &face.vertices {
            let vtx = &mesh.vertices[vi];
            vertices.push(vtx[0] * sf[0]);
            vertices.push(vtx[1] * sf[1]);
            vertices.push(vtx[2] * sf[2]);
            vertices.push(nx);
            vertices.push(ny);
            vertices.push(nz);
        }
    }

    log::info!(
        "Loaded STL {:?}: {} triangles",
        path.file_name().unwrap_or_default(),
        mesh.faces.len()
    );
    vertices
}

/// Public wrapper for loading an STL mesh from an absolute path (no scale).
pub fn load_stl_mesh_public(path: &PathBuf) -> Vec<f32> {
    load_stl_mesh(path, None)
}

/// Load a Wavefront OBJ mesh, returning flat `[x, y, z, nx, ny, nz]` per vertex
/// (same format as `load_stl_mesh`). Normals are recomputed per-triangle
/// (flat shading) — the file's own normals are ignored so output matches STL.
pub fn load_obj_mesh(path: &PathBuf, scale: Option<&[f32; 3]>) -> Vec<f32> {
    let sf = scale.copied().unwrap_or([1.0, 1.0, 1.0]);

    let (models, _materials) = match tobj::load_obj(
        path,
        &tobj::LoadOptions {
            triangulate: true,
            single_index: true,
            ..Default::default()
        },
    ) {
        Ok(v) => v,
        Err(e) => {
            log::warn!("Failed to load OBJ {:?}: {}", path, e);
            return Vec::new();
        }
    };

    let mut vertices: Vec<f32> = Vec::new();
    let mut tri_count: usize = 0;
    for model in &models {
        let mesh = &model.mesh;
        let pos = &mesh.positions;
        for tri in mesh.indices.chunks_exact(3) {
            let i0 = tri[0] as usize * 3;
            let i1 = tri[1] as usize * 3;
            let i2 = tri[2] as usize * 3;
            if i0 + 2 >= pos.len() || i1 + 2 >= pos.len() || i2 + 2 >= pos.len() {
                continue;
            }
            let v0 = [pos[i0], pos[i0 + 1], pos[i0 + 2]];
            let v1 = [pos[i1], pos[i1 + 1], pos[i1 + 2]];
            let v2 = [pos[i2], pos[i2 + 1], pos[i2 + 2]];
            let e1 = [v1[0] - v0[0], v1[1] - v0[1], v1[2] - v0[2]];
            let e2 = [v2[0] - v0[0], v2[1] - v0[1], v2[2] - v0[2]];
            let mut nx = e1[1] * e2[2] - e1[2] * e2[1];
            let mut ny = e1[2] * e2[0] - e1[0] * e2[2];
            let mut nz = e1[0] * e2[1] - e1[1] * e2[0];
            let nlen = (nx * nx + ny * ny + nz * nz).sqrt();
            if nlen > 1e-20 {
                nx /= nlen;
                ny /= nlen;
                nz /= nlen;
            }
            for v in [v0, v1, v2] {
                vertices.push(v[0] * sf[0]);
                vertices.push(v[1] * sf[1]);
                vertices.push(v[2] * sf[2]);
                vertices.push(nx);
                vertices.push(ny);
                vertices.push(nz);
            }
            tri_count += 1;
        }
    }

    log::info!(
        "Loaded OBJ {:?}: {} triangles",
        path.file_name().unwrap_or_default(),
        tri_count
    );
    vertices
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
            if let GeomData::Mesh { vertices, filename, .. } = &mut vis.geometry {
                if filename.is_none() && !vertices.is_empty() {
                    if need_dir {
                        std::fs::create_dir_all(&abs_dir)
                            .map_err(|e| format!("create {abs_dir:?}: {e}"))?;
                        need_dir = false;
                    }
                    let fname = format!("{link_name}_vis_{vi}.stl");
                    write_stl_binary(&abs_dir.join(&fname), vertices)
                        .map_err(|e| format!("write {fname}: {e}"))?;
                    *filename = Some(make_ref(&fname));
                    written += 1;
                }
            }
        }
        for (ci, col) in link.collisions.iter_mut().enumerate() {
            if let GeomData::Mesh { vertices, filename, .. } = &mut col.geometry {
                if filename.is_none() && !vertices.is_empty() {
                    if need_dir {
                        std::fs::create_dir_all(&abs_dir)
                            .map_err(|e| format!("create {abs_dir:?}: {e}"))?;
                        need_dir = false;
                    }
                    let fname = format!("{link_name}_col_{ci}.stl");
                    write_stl_binary(&abs_dir.join(&fname), vertices)
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

/// Load a Collada (.dae) mesh file, returning flat vertex data
/// `[x, y, z, nx, ny, nz]` per vertex (same format as STL loader).
///
/// Handles `<triangles>` and `<polylist>` (triangles-only) elements.
/// Applies an optional uniform scale.
pub fn load_dae_mesh(path: &PathBuf, scale: Option<&[f32; 3]>) -> Vec<f32> {
    let sf = scale.copied().unwrap_or([1.0, 1.0, 1.0]);

    let text = match std::fs::read_to_string(path) {
        Ok(t) => t,
        Err(e) => {
            log::warn!("Failed to read DAE file {:?}: {}", path, e);
            return Vec::new();
        }
    };
    let doc = match roxmltree::Document::parse(&text) {
        Ok(d) => d,
        Err(e) => {
            log::warn!("Failed to parse DAE XML {:?}: {}", path, e);
            return Vec::new();
        }
    };

    // Detect up-axis to build a correction matrix.
    // Collada default is Y_UP; many robot meshes use Z_UP.
    let up_axis = doc
        .descendants()
        .find(|n| n.has_tag_name("up_axis"))
        .and_then(|n| n.text())
        .unwrap_or("Y_UP");

    // We want Z-up output (robotics convention).
    // Y_UP → rotate -90° around X  (y→z, z→-y)
    // X_UP → rotate  90° around Z  (x→y, y→-x)  then same as Y_UP
    let apply_up = |x: f32, y: f32, z: f32| -> [f32; 3] {
        match up_axis {
            "Z_UP" => [x, y, z],
            "X_UP" => [y, z, x],
            _ /* Y_UP */ => [x, z, -y],
        }
    };

    let mut all_vertices: Vec<f32> = Vec::new();

    // Helper: parse a whitespace-separated float array from text content.
    fn parse_floats(text: &str) -> Vec<f32> {
        text.split_whitespace()
            .filter_map(|s| s.parse::<f32>().ok())
            .collect()
    }

    fn parse_ints(text: &str) -> Vec<usize> {
        text.split_whitespace()
            .filter_map(|s| s.parse::<usize>().ok())
            .collect()
    }

    // Iterate over all <geometry> → <mesh> elements.
    for mesh_node in doc
        .descendants()
        .filter(|n| n.has_tag_name("mesh"))
    {
        // Collect <source> elements by their id.
        let mut sources: std::collections::HashMap<String, Vec<f32>> =
            std::collections::HashMap::new();
        for source in mesh_node.children().filter(|n| n.has_tag_name("source")) {
            if let Some(id) = source.attribute("id") {
                if let Some(fa) = source.children().find(|n| n.has_tag_name("float_array")) {
                    if let Some(text) = fa.text() {
                        sources.insert(id.to_string(), parse_floats(text));
                    }
                }
            }
        }

        // <vertices> maps a semantic to a source.
        let mut vertex_source_id: Option<String> = None;
        if let Some(verts_node) = mesh_node.children().find(|n| n.has_tag_name("vertices")) {
            for input in verts_node.children().filter(|n| n.has_tag_name("input")) {
                if input.attribute("semantic") == Some("POSITION") {
                    if let Some(src) = input.attribute("source") {
                        vertex_source_id = Some(src.trim_start_matches('#').to_string());
                    }
                }
            }
        }

        // Process <triangles> and <polylist> elements.
        let tri_elements: Vec<_> = mesh_node
            .children()
            .filter(|n| n.has_tag_name("triangles") || n.has_tag_name("polylist"))
            .collect();

        for tri_elem in tri_elements {
            // Gather <input> semantics, offsets, sources.
            let mut pos_offset: Option<usize> = None;
            let mut norm_offset: Option<usize> = None;
            let mut pos_source: Option<String> = None;
            let mut norm_source: Option<String> = None;
            let mut max_offset: usize = 0;

            for input in tri_elem.children().filter(|n| n.has_tag_name("input")) {
                let semantic = input.attribute("semantic").unwrap_or("");
                let offset: usize = input
                    .attribute("offset")
                    .and_then(|s| s.parse().ok())
                    .unwrap_or(0);
                let src = input
                    .attribute("source")
                    .unwrap_or("")
                    .trim_start_matches('#')
                    .to_string();
                if offset > max_offset {
                    max_offset = offset;
                }
                match semantic {
                    "VERTEX" => {
                        pos_offset = Some(offset);
                        // VERTEX refers to <vertices>, which in turn refers to the position source.
                        pos_source = vertex_source_id.clone();
                    }
                    "NORMAL" => {
                        norm_offset = Some(offset);
                        norm_source = Some(src);
                    }
                    _ => {}
                }
            }

            let stride = max_offset + 1;

            let positions = pos_source
                .as_ref()
                .and_then(|id| sources.get(id));
            let normals = norm_source
                .as_ref()
                .and_then(|id| sources.get(id));

            // For <polylist>, check <vcount> — we only handle triangles (all 3s).
            let is_polylist = tri_elem.has_tag_name("polylist");
            let vcounts: Vec<usize> = if is_polylist {
                tri_elem
                    .children()
                    .find(|n| n.has_tag_name("vcount"))
                    .and_then(|n| n.text())
                    .map(|t| parse_ints(t))
                    .unwrap_or_default()
            } else {
                Vec::new()
            };

            // Parse <p> index data.
            let indices: Vec<usize> = tri_elem
                .children()
                .find(|n| n.has_tag_name("p"))
                .and_then(|n| n.text())
                .map(|t| parse_ints(t))
                .unwrap_or_default();

            if let Some(positions) = positions {
                if is_polylist {
                    // Walk vcounts
                    let mut idx_cursor = 0usize;
                    for &vc in &vcounts {
                        if vc < 3 {
                            idx_cursor += vc * stride;
                            continue;
                        }
                        // Fan triangulate: vertex 0, i, i+1
                        for t in 0..(vc - 2) {
                            let fan_indices = [0, t + 1, t + 2];
                            for &fi in &fan_indices {
                                let base = idx_cursor + fi * stride;
                                let pi = pos_offset.map(|o| indices[base + o]).unwrap_or(0);
                                let ni = norm_offset.map(|o| indices[base + o]);

                                let px = positions.get(pi * 3).copied().unwrap_or(0.0);
                                let py = positions.get(pi * 3 + 1).copied().unwrap_or(0.0);
                                let pz = positions.get(pi * 3 + 2).copied().unwrap_or(0.0);
                                let [ox, oy, oz] = apply_up(px * sf[0], py * sf[1], pz * sf[2]);
                                all_vertices.push(ox);
                                all_vertices.push(oy);
                                all_vertices.push(oz);

                                if let (Some(ni_val), Some(norms)) = (ni, normals) {
                                    let nx = norms.get(ni_val * 3).copied().unwrap_or(0.0);
                                    let ny = norms.get(ni_val * 3 + 1).copied().unwrap_or(0.0);
                                    let nz = norms.get(ni_val * 3 + 2).copied().unwrap_or(0.0);
                                    let [onx, ony, onz] = apply_up(nx, ny, nz);
                                    all_vertices.push(onx);
                                    all_vertices.push(ony);
                                    all_vertices.push(onz);
                                } else {
                                    all_vertices.push(0.0);
                                    all_vertices.push(0.0);
                                    all_vertices.push(1.0);
                                }
                            }
                        }
                        idx_cursor += vc * stride;
                    }
                } else {
                    // <triangles>: every 3 * stride indices form one triangle.
                    let num_verts = indices.len() / stride;
                    for v in 0..num_verts {
                        let base = v * stride;
                        let pi = pos_offset.map(|o| indices[base + o]).unwrap_or(0);
                        let ni = norm_offset.map(|o| indices[base + o]);

                        let px = positions.get(pi * 3).copied().unwrap_or(0.0);
                        let py = positions.get(pi * 3 + 1).copied().unwrap_or(0.0);
                        let pz = positions.get(pi * 3 + 2).copied().unwrap_or(0.0);
                        let [ox, oy, oz] = apply_up(px * sf[0], py * sf[1], pz * sf[2]);
                        all_vertices.push(ox);
                        all_vertices.push(oy);
                        all_vertices.push(oz);

                        if let (Some(ni_val), Some(norms)) = (ni, normals) {
                            let nx = norms.get(ni_val * 3).copied().unwrap_or(0.0);
                            let ny = norms.get(ni_val * 3 + 1).copied().unwrap_or(0.0);
                            let nz = norms.get(ni_val * 3 + 2).copied().unwrap_or(0.0);
                            let [onx, ony, onz] = apply_up(nx, ny, nz);
                            all_vertices.push(onx);
                            all_vertices.push(ony);
                            all_vertices.push(onz);
                        } else {
                            all_vertices.push(0.0);
                            all_vertices.push(0.0);
                            all_vertices.push(1.0);
                        }
                    }
                }
            }
        }
    }

    // If no normals were provided at all, compute face normals.
    // (Check if every 6th float from index 3 is 0,0,1 default.)
    let tri_count = all_vertices.len() / 18;
    if tri_count > 0 {
        let all_default = (0..tri_count).all(|i| {
            let b = i * 18;
            all_vertices[b + 3] == 0.0
                && all_vertices[b + 4] == 0.0
                && all_vertices[b + 5] == 1.0
                && all_vertices[b + 9] == 0.0
                && all_vertices[b + 10] == 0.0
                && all_vertices[b + 11] == 1.0
                && all_vertices[b + 15] == 0.0
                && all_vertices[b + 16] == 0.0
                && all_vertices[b + 17] == 1.0
        });
        if all_default {
            // Recompute face normals from vertex positions.
            for i in 0..tri_count {
                let b = i * 18;
                let v0 = [all_vertices[b], all_vertices[b + 1], all_vertices[b + 2]];
                let v1 = [all_vertices[b + 6], all_vertices[b + 7], all_vertices[b + 8]];
                let v2 = [all_vertices[b + 12], all_vertices[b + 13], all_vertices[b + 14]];
                let e1 = [v1[0] - v0[0], v1[1] - v0[1], v1[2] - v0[2]];
                let e2 = [v2[0] - v0[0], v2[1] - v0[1], v2[2] - v0[2]];
                let nx = e1[1] * e2[2] - e1[2] * e2[1];
                let ny = e1[2] * e2[0] - e1[0] * e2[2];
                let nz = e1[0] * e2[1] - e1[1] * e2[0];
                let len = (nx * nx + ny * ny + nz * nz).sqrt().max(1e-12);
                let nn = [nx / len, ny / len, nz / len];
                for j in 0..3 {
                    let nb = b + j * 6 + 3;
                    all_vertices[nb] = nn[0];
                    all_vertices[nb + 1] = nn[1];
                    all_vertices[nb + 2] = nn[2];
                }
            }
        }
    }

    log::info!(
        "Loaded DAE {:?}: {} triangles",
        path.file_name().unwrap_or_default(),
        tri_count
    );
    all_vertices
}

/// Load a mesh file (STL or DAE) by extension, returning flat `[x,y,z,nx,ny,nz]` vertex data.
pub fn load_mesh_file(path: &std::path::Path) -> Vec<f32> {
    let ext = path
        .extension()
        .and_then(|e| e.to_str())
        .unwrap_or("")
        .to_lowercase();
    let pb = path.to_path_buf();
    match ext.as_str() {
        "stl" => load_stl_mesh(&pb, None),
        "obj" => load_obj_mesh(&pb, None),
        "dae" => load_dae_mesh(&pb, None),
        _ => {
            log::warn!("Unsupported mesh format: {:?}", path);
            Vec::new()
        }
    }
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
