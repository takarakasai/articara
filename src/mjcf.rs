//! MJCF (MuJoCo XML) import and export.
//!
//! MJCF uses a nested body hierarchy rather than flat link/joint lists.
//! We support a practical subset that maps to our RobotModel.

use nalgebra as na;
use std::collections::{HashMap, HashSet};
use std::path::Path;

use crate::robot::*;

// ========== Import ==========

/// Parse an MJCF file and return a RobotModel.
pub fn import_mjcf(path: &Path) -> Result<RobotModel, String> {
    let xml = std::fs::read_to_string(path).map_err(|e| format!("Read MJCF: {e}"))?;
    let doc = roxmltree::Document::parse(&xml).map_err(|e| format!("Parse MJCF XML: {e}"))?;

    let mjcf_dir = path.parent().unwrap_or(Path::new("."));

    let mujoco_el = doc
        .descendants()
        .find(|n| n.tag_name().name() == "mujoco")
        .ok_or("No <mujoco> element found")?;

    let robot_name = mujoco_el
        .attribute("model")
        .unwrap_or("mjcf_model")
        .to_string();

    // Check angle unit
    let angle_in_degrees = mujoco_el
        .descendants()
        .find(|n| n.tag_name().name() == "compiler")
        .and_then(|c| c.attribute("angle"))
        .map(|a| a == "degree")
        .unwrap_or(true); // MJCF default is degree

    // Collect mesh assets: name -> filename
    let mut mesh_assets: HashMap<String, String> = HashMap::new();
    if let Some(asset) = mujoco_el
        .children()
        .find(|n| n.tag_name().name() == "asset")
    {
        for mesh_el in asset.children().filter(|n| n.tag_name().name() == "mesh") {
            if let (Some(name), Some(file)) = (mesh_el.attribute("name"), mesh_el.attribute("file"))
            {
                mesh_assets.insert(name.to_string(), file.to_string());
            }
        }
    }

    let mut links = Vec::new();
    let mut link_map = HashMap::new();
    let mut joints = Vec::new();
    let mut joint_map = HashMap::new();
    let mut children_joints: HashMap<String, Vec<usize>> = HashMap::new();
    let mut child_links: HashSet<String> = HashSet::new();

    // Find <worldbody>
    let worldbody = mujoco_el
        .children()
        .find(|n| n.tag_name().name() == "worldbody")
        .ok_or("No <worldbody> element found")?;

    // Recursively parse bodies
    parse_mjcf_bodies(
        worldbody,
        None, // no parent
        mjcf_dir,
        &mesh_assets,
        angle_in_degrees,
        &mut links,
        &mut link_map,
        &mut joints,
        &mut joint_map,
        &mut children_joints,
        &mut child_links,
    );

    let root_link = links
        .iter()
        .find(|l| !child_links.contains(&l.name))
        .map(|l| l.name.clone())
        .unwrap_or_default();

    let joint_positions = vec![0.0_f64; joints.len()];

    // Parse top-level <equality> entries — both <joint> (mimic) and
    // <connect>/<weld> (closed-loop) variants populate the master-format
    // model so they round-trip through .misarta.toml.
    let mut mimics: Vec<crate::rbd::model::Mimic> = Vec::new();
    let mut loop_closures: Vec<crate::rbd::model::LoopClosure> = Vec::new();
    if let Some(eq) = mujoco_el.children().find(|n| n.tag_name().name() == "equality") {
        for je in eq.children().filter(|n| n.tag_name().name() == "joint") {
            let (Some(j1), Some(j2)) = (je.attribute("joint1"), je.attribute("joint2")) else {
                continue;
            };
            let mut multiplier = 1.0;
            let mut offset = 0.0;
            if let Some(poly) = je.attribute("polycoef") {
                let mut iter = poly
                    .split_whitespace()
                    .filter_map(|s| s.parse::<f64>().ok());
                offset = iter.next().unwrap_or(0.0);
                multiplier = iter.next().unwrap_or(1.0);
            }
            mimics.push(crate::rbd::model::Mimic {
                joint: j1.to_string(),
                source: j2.to_string(),
                multiplier,
                offset,
            });
        }
        // <connect body1=… body2=… anchor="x y z"> → 3-DoF loop closure.
        for ce in eq.children().filter(|n| n.tag_name().name() == "connect") {
            let (Some(b1), Some(b2)) = (ce.attribute("body1"), ce.attribute("body2")) else {
                continue;
            };
            let anchor = ce
                .attribute("anchor")
                .map(parse_vec3_text)
                .unwrap_or(na::Vector3::zeros());
            let name = ce
                .attribute("name")
                .unwrap_or(&format!("{b1}_{b2}_loop"))
                .to_string();
            loop_closures.push(crate::rbd::model::LoopClosure::position(
                name,
                b1.to_string(),
                anchor.cast::<f64>(),
                b2.to_string(),
                na::Vector3::zeros(),
            ));
        }
        // <weld body1=… body2=… relpose="x y z qw qx qy qz"> → 6-DoF.
        for we in eq.children().filter(|n| n.tag_name().name() == "weld") {
            let (Some(b1), Some(b2)) = (we.attribute("body1"), we.attribute("body2")) else {
                continue;
            };
            let mut t = na::Vector3::zeros();
            let mut q = na::UnitQuaternion::identity();
            if let Some(rp) = we.attribute("relpose") {
                let v: Vec<f64> = rp.split_whitespace().filter_map(|s| s.parse().ok()).collect();
                if v.len() >= 7 {
                    t = na::Vector3::new(v[0], v[1], v[2]);
                    q = na::UnitQuaternion::from_quaternion(na::Quaternion::new(
                        v[3], v[4], v[5], v[6],
                    ));
                }
            }
            let off_a = na::Isometry3::from_parts(na::Translation3::from(t), q);
            let name = we
                .attribute("name")
                .unwrap_or(&format!("{b1}_{b2}_weld"))
                .to_string();
            loop_closures.push(crate::rbd::model::LoopClosure::pose(
                name,
                b1.to_string(),
                off_a,
                b2.to_string(),
                na::Isometry3::identity(),
            ));
        }
    }

    // Parse top-level <sensor>. Map each known sub-element type to a master
    // Sensor; everything else stashes as Generic so the round-trip preserves it.
    let mut sensors: Vec<crate::rbd::model::Sensor> = Vec::new();
    if let Some(snode) = mujoco_el.children().find(|n| n.tag_name().name() == "sensor") {
        for el in snode.children().filter(|n| n.is_element()) {
            let kind_str = el.tag_name().name();
            let name = el.attribute("name").unwrap_or(kind_str).to_string();
            // The "site" attribute is more common than "body" for mounted
            // sensors; fall back to "body" / "joint" for the other types.
            let link = el
                .attribute("site")
                .or_else(|| el.attribute("body"))
                .or_else(|| el.attribute("objname"))
                .unwrap_or("")
                .to_string();
            let kind = match kind_str {
                "accelerometer" | "gyro" | "velocimeter" => {
                    crate::rbd::model::SensorKind::Imu {
                        gyro_noise: 0.0,
                        accel_noise: 0.0,
                    }
                }
                "touch" => crate::rbd::model::SensorKind::Contact { partner: None },
                "force" | "torque" | "jointactuatorfrc" | "force_torque" => {
                    crate::rbd::model::SensorKind::ForceTorque {
                        joint: el.attribute("joint").map(|s| s.to_string()),
                    }
                }
                _ => crate::rbd::model::SensorKind::Generic {
                    kind: kind_str.to_string(),
                    params: el
                        .attributes()
                        .map(|a| (a.name().to_string(), a.value().to_string()))
                        .collect(),
                },
            };
            sensors.push(crate::rbd::model::Sensor {
                name,
                link,
                origin: na::Isometry3::identity(),
                update_rate: 0.0,
                kind,
            });
        }
    }

    let mut model = RobotModel {
        name: robot_name,
        links,
        joints,
        link_map,
        joint_map,
        root_link,
        children_joints,
        materials: HashMap::new(),
        joint_positions,
        source_path: Some(path.to_path_buf()),
        base_transform: na::Isometry3::identity(),
        misarta_cache: None,
        loop_closures,
        poses: Vec::new(),
        collision_pairs: Vec::new(),
        sequences: Vec::new(),
        mimics,
        sensors,
    };
    model.rebuild_misarta_model();
    Ok(model)
}

fn parse_mjcf_bodies(
    parent_node: roxmltree::Node,
    parent_link_name: Option<&str>,
    mjcf_dir: &Path,
    mesh_assets: &HashMap<String, String>,
    angle_deg: bool,
    links: &mut Vec<LinkData>,
    link_map: &mut HashMap<String, usize>,
    joints: &mut Vec<JointData>,
    joint_map: &mut HashMap<String, usize>,
    children_joints: &mut HashMap<String, Vec<usize>>,
    child_links: &mut HashSet<String>,
) {
    for body_el in parent_node
        .children()
        .filter(|n| n.tag_name().name() == "body")
    {
        let body_name = body_el
            .attribute("name")
            .unwrap_or(&format!("body_{}", links.len()))
            .to_string();
        let body_pos = parse_pos_attr(body_el);
        let body_quat = parse_quat_attr(body_el);

        // Inertial
        let inertial = parse_mjcf_inertial(body_el);

        // Visuals: geom elements (MJCF uses <geom> for both visual and collision)
        let mut visuals = Vec::new();
        let mut collisions = Vec::new();
        for geom_el in body_el
            .children()
            .filter(|n| n.tag_name().name() == "geom")
        {
            let geom_data = parse_mjcf_geom(geom_el, mjcf_dir, mesh_assets);
            let gpos = parse_pos_attr(geom_el);
            let gquat = parse_quat_attr(geom_el);
            let origin = na::Isometry3::from_parts(
                na::Translation3::from(gpos),
                gquat,
            );

            // Color from rgba attribute
            let color = geom_el
                .attribute("rgba")
                .map(|s| {
                    let v: Vec<f32> = s
                        .split_whitespace()
                        .filter_map(|t| t.parse().ok())
                        .collect();
                    [
                        v.first().copied().unwrap_or(0.8),
                        v.get(1).copied().unwrap_or(0.8),
                        v.get(2).copied().unwrap_or(0.8),
                        v.get(3).copied().unwrap_or(1.0),
                    ]
                })
                .unwrap_or([0.8, 0.8, 0.8, 1.0]);

            visuals.push(VisualData {
                origin,
                geometry: geom_data.clone(),
                color,
            });
            collisions.push(CollisionData {
                origin,
                geometry: geom_data,
            });
        }

        let link_idx = links.len();
        link_map.insert(body_name.clone(), link_idx);
        links.push(LinkData {
            name: body_name.clone(),
            visuals,
            collisions,
            inertial,
        });

        // Joint(s) between parent and this body
        if let Some(parent_name) = parent_link_name {
            // Check for explicit <joint> elements
            let joint_els: Vec<_> = body_el
                .children()
                .filter(|n| n.tag_name().name() == "joint")
                .collect();

            if joint_els.is_empty() {
                // Create a fixed joint
                let ji = joints.len();
                let jname = format!("{}_fixed", body_name);
                joint_map.insert(jname.clone(), ji);
                children_joints
                    .entry(parent_name.to_string())
                    .or_default()
                    .push(ji);
                child_links.insert(body_name.clone());

                let origin = na::Isometry3::from_parts(
                    na::Translation3::from(body_pos),
                    body_quat,
                );
                joints.push(JointData {
                    name: jname,
                    joint_type: "fixed".into(),
                    parent_link: parent_name.to_string(),
                    child_link: body_name.clone(),
                    origin,
                    axis: na::Vector3::z(),
                    lower: 0.0,
                    upper: 0.0,
                    effort: 0.0,
                    velocity: 0.0,
                    actuator_mode: crate::rbd::model::ActuatorMode::default(),
                    actuator_kp: 50.0,
                    actuator_kv: 5.0,
                    armature: 0.0,
                    joint_damping: 0.0,
                });
            } else {
                for joint_el in joint_els {
                    let ji = joints.len();
                    let jname = joint_el
                        .attribute("name")
                        .unwrap_or(&format!("joint_{ji}"))
                        .to_string();

                    let jtype = match joint_el.attribute("type").unwrap_or("hinge") {
                        "hinge" => "revolute",
                        "slide" => "prismatic",
                        "ball" => "ball",
                        "free" => "free",
                        other => other,
                    }
                    .to_string();

                    let axis = joint_el
                        .attribute("axis")
                        .map(|s| parse_vec3_text(s))
                        .unwrap_or(na::Vector3::z());

                    let (lower, upper) = if let Some(range_str) = joint_el.attribute("range") {
                        let vals: Vec<f64> = range_str
                            .split_whitespace()
                            .filter_map(|s| s.parse().ok())
                            .collect();
                        let lo = vals.first().copied().unwrap_or(0.0);
                        let hi = vals.get(1).copied().unwrap_or(0.0);
                        if angle_deg && jtype == "revolute" {
                            (lo.to_radians(), hi.to_radians())
                        } else {
                            (lo, hi)
                        }
                    } else {
                        (0.0, 0.0)
                    };

                    let armature = joint_el
                        .attribute("armature")
                        .and_then(|s| s.parse::<f64>().ok())
                        .unwrap_or(0.0);
                    let joint_damping = joint_el
                        .attribute("damping")
                        .and_then(|s| s.parse::<f64>().ok())
                        .unwrap_or(0.0);

                    let origin = na::Isometry3::from_parts(
                        na::Translation3::from(body_pos),
                        body_quat,
                    );

                    joint_map.insert(jname.clone(), ji);
                    children_joints
                        .entry(parent_name.to_string())
                        .or_default()
                        .push(ji);
                    child_links.insert(body_name.clone());

                    joints.push(JointData {
                        name: jname,
                        joint_type: jtype,
                        parent_link: parent_name.to_string(),
                        child_link: body_name.clone(),
                        origin,
                        axis,
                        lower,
                        upper,
                        effort: 0.0,
                        velocity: 0.0,
                        actuator_mode: crate::rbd::model::ActuatorMode::default(),
                        actuator_kp: 50.0,
                        actuator_kv: 5.0,
                        armature,
                        joint_damping,
                    });
                }
            }
        }

        // Recurse into child bodies
        parse_mjcf_bodies(
            body_el,
            Some(&body_name),
            mjcf_dir,
            mesh_assets,
            angle_deg,
            links,
            link_map,
            joints,
            joint_map,
            children_joints,
            child_links,
        );
    }
}

// ========== Export ==========

/// World-frame ground plane to embed in exported MJCF (for MuJoCo sim).
#[derive(Clone, Copy, Debug)]
pub struct GroundPlaneCfg {
    /// Z height of the plane (world frame).
    pub z: f64,
    /// Half-extent (rendering hint; the plane is mathematically infinite).
    pub half_size: f64,
    /// Rotation about the X axis (radians).
    pub roll: f64,
    /// Rotation about the Y axis (radians).
    pub pitch: f64,
}

/// Options controlling how a [`RobotModel`] is exported to MJCF XML.
///
/// All fields are optional / defaulted, so [`MjcfExportOptions::default()`]
/// reproduces the legacy behaviour of [`export_mjcf`] (auto-lifted base, no
/// ground plane, no actuators).
#[derive(Default, Clone, Debug)]
pub struct MjcfExportOptions {
    /// Override for the floating-base world position. `None` = auto-lift so
    /// the lowest link sits just above z = 0.
    pub base_pos: Option<[f64; 3]>,
    /// Embed a collidable ground plane geom at the given configuration.
    pub ground_plane: Option<GroundPlaneCfg>,
    /// When true, emit `<motor>` actuators (named `motor_<joint>`) for each
    /// non-fixed joint so torques can be applied via `data.ctrl`.
    pub add_actuators: bool,
    /// Per-axis locks on the floating-base, ordered `[TX, TY, TZ, RX, RY, RZ]`.
    /// `true` = axis locked (no DoF), `false` = axis free.
    ///
    /// - All `false` → emit `<freejoint/>` (full 6-DoF base, the default)
    /// - All `true`  → emit no joint (base welded to the world at `base_pos`)
    /// - Mixed       → emit individual `<joint type="slide"/>` / `<hinge>`
    ///                 elements only for the unlocked axes
    pub base_locked_axes: [bool; 6],
}

/// Export a RobotModel to MJCF XML string with default options.
pub fn export_mjcf(model: &RobotModel) -> String {
    export_mjcf_with_options(model, MjcfExportOptions::default())
}

/// Full-configurability MJCF export.
pub fn export_mjcf_with_options(
    model: &RobotModel,
    opts: MjcfExportOptions,
) -> String {
    let MjcfExportOptions {
        base_pos,
        ground_plane,
        add_actuators,
        base_locked_axes,
    } = opts;
    let mut s = String::new();
    s.push_str(&format!(
        "<mujoco model=\"{}\">\n",
        model.name
    ));

    s.push_str("  <compiler angle=\"radian\"/>\n\n");

    // Resolve mesh URIs to absolute paths the same way the URDF loader does:
    // package_dir = urdf_dir.parent() (one level above the URDF file).
    let package_dir = model
        .source_path
        .as_ref()
        .and_then(|p| p.parent())   // urdf_dir
        .and_then(|d| d.parent());  // package_dir

    // Collect mesh assets from BOTH visuals and collisions. We previously
    // only walked `link.visuals`, but the MJCF export now emits collision
    // geoms separately (with `contype=1 conaffinity=1 group=3`) so the
    // physics engine uses the URDF's simplified collision shapes — and any
    // mesh-typed collision geom needs its asset registered too.
    let mut mesh_names: Vec<(String, String)> = Vec::new();
    let mut mesh_counter = 0usize;
    let mut geom_mesh_map: HashMap<*const GeomData, String> = HashMap::new();

    let resolve = |filename: &Option<String>| -> String {
        match (filename.as_deref(), package_dir.as_ref()) {
            (Some(uri), Some(pkg)) => crate::robot::resolve_package_path(uri, pkg)
                .to_string_lossy()
                .into_owned(),
            (Some(uri), None) => uri.to_string(),
            (None, _) => "mesh.stl".to_string(),
        }
    };

    for link in &model.links {
        for vis in &link.visuals {
            if let GeomData::Mesh { filename, .. } = &vis.geometry {
                let mesh_name = format!("mesh_{mesh_counter}");
                mesh_names.push((mesh_name.clone(), resolve(filename)));
                geom_mesh_map.insert(&vis.geometry as *const GeomData, mesh_name);
                mesh_counter += 1;
            }
        }
        for col in &link.collisions {
            if let GeomData::Mesh { filename, .. } = &col.geometry {
                let mesh_name = format!("mesh_{mesh_counter}");
                mesh_names.push((mesh_name.clone(), resolve(filename)));
                geom_mesh_map.insert(&col.geometry as *const GeomData, mesh_name);
                mesh_counter += 1;
            }
        }
    }

    if !mesh_names.is_empty() {
        s.push_str("  <asset>\n");
        for (name, file) in &mesh_names {
            s.push_str(&format!("    <mesh name=\"{name}\" file=\"{file}\"/>\n"));
        }
        s.push_str("  </asset>\n\n");
    }

    s.push_str("  <worldbody>\n");

    if let Some(gp) = ground_plane {
        s.push_str(&format!(
            "    <geom name=\"ground\" type=\"plane\" pos=\"0 0 {z}\" euler=\"{roll} {pitch} 0\" size=\"{hs} {hs} 0.1\" rgba=\"0.5 0.5 0.55 1\"/>\n",
            z = gp.z,
            roll = gp.roll,
            pitch = gp.pitch,
            hs = gp.half_size,
        ));
    }

    // Either honour the user-supplied base position or auto-lift the root so
    // the lowest link sits just above the ground plane.
    let root_pos = base_pos.unwrap_or_else(|| [0.0, 0.0, compute_initial_z(model)]);
    let base_spec = BaseSpec { pos: root_pos, locked: base_locked_axes };
    write_mjcf_body(&mut s, model, &model.root_link, 4, &geom_mesh_map, Some(base_spec));

    s.push_str("  </worldbody>\n");

    if add_actuators {
        write_mjcf_actuators(&mut s, model);
    }

    // Emit `<equality><joint>` for any mimic relationships and `<sensor>`
    // entries for the master-format sensor list. Both are MuJoCo-native
    // representations of articara's master Mimic / Sensor types.
    // `write_mjcf_equalities` also emits `<connect>` / `<weld>` for
    // closed-kinematic-loop constraints stored in `model.loop_closures`.
    write_mjcf_equalities(&mut s, model);
    write_mjcf_sensors(&mut s, model);

    // Emit `<contact><exclude>` blocks for any link pairs the user has
    // explicitly disabled in the collision-pair matrix. Pairs marked
    // `enabled = true` are no-ops in MuJoCo (collide-by-default) and are
    // skipped here.
    write_mjcf_contact_excludes(&mut s, model);

    s.push_str("</mujoco>\n");
    s
}

/// Emit `<equality>` block(s) covering both mimic relationships and closed
/// kinematic loops:
///
/// - `<joint joint1=… joint2=… polycoef="off mult 0 0 0">` per mimic
///   (linear coupling, polynomial: `target = off + mult·src + 0·src² + …`).
/// - `<connect body1=… body2=… anchor="x y z">` per 3-DoF loop closure
///   (anchor in body1's local frame; body2 is constrained at the same world
///   point as body1·anchor, with the corresponding body2-local point baked
///   in by MuJoCo at compile time from the bodies' rest poses).
/// - `<weld body1=… body2=… relpose="x y z qw qx qy qz">` per 6-DoF loop
///   closure (full pose constraint).
///
/// All entries that reference unknown bodies / joints are silently dropped
/// so a partial sidecar doesn't cause MuJoCo to refuse the file.
fn write_mjcf_equalities(s: &mut String, model: &RobotModel) {
    let active_mimics: Vec<&crate::rbd::model::Mimic> = model
        .mimics
        .iter()
        .filter(|m| {
            model.joint_map.contains_key(&m.joint)
                && model.joint_map.contains_key(&m.source)
        })
        .collect();
    let active_loops: Vec<&crate::rbd::model::LoopClosure> = model
        .loop_closures
        .iter()
        .filter(|lc| {
            model.link_map.contains_key(&lc.link_a)
                && model.link_map.contains_key(&lc.link_b)
        })
        .collect();
    if active_mimics.is_empty() && active_loops.is_empty() {
        return;
    }
    s.push_str("\n  <equality>\n");
    for m in active_mimics {
        s.push_str(&format!(
            "    <joint name=\"mimic_{}\" joint1=\"{}\" joint2=\"{}\" polycoef=\"{} {} 0 0 0\"/>\n",
            m.joint, m.joint, m.source, m.offset, m.multiplier,
        ));
    }
    for lc in active_loops {
        let oa = lc.offset_a.translation.vector;
        if lc.pose_6dof {
            // 6-DoF: weld with full relative pose. relpose = b2 in b1's
            // frame at the constraint instant. We compute it from the
            // user-set offsets: B-frame seen from A is offset_a · offset_b⁻¹.
            let rel = lc.offset_a * lc.offset_b.inverse();
            let rt = rel.translation.vector;
            let rq = rel.rotation.quaternion();
            s.push_str(&format!(
                "    <weld name=\"{}\" body1=\"{}\" body2=\"{}\" relpose=\"{} {} {} {} {} {} {}\"/>\n",
                lc.name, lc.link_a, lc.link_b,
                rt.x, rt.y, rt.z, rq.w, rq.i, rq.j, rq.k,
            ));
        } else {
            // 3-DoF: connect at offset_a in link_a's local frame. MuJoCo
            // resolves the corresponding point on link_b from the bodies'
            // rest configuration at compile time.
            s.push_str(&format!(
                "    <connect name=\"{}\" body1=\"{}\" body2=\"{}\" anchor=\"{} {} {}\"/>\n",
                lc.name, lc.link_a, lc.link_b, oa.x, oa.y, oa.z,
            ));
        }
    }
    s.push_str("  </equality>\n");
}

/// Emit `<sensor>` entries. MuJoCo's sensor model is rich; we map our
/// core kinds to the closest native types and fall back to a comment for
/// types that don't have a direct equivalent.
fn write_mjcf_sensors(s: &mut String, model: &RobotModel) {
    if model.sensors.is_empty() {
        return;
    }
    // We need a `<site>` per sensor for those that mount on a site
    // (force-torque, accelerometer, etc.); for v0 we attach to the
    // body's frame via `objtype="body" objname=<link>`, which works for
    // most sensor types in modern MuJoCo.
    s.push_str("\n  <sensor>\n");
    for sensor in &model.sensors {
        match &sensor.kind {
            crate::rbd::model::SensorKind::Imu { .. } => {
                s.push_str(&format!(
                    "    <accelerometer name=\"{}_accel\" site=\"{}\"/>\n",
                    sensor.name, sensor.link,
                ));
                s.push_str(&format!(
                    "    <gyro name=\"{}_gyro\" site=\"{}\"/>\n",
                    sensor.name, sensor.link,
                ));
            }
            crate::rbd::model::SensorKind::ForceTorque { joint } => {
                if let Some(j) = joint {
                    s.push_str(&format!(
                        "    <jointactuatorfrc name=\"{}\" joint=\"{}\"/>\n",
                        sensor.name, j,
                    ));
                } else {
                    s.push_str(&format!(
                        "    <!-- force_torque '{}' on link '{}' (no joint specified) -->\n",
                        sensor.name, sensor.link,
                    ));
                }
            }
            crate::rbd::model::SensorKind::Contact { .. } => {
                s.push_str(&format!(
                    "    <touch name=\"{}\" site=\"{}\"/>\n",
                    sensor.name, sensor.link,
                ));
            }
            // Camera / Lidar / Generic don't have direct MJCF sensor
            // counterparts; emit comments so users can wire them up
            // manually if needed without losing the master record.
            crate::rbd::model::SensorKind::Camera { .. } => {
                s.push_str(&format!(
                    "    <!-- camera '{}' on link '{}' — use <camera> element manually -->\n",
                    sensor.name, sensor.link,
                ));
            }
            crate::rbd::model::SensorKind::Lidar { .. } => {
                s.push_str(&format!(
                    "    <!-- lidar '{}' on link '{}' — MuJoCo has no native ray sensor -->\n",
                    sensor.name, sensor.link,
                ));
            }
            crate::rbd::model::SensorKind::Generic { kind, .. } => {
                s.push_str(&format!(
                    "    <!-- generic sensor '{}' (kind='{}') on link '{}' -->\n",
                    sensor.name, kind, sensor.link,
                ));
            }
        }
    }
    s.push_str("  </sensor>\n");
}

/// Emit `<contact><exclude>` for every collision pair the user has marked
/// as disabled. MuJoCo's default is "all geoms collide", so we only need to
/// emit excludes — `enabled = true` pairs are implicit.
fn write_mjcf_contact_excludes(s: &mut String, model: &RobotModel) {
    let excluded: Vec<&crate::rbd::model::CollisionPair> = model
        .collision_pairs
        .iter()
        .filter(|p| !p.enabled)
        .filter(|p| {
            // Only emit pairs where both links exist in the model — silently
            // dropping orphans avoids confusing MuJoCo errors.
            model.link_map.contains_key(&p.link_a)
                && model.link_map.contains_key(&p.link_b)
        })
        .collect();
    if excluded.is_empty() {
        return;
    }
    s.push_str("\n  <contact>\n");
    for p in excluded {
        s.push_str(&format!(
            "    <exclude body1=\"{}\" body2=\"{}\"/>\n",
            p.link_a, p.link_b,
        ));
    }
    s.push_str("  </contact>\n");
}

/// Emit one `<motor>` actuator per non-fixed joint, named `motor_<joint>`.
///
/// The motor's `gear` is 1, so writing the joint torque directly into
/// `data.ctrl[motor_id]` applies that torque. `ctrlrange` mirrors the URDF
/// `effort` limit when present; joints without an effort limit get an
/// unbounded ctrl range.
/// Emit one `<motor>` actuator per non-fixed joint. The host application
/// computes per-step torque commands externally based on each joint's
/// [`ActuatorMode`] / `kp` / `kv` (held in [`JointData`]). The MJCF itself
/// is always plain torque-mode so the same file can be used for any control
/// strategy when re-imported elsewhere.
fn write_mjcf_actuators(s: &mut String, model: &RobotModel) {
    let movable: Vec<&JointData> = model
        .joints
        .iter()
        .filter(|j| j.joint_type != "fixed")
        .collect();
    if movable.is_empty() {
        return;
    }
    s.push_str("\n  <actuator>\n");
    for joint in movable {
        let force_attrs = if joint.effort > 0.0 {
            format!(
                " forcelimited=\"true\" forcerange=\"{} {}\"",
                -joint.effort, joint.effort,
            )
        } else {
            String::new()
        };
        s.push_str(&format!(
            "    <motor name=\"motor_{name}\" joint=\"{name}\" gear=\"1\"{force_attrs}/>\n",
            name = joint.name,
        ));
    }
    s.push_str("  </actuator>\n");
}

/// Computes the minimum cumulative z translation in the kinematic chain
/// (all joints at zero). Returns how much to lift the root so the lowest
/// link is just above z = 0.
fn compute_initial_z(model: &RobotModel) -> f64 {
    fn min_z_recursive(model: &RobotModel, link: &str, z: f64) -> f64 {
        let mut min = z;
        if let Some(children) = model.children_joints.get(link) {
            for &ji in children {
                let dz = model.joints[ji].origin.translation.z as f64;
                min = min.min(min_z_recursive(model, &model.joints[ji].child_link, z + dz));
            }
        }
        min
    }
    let min_z = min_z_recursive(model, &model.root_link, 0.0);
    if min_z < 0.0 { -min_z + 0.01 } else { 0.01 }
}

/// World-frame placement + per-axis lock state for the root body.
#[derive(Clone, Copy)]
struct BaseSpec {
    pos: [f64; 3],
    /// `[TX, TY, TZ, RX, RY, RZ]`: `true` = locked (no DoF).
    locked: [bool; 6],
}

fn write_mjcf_body(
    s: &mut String,
    model: &RobotModel,
    link_name: &str,
    indent: usize,
    geom_mesh_map: &HashMap<*const GeomData, String>,
    base_spec: Option<BaseSpec>,
) {
    let pad: String = " ".repeat(indent);

    let link_idx = match model.link_map.get(link_name) {
        Some(&i) => i,
        None => return,
    };
    let link = &model.links[link_idx];

    // Find the joint connecting this link to its parent for pose
    let (pos_str, joint_info) = if let Some(spec) = base_spec {
        let [x, y, z] = spec.pos;
        // Root body: place at user-specified or auto-lifted world position
        (format!("{x} {y} {z}"), None)
    } else if let Some(ji) = model.parent_joint_of_link(link_name) {
        let joint = &model.joints[ji];
        let t = &joint.origin.translation;
        (format!("{} {} {}", t.x, t.y, t.z), Some(joint))
    } else {
        ("0 0 0".into(), None)
    };

    s.push_str(&format!("{pad}<body name=\"{link_name}\" pos=\"{pos_str}\">\n"));

    // Root body: emit floating-base joints based on the per-axis lock state.
    if let Some(spec) = base_spec {
        let any_free = spec.locked.iter().any(|&l| !l);
        let all_free = spec.locked.iter().all(|&l| !l);
        if all_free {
            // Cleanest 6-DoF representation; avoids the gimbal singularity
            // of stacking three hinge joints for orientation.
            s.push_str(&format!("{pad}  <freejoint/>\n"));
        } else if any_free {
            // Partial constraint: emit only the unlocked axes as individual
            // slide / hinge joints. Translations first, then rotations, so
            // the kinematic chain is intuitive (X→Y→Z→roll→pitch→yaw).
            const AXES: [(&str, &str, &str); 6] = [
                ("base_tx", "slide", "1 0 0"),
                ("base_ty", "slide", "0 1 0"),
                ("base_tz", "slide", "0 0 1"),
                ("base_rx", "hinge", "1 0 0"),
                ("base_ry", "hinge", "0 1 0"),
                ("base_rz", "hinge", "0 0 1"),
            ];
            for (i, (jname, jtype, axis)) in AXES.iter().enumerate() {
                if !spec.locked[i] {
                    s.push_str(&format!(
                        "{pad}  <joint name=\"{jname}\" type=\"{jtype}\" axis=\"{axis}\"/>\n",
                    ));
                }
            }
        }
        // else: all 6 locked → no joint emitted; body welds to world at `pos`.
    }

    // Inertial
    if link.inertial.mass > 1e-12 {
        let it = &link.inertial.origin.translation;
        s.push_str(&format!(
            "{pad}  <inertial mass=\"{}\" pos=\"{} {} {}\" diaginertia=\"{} {} {}\"/>\n",
            link.inertial.mass,
            it.x,
            it.y,
            it.z,
            link.inertial.ixx,
            link.inertial.iyy,
            link.inertial.izz
        ));
    }

    // Joint
    if let Some(joint) = joint_info {
        if joint.joint_type != "fixed" {
            let mjcf_type = match joint.joint_type.as_str() {
                "revolute" | "continuous" => "hinge",
                "prismatic" => "slide",
                other => other,
            };
            s.push_str(&format!(
                "{pad}  <joint name=\"{}\" type=\"{mjcf_type}\" axis=\"{} {} {}\"",
                joint.name, joint.axis.x, joint.axis.y, joint.axis.z
            ));
            if joint.lower < joint.upper {
                s.push_str(&format!(
                    " range=\"{} {}\"",
                    joint.lower, joint.upper
                ));
            }
            // Emit rotor inertia + passive damping when set. Both default to
            // 0; positive values stabilise the external PD controller and
            // soak up landing impacts. Mapped 1:1 to MuJoCo's joint attrs.
            if joint.armature > 0.0 {
                s.push_str(&format!(" armature=\"{}\"", joint.armature));
            }
            if joint.joint_damping > 0.0 {
                s.push_str(&format!(" damping=\"{}\"", joint.joint_damping));
            }
            s.push_str("/>\n");
        }
    }

    // Geoms — emit visuals and collisions as separate <geom> elements.
    //
    // MuJoCo doesn't have a built-in visual/collision split, so we use the
    // standard contype/conaffinity/group convention:
    //   • visual    → contype=0  conaffinity=0  group=1   (no physics, render only)
    //   • collision → contype=1  conaffinity=1  group=3   (physics, optionally hidden)
    //
    // The viewer can show either by toggling group bits, the physics engine
    // only ever resolves the collision geoms (which are normally low-poly
    // primitives in URDF and therefore avoid the visual mesh's intentional
    // joint-boundary overlaps that were producing ~70 N spurious self-
    // collision penalties pre-fix).
    let visual_extra = " contype=\"0\" conaffinity=\"0\" group=\"1\"";
    let collision_extra = " contype=\"1\" conaffinity=\"1\" group=\"3\"";

    // Visuals (rendering only).
    for vis in &link.visuals {
        let t = &vis.origin.translation;
        let pos_attr = format!("{} {} {}", t.x, t.y, t.z);
        let rgba = format!(
            "{} {} {} {}",
            vis.color[0], vis.color[1], vis.color[2], vis.color[3]
        );
        match &vis.geometry {
            GeomData::Box { hx, hy, hz } => {
                s.push_str(&format!(
                    "{pad}  <geom type=\"box\" pos=\"{pos_attr}\" size=\"{hx} {hy} {hz}\" rgba=\"{rgba}\"{visual_extra}/>\n",
                ));
            }
            GeomData::Cylinder { radius, half_length } => {
                s.push_str(&format!(
                    "{pad}  <geom type=\"cylinder\" pos=\"{pos_attr}\" size=\"{radius} {half_length}\" rgba=\"{rgba}\"{visual_extra}/>\n",
                ));
            }
            GeomData::Sphere { radius } => {
                s.push_str(&format!(
                    "{pad}  <geom type=\"sphere\" pos=\"{pos_attr}\" size=\"{radius}\" rgba=\"{rgba}\"{visual_extra}/>\n",
                ));
            }
            GeomData::Capsule { radius, half_length } => {
                s.push_str(&format!(
                    "{pad}  <geom type=\"capsule\" pos=\"{pos_attr}\" size=\"{radius} {half_length}\" rgba=\"{rgba}\"{visual_extra}/>\n",
                ));
            }
            GeomData::Mesh { .. } => {
                let ptr = &vis.geometry as *const GeomData;
                if let Some(mesh_name) = geom_mesh_map.get(&ptr) {
                    s.push_str(&format!(
                        "{pad}  <geom type=\"mesh\" mesh=\"{mesh_name}\" pos=\"{pos_attr}\" rgba=\"{rgba}\"{visual_extra}/>\n",
                    ));
                }
            }
        }
    }

    // Collisions (physics).
    //
    // We give them a faint translucent green so the user can still inspect
    // them when they enable the group=3 bit in the viewer; the physics
    // engine ignores the rgba attribute.
    let col_rgba = "0.4 0.85 0.4 0.25";
    for col in &link.collisions {
        let t = &col.origin.translation;
        let pos_attr = format!("{} {} {}", t.x, t.y, t.z);
        match &col.geometry {
            GeomData::Box { hx, hy, hz } => {
                s.push_str(&format!(
                    "{pad}  <geom type=\"box\" pos=\"{pos_attr}\" size=\"{hx} {hy} {hz}\" rgba=\"{col_rgba}\"{collision_extra}/>\n",
                ));
            }
            GeomData::Cylinder { radius, half_length } => {
                s.push_str(&format!(
                    "{pad}  <geom type=\"cylinder\" pos=\"{pos_attr}\" size=\"{radius} {half_length}\" rgba=\"{col_rgba}\"{collision_extra}/>\n",
                ));
            }
            GeomData::Sphere { radius } => {
                s.push_str(&format!(
                    "{pad}  <geom type=\"sphere\" pos=\"{pos_attr}\" size=\"{radius}\" rgba=\"{col_rgba}\"{collision_extra}/>\n",
                ));
            }
            GeomData::Capsule { radius, half_length } => {
                s.push_str(&format!(
                    "{pad}  <geom type=\"capsule\" pos=\"{pos_attr}\" size=\"{radius} {half_length}\" rgba=\"{col_rgba}\"{collision_extra}/>\n",
                ));
            }
            GeomData::Mesh { .. } => {
                let ptr = &col.geometry as *const GeomData;
                if let Some(mesh_name) = geom_mesh_map.get(&ptr) {
                    s.push_str(&format!(
                        "{pad}  <geom type=\"mesh\" mesh=\"{mesh_name}\" pos=\"{pos_attr}\" rgba=\"{col_rgba}\"{collision_extra}/>\n",
                    ));
                }
            }
        }
    }

    // Recurse children
    if let Some(child_joints) = model.children_joints.get(link_name) {
        for &ji in child_joints {
            let child_link = &model.joints[ji].child_link;
            write_mjcf_body(s, model, child_link, indent + 2, geom_mesh_map, None);
        }
    }

    s.push_str(&format!("{pad}</body>\n"));
}

// ========== Helpers ==========

fn parse_pos_attr(node: roxmltree::Node) -> na::Vector3<f32> {
    node.attribute("pos")
        .map(|s| parse_vec3_text(s))
        .unwrap_or(na::Vector3::zeros())
}

fn parse_quat_attr(node: roxmltree::Node) -> na::UnitQuaternion<f32> {
    if let Some(quat_str) = node.attribute("quat") {
        let vals: Vec<f32> = quat_str
            .split_whitespace()
            .filter_map(|s| s.parse().ok())
            .collect();
        if vals.len() >= 4 {
            // MJCF: w x y z order
            return na::UnitQuaternion::from_quaternion(na::Quaternion::new(
                vals[0], vals[1], vals[2], vals[3],
            ));
        }
    }
    if let Some(euler_str) = node.attribute("euler") {
        let v = parse_vec3_text(euler_str);
        return na::UnitQuaternion::from_euler_angles(v.x, v.y, v.z);
    }
    na::UnitQuaternion::identity()
}

fn parse_mjcf_inertial(body_el: roxmltree::Node) -> InertialData {
    if let Some(i) = body_el
        .children()
        .find(|n| n.tag_name().name() == "inertial")
    {
        let mass = i
            .attribute("mass")
            .and_then(|s| s.parse().ok())
            .unwrap_or(0.0);
        let pos = parse_pos_attr(i);
        let origin = na::Isometry3::from_parts(
            na::Translation3::from(pos),
            parse_quat_attr(i),
        );

        // diaginertia or fullinertia
        let (ixx, ixy, ixz, iyy, iyz, izz) =
            if let Some(diag) = i.attribute("diaginertia") {
                let v: Vec<f64> = diag
                    .split_whitespace()
                    .filter_map(|s| s.parse().ok())
                    .collect();
                (
                    v.first().copied().unwrap_or(0.0),
                    0.0,
                    0.0,
                    v.get(1).copied().unwrap_or(0.0),
                    0.0,
                    v.get(2).copied().unwrap_or(0.0),
                )
            } else if let Some(full) = i.attribute("fullinertia") {
                let v: Vec<f64> = full
                    .split_whitespace()
                    .filter_map(|s| s.parse().ok())
                    .collect();
                (
                    v.first().copied().unwrap_or(0.0),
                    v.get(1).copied().unwrap_or(0.0),
                    v.get(2).copied().unwrap_or(0.0),
                    v.get(3).copied().unwrap_or(0.0),
                    v.get(4).copied().unwrap_or(0.0),
                    v.get(5).copied().unwrap_or(0.0),
                )
            } else {
                (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            };

        InertialData {
            origin,
            mass,
            ixx,
            ixy,
            ixz,
            iyy,
            iyz,
            izz,
        }
    } else {
        InertialData {
            origin: na::Isometry3::identity(),
            mass: 0.0,
            ixx: 0.0,
            ixy: 0.0,
            ixz: 0.0,
            iyy: 0.0,
            iyz: 0.0,
            izz: 0.0,
        }
    }
}

fn parse_mjcf_geom(
    geom_el: roxmltree::Node,
    mjcf_dir: &Path,
    mesh_assets: &HashMap<String, String>,
) -> GeomData {
    let geom_type = geom_el.attribute("type").unwrap_or("sphere");
    let size: Vec<f32> = geom_el
        .attribute("size")
        .unwrap_or("")
        .split_whitespace()
        .filter_map(|s| s.parse().ok())
        .collect();

    match geom_type {
        "box" => {
            let hx = size.first().copied().unwrap_or(0.05);
            let hy = size.get(1).copied().unwrap_or(hx);
            let hz = size.get(2).copied().unwrap_or(hy);
            GeomData::Box { hx, hy, hz }
        }
        "cylinder" => {
            let radius = size.first().copied().unwrap_or(0.05);
            let half_length = size.get(1).copied().unwrap_or(0.1);
            GeomData::Cylinder {
                radius,
                half_length,
            }
        }
        "capsule" => {
            let radius = size.first().copied().unwrap_or(0.05);
            let half_length = size.get(1).copied().unwrap_or(0.1);
            GeomData::Capsule {
                radius,
                half_length,
            }
        }
        "sphere" => {
            let radius = size.first().copied().unwrap_or(0.05);
            GeomData::Sphere { radius }
        }
        "mesh" => {
            if let Some(mesh_name) = geom_el.attribute("mesh") {
                let filename = mesh_assets
                    .get(mesh_name)
                    .cloned()
                    .unwrap_or_else(|| format!("{mesh_name}.stl"));
                let mesh_path = mjcf_dir.join(&filename);
                let vertices = crate::robot::load_stl_mesh_public(&mesh_path);
                GeomData::Mesh {
                    vertices,
                    filename: Some(filename),
                    scale: None,
                }
            } else {
                GeomData::Box {
                    hx: 0.01,
                    hy: 0.01,
                    hz: 0.01,
                }
            }
        }
        _ => GeomData::Sphere {
            radius: size.first().copied().unwrap_or(0.05),
        },
    }
}

fn parse_vec3_text(text: &str) -> na::Vector3<f32> {
    let vals: Vec<f32> = text
        .split_whitespace()
        .filter_map(|s| s.parse().ok())
        .collect();
    na::Vector3::new(
        vals.first().copied().unwrap_or(0.0),
        vals.get(1).copied().unwrap_or(0.0),
        vals.get(2).copied().unwrap_or(0.0),
    )
}

// GeomData now derives Clone in rbd::model.
