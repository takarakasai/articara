//! MJCF (MuJoCo XML) import and export — articara layer.
//!
//! MJCF uses a nested body hierarchy rather than flat link/joint lists.
//! This module produces / consumes [`crate::robot::RobotModel`] directly
//! and includes articara-specific handling: actuators with mode/kp/kv,
//! `<equality>` constraints, sensors, and `[[contact]]` exclusion lists.
//!
//! # Layering note
//!
//! There is no `misarta::mjcf` parallel layer at the moment — unlike URDF
//! and SDF, the MJCF importer lives only in articara. Splitting it into
//! a "structural-only" misarta layer (returning `Model<f64>` +
//! `GeometryModel`) plus an articara wrapper is tracked in
//! `doc/refactor_20260502.md` §9.1 as future work. The parser is ~1300
//! lines and tightly coupled to `RobotModel`'s extension fields, so a
//! safe split needs a dedicated change with full round-trip regression.

use nalgebra as na;
use std::collections::{HashMap, HashSet};
use std::path::Path;

use crate::robot::*;

// ========== Import ==========

/// Flattened `<default>` class table.
///
/// Outer key: class name (the unnamed top-level `<default>` becomes
/// `"main"`). Inner: element tag (e.g. `joint`, `geom`, `motor`,
/// `position`) → attribute name → value. Each class's map already has
/// its parent class's defaults merged in, so a single lookup gives the
/// fully resolved attribute.
type MjcfClassTable = HashMap<String, HashMap<String, HashMap<String, String>>>;

/// Resolve `attr` on `el`, falling back to the class system.
///
/// MuJoCo's resolution order is:
/// 1. explicit attribute on the element
/// 2. element's own `class="X"` defaults
/// 3. the most-recent ancestor `<body>`'s `childclass`
/// 4. the unnamed top-level `<default>` (`"main"`)
fn mjcf_attr<'a>(
    el: roxmltree::Node<'a, 'a>,
    tag: &str,
    attr: &str,
    body_childclass: &str,
    table: &MjcfClassTable,
) -> Option<String> {
    if let Some(v) = el.attribute(attr) {
        return Some(v.to_string());
    }
    // Inheritance fallback: class on the element wins over the body's
    // childclass; both fall back to "main".
    let candidates = [el.attribute("class").unwrap_or(""), body_childclass, "main"];
    for cls in candidates.iter().filter(|c| !c.is_empty()) {
        if let Some(v) = table
            .get(*cls)
            .and_then(|tags| tags.get(tag))
            .and_then(|attrs| attrs.get(attr))
        {
            return Some(v.clone());
        }
    }
    None
}

/// Walk a `<default>` element, merging its declared element defaults
/// onto `parent_class`'s already-flattened defaults, then recurse into
/// any nested `<default class="X">`.
fn walk_mjcf_default(
    el: roxmltree::Node,
    parent_class: Option<&str>,
    table: &mut MjcfClassTable,
) {
    // The outermost <default> is unnamed and becomes "main"; named
    // siblings/children carry their own class= attribute.
    let this_class = el
        .attribute("class")
        .unwrap_or("main")
        .to_string();

    // Start from the parent class's flattened map (deep clone) so each
    // class can be queried in one step.
    let mut my_defaults: HashMap<String, HashMap<String, String>> = parent_class
        .and_then(|pc| table.get(pc).cloned())
        .unwrap_or_default();

    for child in el.children().filter(|n| n.is_element()) {
        let tag = child.tag_name().name();
        if tag == "default" {
            // Nested class — handled in the recursion below.
            continue;
        }
        let entry = my_defaults.entry(tag.to_string()).or_default();
        for attr in child.attributes() {
            // Skip `class` itself — it's metadata, not a per-element default.
            if attr.name() == "class" {
                continue;
            }
            entry.insert(attr.name().to_string(), attr.value().to_string());
        }
    }

    table.insert(this_class.clone(), my_defaults);

    for child in el.children().filter(|n| n.tag_name().name() == "default") {
        walk_mjcf_default(child, Some(&this_class), table);
    }
}

/// Build the full class table for a `<mujoco>` element.
fn parse_mjcf_class_table(mujoco_el: roxmltree::Node) -> MjcfClassTable {
    let mut table: MjcfClassTable = HashMap::new();
    // The root <default> is optional; if missing, "main" is just empty.
    table.insert("main".to_string(), HashMap::new());
    for top in mujoco_el
        .children()
        .filter(|n| n.tag_name().name() == "default")
    {
        walk_mjcf_default(top, None, &mut table);
    }
    table
}

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
    let compiler_el = mujoco_el
        .descendants()
        .find(|n| n.tag_name().name() == "compiler");
    let angle_in_degrees = compiler_el
        .and_then(|c| c.attribute("angle"))
        .map(|a| a == "degree")
        .unwrap_or(true); // MJCF default is degree

    // Collect mesh assets: name -> filename
    let mut mesh_assets: HashMap<String, String> = HashMap::new();
    if let Some(asset) = mujoco_el
        .children()
        .find(|n| n.tag_name().name() == "asset")
    {
        // Compose `meshdir` (read above) onto every asset path here so the
        // resulting URI resolves correctly both at load time *and* later
        // through `crate::mesh_paths::resolve_source` (= MJCF re-export
        // for in-process MuJoCo). Storing just the bare `file=` value
        // strips the meshdir and breaks the in-process sim with
        // "Error opening file ..." because MuJoCo looks beside the MJCF,
        // not in `assets/`.
        let meshdir_rel = compiler_el.and_then(|c| c.attribute("meshdir"));
        for mesh_el in asset.children().filter(|n| n.tag_name().name() == "mesh") {
            // MJCF allows `<mesh file="foo.obj"/>` with no explicit `name=`;
            // in that case the asset name is the file's basename without
            // its extension. Menagerie (e.g. Unitree Go2) relies on this,
            // so a strict `name && file` requirement silently drops every
            // mesh in those files.
            let Some(file) = mesh_el.attribute("file") else {
                continue;
            };
            let name = mesh_el.attribute("name").map(str::to_string).unwrap_or_else(|| {
                Path::new(file)
                    .file_stem()
                    .and_then(|s| s.to_str())
                    .unwrap_or(file)
                    .to_string()
            });
            // Skip the meshdir prefix when the file= is already absolute
            // or already includes the meshdir — keep the import idempotent
            // against author MJCFs that write either form.
            let file_path = Path::new(file);
            let stored = if file_path.is_absolute() {
                file.to_string()
            } else if let Some(md) = meshdir_rel {
                let combined = Path::new(md).join(file);
                combined.to_string_lossy().into_owned()
            } else {
                file.to_string()
            };
            mesh_assets.insert(name, stored);
        }
    }

    let mut links = Vec::new();
    let mut link_map = HashMap::new();
    let mut joints = Vec::new();
    let mut joint_map = HashMap::new();
    let mut children_joints: HashMap<String, Vec<usize>> = HashMap::new();
    let mut child_links: HashSet<String> = HashSet::new();

    // Build the <default> class table once before walking bodies so each
    // `<joint>` / `<geom>` / `<motor>` etc. element can fall back to its
    // class's (and inherited classes') attributes. Without this every
    // joint axis / range / damping silently collapses to MJCF's hard-coded
    // defaults — see the Unitree Go2 menagerie model, which declares all
    // joint axes in `<default class="abduction|hip|knee">` blocks.
    let class_table = parse_mjcf_class_table(mujoco_el);

    // Find <worldbody>
    let worldbody = mujoco_el
        .children()
        .find(|n| n.tag_name().name() == "worldbody")
        .ok_or("No <worldbody> element found")?;

    // Recursively parse bodies
    parse_mjcf_bodies(
        worldbody,
        None, // no parent
        "main", // starting childclass
        &class_table,
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

    // Parse <actuator> after bodies so joint refs exist. Motor / position /
    // velocity / general elements set the corresponding `ActuatorMode` and
    // pick up Kp / Kv / effort (force range) from the class chain.
    parse_mjcf_actuators_import(mujoco_el, &class_table, &mut joints, &joint_map);

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
        gaits: Vec::new(),
    };
    model.rebuild_misarta_model();
    Ok(model)
}

fn parse_mjcf_bodies(
    parent_node: roxmltree::Node,
    parent_link_name: Option<&str>,
    parent_childclass: &str,
    class_table: &MjcfClassTable,
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
        // `childclass` propagates: a body's childclass becomes the default
        // class for all elements inside it (and its descendants) until
        // another body overrides it.
        let body_childclass = body_el
            .attribute("childclass")
            .unwrap_or(parent_childclass)
            .to_string();

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
            let geom_data =
                parse_mjcf_geom(geom_el, mjcf_dir, mesh_assets, &body_childclass, class_table);
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
            
                physics: None,
            });
        }

        let link_idx = links.len();
        link_map.insert(body_name.clone(), link_idx);
        links.push(LinkData {
            name: body_name.clone(),
            visuals,
            collisions,
            inertial,
            collision_enabled: true,
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

                    // `type` itself can be set in <default><joint type="..."/>
                    // so resolve through the class table too. Default is
                    // "hinge" per MJCF.
                    let jtype_raw = mjcf_attr(joint_el, "joint", "type", &body_childclass, class_table)
                        .unwrap_or_else(|| "hinge".to_string());
                    let jtype = match jtype_raw.as_str() {
                        "hinge" => "revolute",
                        "slide" => "prismatic",
                        "ball" => "ball",
                        "free" => "free",
                        other => other,
                    }
                    .to_string();

                    let axis = mjcf_attr(joint_el, "joint", "axis", &body_childclass, class_table)
                        .as_deref()
                        .map(parse_vec3_text)
                        .unwrap_or(na::Vector3::z());

                    let (lower, upper) = if let Some(range_str) =
                        mjcf_attr(joint_el, "joint", "range", &body_childclass, class_table)
                    {
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

                    let armature =
                        mjcf_attr(joint_el, "joint", "armature", &body_childclass, class_table)
                            .and_then(|s| s.parse::<f64>().ok())
                            .unwrap_or(0.0);
                    let joint_damping =
                        mjcf_attr(joint_el, "joint", "damping", &body_childclass, class_table)
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

        // Recurse into child bodies — propagate this body's effective
        // childclass so nested joints/geoms keep inheriting the right
        // class chain.
        parse_mjcf_bodies(
            body_el,
            Some(&body_name),
            &body_childclass,
            class_table,
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

/// Parse the top-level `<actuator>` block and fold per-element settings
/// (mode, Kp / Kv, effort range) onto the already-loaded `joints`.
///
/// MJCF maps actuator element tags to articara modes:
/// - `<motor>`        → [`crate::rbd::model::ActuatorMode::Torque`]
/// - `<position>`     → [`crate::rbd::model::ActuatorMode::Position`]
/// - `<velocity>`     → [`crate::rbd::model::ActuatorMode::Velocity`]
/// - `<general>`      → defaults to Torque unless `dyntype`/`gaintype` etc.
///   force a different interpretation; the import is best-effort.
///
/// `ctrlrange` / `forcerange` populate `joint.effort` (taken as the
/// symmetric max). Inheritance is via the `class` system (resolved
/// against the top-level `<default>` table; bodies aren't involved
/// for actuators).
fn parse_mjcf_actuators_import(
    mujoco_el: roxmltree::Node,
    class_table: &MjcfClassTable,
    joints: &mut [JointData],
    joint_map: &HashMap<String, usize>,
) {
    let Some(actuator_el) = mujoco_el
        .children()
        .find(|n| n.tag_name().name() == "actuator")
    else {
        return;
    };

    use crate::rbd::model::ActuatorMode;

    for el in actuator_el.children().filter(|n| n.is_element()) {
        let tag = el.tag_name().name();
        let (mode, default_kp, default_kv) = match tag {
            "motor" => (ActuatorMode::Torque, None, None),
            "position" => (ActuatorMode::Position, Some(50.0_f64), Some(0.0_f64)),
            "velocity" => (ActuatorMode::Velocity, Some(0.0_f64), Some(5.0_f64)),
            "general" => (ActuatorMode::Torque, None, None),
            // intvelocity, adhesion, damper, cylinder, muscle, ... aren't
            // mapped — leave the joint's default mode untouched.
            _ => continue,
        };

        // Actuators look up via `<actuator>`-level class (no body context).
        let Some(joint_name) = el.attribute("joint") else {
            continue;
        };
        let Some(&ji) = joint_map.get(joint_name) else {
            continue;
        };

        let j = &mut joints[ji];
        j.actuator_mode = mode;

        // Kp / Kv from <position>/<velocity> kp/kv attributes (or class).
        if let Some(kp) =
            mjcf_attr(el, tag, "kp", "", class_table).and_then(|s| s.parse::<f64>().ok())
        {
            j.actuator_kp = kp;
        } else if let Some(def) = default_kp {
            j.actuator_kp = def;
        }
        if let Some(kv) =
            mjcf_attr(el, tag, "kv", "", class_table).and_then(|s| s.parse::<f64>().ok())
        {
            j.actuator_kv = kv;
        } else if let Some(def) = default_kv {
            j.actuator_kv = def;
        }

        // Effort: prefer forcerange (limit on output force), fall back
        // to ctrlrange (limit on the control signal — same units for
        // motor-class actuators when gear == 1).
        if let Some(fr) = mjcf_attr(el, tag, "forcerange", "", class_table)
            .or_else(|| mjcf_attr(el, tag, "ctrlrange", "", class_table))
        {
            let vals: Vec<f64> = fr.split_whitespace().filter_map(|s| s.parse().ok()).collect();
            if vals.len() == 2 {
                let max_abs = vals[0].abs().max(vals[1].abs());
                if max_abs > 0.0 {
                    j.effort = max_abs;
                }
            }
        }
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
/// ground plane, no actuators, all per-joint hardware limits baked in).
#[derive(Clone, Debug)]
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
    /// When true, the `<motor>` actuators carry `forcelimited="true"
    /// forcerange="-effort effort"`, making MuJoCo clamp `data.ctrl` to the
    /// joint's catalogue effort. When false the motors are unrestricted at
    /// the MuJoCo level — useful for "what if the motor were stronger" sweeps.
    /// Defaults to true so a one-off `export_mjcf()` produces a faithful
    /// hardware spec for re-loading in other tools.
    pub bake_actuator_limits: bool,
    /// When true, joints carry their `range="lower upper"` so MuJoCo enforces
    /// the URDF position limits. False omits the range so the joint can swing
    /// past mechanical stops — matching the semantics of "limits off" for
    /// users probing the dynamic envelope.
    pub bake_joint_position_limits: bool,
    /// How `<mesh file="...">` paths are emitted. Default
    /// [`MeshPathStyle::Absolute`] suits in-process loading via
    /// [`mujoco-rs::MjModel::from_xml_string`] (no on-disk anchor →
    /// MuJoCo would otherwise fail to resolve relative paths). When
    /// exporting to a file the user can ship, switch to
    /// [`MeshPathStyle::RelativeToDir`] and call
    /// [`crate::mesh_paths::copy_meshes_to`] afterwards.
    pub mesh_path_style: crate::mesh_paths::MeshPathStyle,
    /// Default contact friction for every emitted geom, ordered
    /// `[sliding, torsional, rolling]`. Emitted into MJCF's
    /// `<default><geom friction="..."/></default>` so ground plane,
    /// foot collisions, and every link collider inherit the same value.
    /// MuJoCo combines contact pairs by per-axis `max`, so foot-on-ground
    /// at this μ from both sides gives a contact μ equal to `sliding`.
    /// Default `[0.7, 0.005, 0.0001]` — μ_slide=0.7 sits in the middle of
    /// the realistic rubber-on-lab-floor range (0.4–1.0) and matches
    /// MPC `friction_mu` defaults.
    pub default_friction: [f64; 3],
}

impl Default for MjcfExportOptions {
    fn default() -> Self {
        Self {
            base_pos: None,
            ground_plane: None,
            add_actuators: false,
            base_locked_axes: [false; 6],
            bake_actuator_limits: true,
            bake_joint_position_limits: true,
            mesh_path_style: crate::mesh_paths::MeshPathStyle::default(),
            default_friction: [0.7, 0.005, 0.0001],
        }
    }
}

/// Export a RobotModel to MJCF XML string with default options.
pub fn export_mjcf(model: &RobotModel) -> String {
    export_mjcf_with_options(model, MjcfExportOptions::default())
}

/// Export a RobotModel to a `.xml` file on disk, copying referenced
/// meshes to `<output_dir>/meshes/` and emitting `meshes/<basename>`
/// relative paths. The result is self-contained and portable —
/// `tar`-ing the output directory and shipping it Just Works on the
/// receiving end.
///
/// For in-process loading via `MjModel::from_xml_string` use
/// [`export_mjcf`] / [`export_mjcf_with_options`] directly with the
/// default `Absolute` mesh-path style.
pub fn export_mjcf_to_file(
    model: &RobotModel,
    output_path: &std::path::Path,
) -> Result<(), String> {
    let output_dir = output_path
        .parent()
        .ok_or_else(|| format!("export_mjcf_to_file: invalid path {:?}", output_path))?
        .to_path_buf();
    let mut opts = MjcfExportOptions::default();
    opts.mesh_path_style =
        crate::mesh_paths::MeshPathStyle::RelativeToDir(output_dir.clone());
    let xml = export_mjcf_with_options(model, opts);
    std::fs::write(output_path, xml)
        .map_err(|e| format!("write {:?}: {e}", output_path))?;
    let copied = crate::mesh_paths::copy_meshes_to(model, &output_dir)?;
    log::info!(
        "Exported MJCF to {:?}, copied {} mesh file(s)",
        output_path,
        copied,
    );
    Ok(())
}

// `absolute_path` moved to `crate::mesh_paths` (shared helper) in the
// MeshPathStyle refactor.

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
        bake_actuator_limits,
        bake_joint_position_limits,
        mesh_path_style,
        default_friction,
    } = opts;
    let mut s = String::new();
    s.push_str(&format!(
        "<mujoco model=\"{}\">\n",
        model.name
    ));

    s.push_str("  <compiler angle=\"radian\"/>\n\n");

    // Sim-side default contact friction. MuJoCo's built-in default
    // (μ_sliding = 1.0) overshoots typical rubber-on-lab-floor values.
    // The host plumbs the desired value through `default_friction`
    // (default 0.7) so every emitted geom — ground plane, foot collisions,
    // link colliders — inherits the same μ. Contact pairs combine via
    // MuJoCo's per-axis `max` policy, so foot-on-ground at this μ from
    // both sides gives a contact μ equal to `default_friction[0]` —
    // matching what the MPC and gait planner expect.
    s.push_str("  <default>\n");
    s.push_str(&format!(
        "    <geom friction=\"{} {} {}\"/>\n",
        default_friction[0], default_friction[1], default_friction[2],
    ));
    s.push_str("  </default>\n\n");

    // Mesh path resolution: delegate to the shared helper so all three
    // exporters (MJCF / SDF / URDF) emit consistent paths and share a
    // single resolution rule for the various URI flavours
    // (package:// / file:// / .misa-style relative / absolute).
    // Path style comes from `MjcfExportOptions.mesh_path_style`:
    // - `Absolute` (default): in-process MuJoCo via `from_xml_string`
    // - `RelativeToDir(dir)`: file export, paired with `copy_meshes_to`
    // - `Preserve`: keep URI verbatim
    // Each mesh asset carries (name, file, scale). The scale tuple matters
    // because URDFs commonly carry meshes in millimetres and apply
    // `scale="0.001 0.001 0.001"` at reference time — dropping the scale on
    // MJCF emission means MuJoCo loads the mesh 1000× larger than the source
    // model intended, producing catastrophic ground penetration at t=0
    // (~MN-scale contact forces).
    let mut mesh_names: Vec<(String, String, [f64; 3])> = Vec::new();
    let mut mesh_counter = 0usize;
    let mut geom_mesh_map: HashMap<*const GeomData, String> = HashMap::new();

    let resolve = |filename: &Option<String>| -> String {
        match filename.as_deref() {
            Some(uri) => crate::mesh_paths::emit_path(uri, model, &mesh_path_style),
            None => "mesh.stl".to_string(),
        }
    };
    let scale_or_unit = |scale: &Option<[f32; 3]>| -> [f64; 3] {
        scale
            .map(|s| [s[0] as f64, s[1] as f64, s[2] as f64])
            .unwrap_or([1.0, 1.0, 1.0])
    };

    for link in &model.links {
        for vis in &link.visuals {
            if let GeomData::Mesh { filename, scale, .. } = &vis.geometry {
                let mesh_name = format!("mesh_{mesh_counter}");
                mesh_names.push((mesh_name.clone(), resolve(filename), scale_or_unit(scale)));
                geom_mesh_map.insert(&vis.geometry as *const GeomData, mesh_name);
                mesh_counter += 1;
            }
        }
        for col in &link.collisions {
            if let GeomData::Mesh { filename, scale, .. } = &col.geometry {
                let mesh_name = format!("mesh_{mesh_counter}");
                mesh_names.push((mesh_name.clone(), resolve(filename), scale_or_unit(scale)));
                geom_mesh_map.insert(&col.geometry as *const GeomData, mesh_name);
                mesh_counter += 1;
            }
        }
    }

    if !mesh_names.is_empty() {
        s.push_str("  <asset>\n");
        for (name, file, scale) in &mesh_names {
            let unit = (scale[0] - 1.0).abs() < 1e-12
                && (scale[1] - 1.0).abs() < 1e-12
                && (scale[2] - 1.0).abs() < 1e-12;
            if unit {
                s.push_str(&format!("    <mesh name=\"{name}\" file=\"{file}\"/>\n"));
            } else {
                s.push_str(&format!(
                    "    <mesh name=\"{name}\" file=\"{file}\" scale=\"{} {} {}\"/>\n",
                    scale[0], scale[1], scale[2],
                ));
            }
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
    // the lowest visual geometry sits ~5 mm above the active ground plane.
    // `compute_initial_z` walked only joint-origin Z and ignored geom shapes
    // (sphere radius, capsule half-length, mesh extent) — for any robot with
    // collision spheres / capsules on its feet that produced a t=0 contact
    // penetration which MuJoCo's contact solver answered with a violent
    // bounce. `RobotModel::compute_min_z` samples the actual visual primitives.
    let root_pos = match base_pos {
        Some(p) => p,
        None => {
            const CLEARANCE_M: f64 = 0.005;
            // model_min_z is in world coords with the current base_transform
            // applied; we want body-relative min_z so subtract the base's
            // current Z out before re-applying the new root_z below.
            let base_z = model.base_transform.translation.z as f64;
            let local_min_z = model
                .compute_min_z()
                .map(|z| z as f64 - base_z)
                .unwrap_or_else(|| compute_initial_z_legacy(model) * -1.0 + 0.01);
            let ground_z = ground_plane.as_ref().map(|g| g.z).unwrap_or(0.0);
            // Solve  root_z + local_min_z = ground_z + clearance  for root_z.
            let root_z = ground_z + CLEARANCE_M - local_min_z;
            [0.0, 0.0, root_z]
        }
    };
    let base_spec = BaseSpec { pos: root_pos, locked: base_locked_axes };
    write_mjcf_body(
        &mut s,
        model,
        &model.root_link,
        4,
        &geom_mesh_map,
        Some(base_spec),
        bake_joint_position_limits,
    );

    s.push_str("  </worldbody>\n");

    if add_actuators {
        write_mjcf_actuators(&mut s, model, bake_actuator_limits);
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
    // Sensor mount sites are emitted by `write_mjcf_body` as
    // `<site name="<sensor>_site"/>` inside the body of `sensor.link`.
    // The sensor entries below reference those sites by name.
    s.push_str("\n  <sensor>\n");
    for sensor in &model.sensors {
        match &sensor.kind {
            crate::rbd::model::SensorKind::Imu { gyro_noise, accel_noise } => {
                let accel_noise_attr = if *accel_noise > 0.0 {
                    format!(" noise=\"{}\"", accel_noise)
                } else {
                    String::new()
                };
                let gyro_noise_attr = if *gyro_noise > 0.0 {
                    format!(" noise=\"{}\"", gyro_noise)
                } else {
                    String::new()
                };
                s.push_str(&format!(
                    "    <accelerometer name=\"{}_accel\" site=\"{}_site\"{accel_noise_attr}/>\n",
                    sensor.name, sensor.name,
                ));
                s.push_str(&format!(
                    "    <gyro name=\"{}_gyro\" site=\"{}_site\"{gyro_noise_attr}/>\n",
                    sensor.name, sensor.name,
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

/// Emit `<contact><exclude>` for parent-child link pairs (URDF semantics:
/// links connected by a joint are NOT supposed to collide) and for every
/// collision pair the user has marked as disabled. MuJoCo's default is
/// "all geoms collide", so we only need to emit excludes — `enabled = true`
/// pairs are implicit.
///
/// Pre-fix, articara only emitted user-defined excludes, so any model that
/// didn't enumerate every parent-child pair by hand (notably models exported
/// without per-pair tuning, e.g. a freshly converted URDF) would simulate
/// with all adjacent collision geoms in mutual contact — generating
/// ~hundreds of self-contacts and meganewton-scale force vectors at t=0.
/// Mirroring URDF's "parent and child don't collide" rule fixes this for
/// every model without requiring user input.
fn write_mjcf_contact_excludes(s: &mut String, model: &RobotModel) {
    // 1. Parent-child pairs from every joint (URDF semantic).
    let mut pairs: Vec<(String, String)> = Vec::new();
    let mut seen: std::collections::HashSet<(String, String)> =
        std::collections::HashSet::new();
    let mut record = |a: &str, b: &str, list: &mut Vec<(String, String)>, seen: &mut std::collections::HashSet<(String, String)>| {
        // Canonical (sorted) key for dedup so (A,B) and (B,A) collapse.
        let (k_a, k_b) = if a <= b { (a.to_string(), b.to_string()) } else { (b.to_string(), a.to_string()) };
        if seen.insert((k_a.clone(), k_b.clone())) {
            list.push((k_a, k_b));
        }
    };
    for j in &model.joints {
        if model.link_map.contains_key(&j.parent_link)
            && model.link_map.contains_key(&j.child_link)
            && j.parent_link != j.child_link
        {
            record(&j.parent_link, &j.child_link, &mut pairs, &mut seen);
        }
    }
    // 2. User-marked disabled pairs (overrides + extras the URDF doesn't capture,
    //    e.g. closed-kinematic-loop neighbours connected only via a loop closure).
    for p in &model.collision_pairs {
        if p.enabled {
            continue;
        }
        if model.link_map.contains_key(&p.link_a)
            && model.link_map.contains_key(&p.link_b)
        {
            record(&p.link_a, &p.link_b, &mut pairs, &mut seen);
        }
    }
    if pairs.is_empty() {
        return;
    }
    s.push_str("\n  <contact>\n");
    for (a, b) in &pairs {
        s.push_str(&format!(
            "    <exclude body1=\"{a}\" body2=\"{b}\"/>\n",
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
fn write_mjcf_actuators(s: &mut String, model: &RobotModel, bake_limits: bool) {
    // Skip both URDF-fixed joints and joints the user marked as
    // `ActuatorMode::Fixed`. The body emitter already omits the <joint>
    // element for the latter, so an actuator referencing it would be a
    // dangling MJCF reference.
    let movable: Vec<&JointData> = model
        .joints
        .iter()
        .filter(|j| j.joint_type != "fixed" && !j.actuator_mode.is_fixed())
        .collect();
    if movable.is_empty() {
        return;
    }
    s.push_str("\n  <actuator>\n");
    for joint in movable {
        let force_attrs = if bake_limits && joint.effort > 0.0 {
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
/// Legacy joint-origin-only fallback. Used only when `compute_min_z` returns
/// `None` (e.g. a model with no visual geometry at all). For models with
/// visuals, the auto-lift path uses `RobotModel::compute_min_z` directly
/// because it accounts for primitive shapes (sphere radius etc.).
fn compute_initial_z_legacy(model: &RobotModel) -> f64 {
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
    min_z_recursive(model, &model.root_link, 0.0)
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
    bake_joint_position_limits: bool,
) {
    let pad: String = " ".repeat(indent);

    let link_idx = match model.link_map.get(link_name) {
        Some(&i) => i,
        None => return,
    };
    let link = &model.links[link_idx];

    // Find the joint connecting this link to its parent for pose
    let (pos_str, quat_str, joint_info) = if let Some(spec) = base_spec {
        let [x, y, z] = spec.pos;
        // Root body: place at user-specified or auto-lifted world position.
        // Identity orientation; the floating-base joints below carry the
        // current trunk rotation.
        (format!("{x} {y} {z}"), String::new(), None)
    } else if let Some(ji) = model.parent_joint_of_link(link_name) {
        let joint = &model.joints[ji];
        let t = &joint.origin.translation;
        let pos = format!("{} {} {}", t.x, t.y, t.z);
        // URDF joint origin's `rpy` rotates the *child link* relative to its
        // parent (it's the orientation of the joint frame, which the child
        // inherits at q=0). MuJoCo's `<body>` has no rpy attribute, so we
        // express the same rotation as a `quat` (`w x y z`).
        //
        // Dropping this rotation here was the root cause of the gait /
        // CHAMP "forward command produces lateral motion" bug on robots
        // (keel, etc.) that spell their thigh pitch axis as
        // `<origin rpy="0 0 π/2"/> <axis xyz="1 0 0"/>` — without the
        // body quaternion, MuJoCo sees the joint axis in its local frame
        // (= body X) and rotates the thigh about body X instead of body Y.
        let q = joint.origin.rotation.quaternion();
        let is_identity = (q.w - 1.0).abs() < 1e-9
            && q.i.abs() < 1e-9
            && q.j.abs() < 1e-9
            && q.k.abs() < 1e-9;
        let quat = if is_identity {
            String::new()
        } else {
            // MuJoCo quat order is `w x y z`.
            format!(" quat=\"{} {} {} {}\"", q.w, q.i, q.j, q.k)
        };
        (pos, quat, Some(joint))
    } else {
        ("0 0 0".into(), String::new(), None)
    };

    s.push_str(&format!(
        "{pad}<body name=\"{link_name}\" pos=\"{pos_str}\"{quat_str}>\n"
    ));

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

    // Inertial.
    //
    // When the link reports non-zero products of inertia (`ixy / ixz / iyz`)
    // the inertia tensor's principal axes are rotated relative to the link
    // frame — emitting only `diaginertia` silently throws those rotations
    // away and gives MuJoCo a different inertia than the source URDF / .misa
    // describes. For a heavy trunk that effect alone is enough to make a
    // passive sim look unstable / wobbly on contact even before the gait
    // controller runs. Use MuJoCo's `fullinertia="Ixx Iyy Izz Ixy Ixz Iyz"`
    // form whenever any off-diagonal is non-trivial; fall back to the
    // cheaper `diaginertia` only for clean diagonal tensors.
    if link.inertial.mass > 1e-12 {
        let it = &link.inertial.origin.translation;
        let off_diag_eps = 1e-12;
        let has_off_diag = link.inertial.ixy.abs() > off_diag_eps
            || link.inertial.ixz.abs() > off_diag_eps
            || link.inertial.iyz.abs() > off_diag_eps;
        if has_off_diag {
            s.push_str(&format!(
                "{pad}  <inertial mass=\"{}\" pos=\"{} {} {}\" \
                 fullinertia=\"{} {} {} {} {} {}\"/>\n",
                link.inertial.mass,
                it.x, it.y, it.z,
                link.inertial.ixx,
                link.inertial.iyy,
                link.inertial.izz,
                link.inertial.ixy,
                link.inertial.ixz,
                link.inertial.iyz,
            ));
        } else {
            s.push_str(&format!(
                "{pad}  <inertial mass=\"{}\" pos=\"{} {} {}\" diaginertia=\"{} {} {}\"/>\n",
                link.inertial.mass,
                it.x, it.y, it.z,
                link.inertial.ixx,
                link.inertial.iyy,
                link.inertial.izz
            ));
        }
    }

    // Joint
    if let Some(joint) = joint_info {
        // `actuator_mode == Fixed` is the "MJCF-only weld" shortcut: omit the
        // <joint> element entirely so MuJoCo treats parent and child as a
        // single rigid body. We deliberately don't touch `joint.joint_type`
        // here — URDF / .misa export and the host's FK keep using the
        // declared type. See `ActuatorMode::Fixed` doc comment.
        if joint.joint_type != "fixed" && !joint.actuator_mode.is_fixed() {
            let mjcf_type = match joint.joint_type.as_str() {
                "revolute" | "continuous" => "hinge",
                "prismatic" => "slide",
                other => other,
            };
            s.push_str(&format!(
                "{pad}  <joint name=\"{}\" type=\"{mjcf_type}\" axis=\"{} {} {}\"",
                joint.name, joint.axis.x, joint.axis.y, joint.axis.z
            ));
            if bake_joint_position_limits && joint.lower < joint.upper {
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
    // When the link's `collision_enabled` flag is OFF, the collision geoms
    // get the same contype/conaffinity bits as visuals — they're rendered
    // in the collision-group viewer but the physics solver skips every
    // contact pair involving them. Same effect as the MuJoCo
    // contype=0/conaffinity=0 convention without needing dedicated bit-mask
    // bookkeeping.
    let collision_extra = if link.collision_enabled {
        " contype=\"1\" conaffinity=\"1\" group=\"3\""
    } else {
        " contype=\"0\" conaffinity=\"0\" group=\"3\""
    };

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
        // Build per-geom physics attribute suffix (friction / condim /
        // priority / solimp / margin). Empty when `col.physics` is None
        // or all sub-fields are None ⇒ MuJoCo falls back to <default>.
        let mut phys_attrs = String::new();
        if let Some(p) = &col.physics {
            if let Some(f) = p.friction {
                phys_attrs.push_str(&format!(" friction=\"{} {} {}\"", f[0], f[1], f[2]));
            }
            if let Some(c) = p.condim {
                phys_attrs.push_str(&format!(" condim=\"{c}\""));
            }
            if let Some(pr) = p.priority {
                phys_attrs.push_str(&format!(" priority=\"{pr}\""));
            }
            if let Some(si) = p.solimp {
                phys_attrs.push_str(&format!(" solimp=\"{} {} {}\"", si[0], si[1], si[2]));
            }
            if let Some(m) = p.margin {
                phys_attrs.push_str(&format!(" margin=\"{m}\""));
            }
        }
        let phys_attrs = phys_attrs.as_str();
        match &col.geometry {
            GeomData::Box { hx, hy, hz } => {
                s.push_str(&format!(
                    "{pad}  <geom type=\"box\" pos=\"{pos_attr}\" size=\"{hx} {hy} {hz}\" rgba=\"{col_rgba}\"{collision_extra}{phys_attrs}/>\n",
                ));
            }
            GeomData::Cylinder { radius, half_length } => {
                s.push_str(&format!(
                    "{pad}  <geom type=\"cylinder\" pos=\"{pos_attr}\" size=\"{radius} {half_length}\" rgba=\"{col_rgba}\"{collision_extra}{phys_attrs}/>\n",
                ));
            }
            GeomData::Sphere { radius } => {
                s.push_str(&format!(
                    "{pad}  <geom type=\"sphere\" pos=\"{pos_attr}\" size=\"{radius}\" rgba=\"{col_rgba}\"{collision_extra}{phys_attrs}/>\n",
                ));
            }
            GeomData::Capsule { radius, half_length } => {
                s.push_str(&format!(
                    "{pad}  <geom type=\"capsule\" pos=\"{pos_attr}\" size=\"{radius} {half_length}\" rgba=\"{col_rgba}\"{collision_extra}{phys_attrs}/>\n",
                ));
            }
            GeomData::Mesh { .. } => {
                let ptr = &col.geometry as *const GeomData;
                if let Some(mesh_name) = geom_mesh_map.get(&ptr) {
                    s.push_str(&format!(
                        "{pad}  <geom type=\"mesh\" mesh=\"{mesh_name}\" pos=\"{pos_attr}\" rgba=\"{col_rgba}\"{collision_extra}{phys_attrs}/>\n",
                    ));
                }
            }
        }
    }

    // Sites for sensors mounted on this link. MuJoCo sensors like
    // `<accelerometer>` / `<gyro>` reference a `<site>` for their
    // attachment frame; without an explicit site definition the model
    // load fails with "site not found". The site is named `<sensor>_site`
    // so `write_mjcf_sensors` can refer to it deterministically.
    for sensor in &model.sensors {
        if sensor.link != link_name {
            continue;
        }
        let t = &sensor.origin.translation;
        let q = sensor.origin.rotation.quaternion();
        // MuJoCo quaternion order is (w, x, y, z).
        s.push_str(&format!(
            "{pad}  <site name=\"{}_site\" pos=\"{} {} {}\" quat=\"{} {} {} {}\" size=\"0.005\"/>\n",
            sensor.name,
            t.x, t.y, t.z,
            q.w, q.i, q.j, q.k,
        ));
    }

    // Recurse children
    if let Some(child_joints) = model.children_joints.get(link_name) {
        for &ji in child_joints {
            let child_link = &model.joints[ji].child_link;
            write_mjcf_body(
                s,
                model,
                child_link,
                indent + 2,
                geom_mesh_map,
                None,
                bake_joint_position_limits,
            );
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
    body_childclass: &str,
    class_table: &MjcfClassTable,
) -> GeomData {
    // `type` and `mesh` must go through the <default> resolver. The
    // Menagerie convention puts mesh visuals on a `class="visual"` that
    // declares `<geom type="mesh"/>` in the class block, so the per-geom
    // element has *only* `mesh="…" class="visual"` — no inline `type=`.
    // Reading `type` inline-only made every such geom collapse to the
    // hard-coded "sphere" default and the mesh visual to disappear.
    let geom_type =
        mjcf_attr(geom_el, "geom", "type", body_childclass, class_table)
            .unwrap_or_else(|| {
                // MJCF's true default for a `<geom>` referencing a mesh is
                // `type="mesh"`, otherwise `type="sphere"`. We don't
                // implement the full element-aware default fallback, but
                // we can at least promote any geom with a `mesh` attribute
                // (inline or class-inherited) to mesh-type so Menagerie's
                // implicit-typed mesh geoms round-trip correctly.
                if mjcf_attr(geom_el, "geom", "mesh", body_childclass, class_table).is_some() {
                    "mesh".to_string()
                } else {
                    "sphere".to_string()
                }
            });
    let size: Vec<f32> = mjcf_attr(geom_el, "geom", "size", body_childclass, class_table)
        .unwrap_or_default()
        .split_whitespace()
        .filter_map(|s| s.parse().ok())
        .collect();

    match geom_type.as_str() {
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
            if let Some(mesh_name) =
                mjcf_attr(geom_el, "geom", "mesh", body_childclass, class_table)
            {
                let filename = mesh_assets
                    .get(&mesh_name)
                    .cloned()
                    .unwrap_or_else(|| format!("{mesh_name}.stl"));
                // `filename` already carries the meshdir prefix from the
                // asset table, so the on-disk path is simply
                // `<mjcf_dir>/<filename>`.
                let mesh_path = mjcf_dir.join(&filename);
                // Dispatch on extension so .obj (used by Menagerie's
                // Unitree / Boston Dynamics / ANYmal models) loads
                // through the OBJ parser, not the STL one.
                let vertices = crate::robot::load_mesh_file(&mesh_path);
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
