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

    let joint_positions = vec![0.0f32; joints.len()];

    Ok(RobotModel {
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
    })
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

/// Export a RobotModel to MJCF XML string.
pub fn export_mjcf(model: &RobotModel) -> String {
    let mut s = String::new();
    s.push_str(&format!(
        "<mujoco model=\"{}\">\n",
        model.name
    ));
    s.push_str("  <compiler angle=\"radian\"/>\n\n");

    // Collect mesh assets
    let mut mesh_names: Vec<(String, String)> = Vec::new();
    let mut mesh_counter = 0usize;
    let mut geom_mesh_map: HashMap<*const GeomData, String> = HashMap::new();

    for link in &model.links {
        for vis in &link.visuals {
            if let GeomData::Mesh { filename, .. } = &vis.geometry {
                let mesh_name = format!("mesh_{mesh_counter}");
                let fname = filename
                    .as_deref()
                    .and_then(|f| f.rsplit('/').next())
                    .unwrap_or("mesh.stl");
                mesh_names.push((mesh_name.clone(), fname.to_string()));
                geom_mesh_map.insert(&vis.geometry as *const GeomData, mesh_name);
                mesh_counter += 1;
            }
        }
    }

    if !mesh_names.is_empty() {
        s.push_str("  <asset>\n");
        for (name, file) in &mesh_names {
            s.push_str(&format!(
                "    <mesh name=\"{name}\" file=\"meshes/{file}\"/>\n"
            ));
        }
        s.push_str("  </asset>\n\n");
    }

    s.push_str("  <worldbody>\n");

    // Build body hierarchy
    write_mjcf_body(&mut s, model, &model.root_link, 4, &geom_mesh_map);

    s.push_str("  </worldbody>\n");
    s.push_str("</mujoco>\n");
    s
}

fn write_mjcf_body(
    s: &mut String,
    model: &RobotModel,
    link_name: &str,
    indent: usize,
    geom_mesh_map: &HashMap<*const GeomData, String>,
) {
    let pad: String = " ".repeat(indent);

    let link_idx = match model.link_map.get(link_name) {
        Some(&i) => i,
        None => return,
    };
    let link = &model.links[link_idx];

    // Find the joint connecting this link to its parent for pose
    let (pos_str, joint_info) = if let Some(ji) = model.parent_joint_of_link(link_name) {
        let joint = &model.joints[ji];
        let t = &joint.origin.translation;
        let pos = format!("{} {} {}", t.x, t.y, t.z);
        (pos, Some(joint))
    } else {
        ("0 0 0".into(), None)
    };

    s.push_str(&format!("{pad}<body name=\"{link_name}\" pos=\"{pos_str}\">\n"));

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
            s.push_str("/>\n");
        }
    }

    // Geoms (visuals)
    for vis in &link.visuals {
        let t = &vis.origin.translation;
        let pos_attr = format!("{} {} {}", t.x, t.y, t.z);
        match &vis.geometry {
            GeomData::Box { hx, hy, hz } => {
                s.push_str(&format!(
                    "{pad}  <geom type=\"box\" pos=\"{pos_attr}\" size=\"{hx} {hy} {hz}\" rgba=\"{} {} {} {}\"/>\n",
                    vis.color[0], vis.color[1], vis.color[2], vis.color[3]
                ));
            }
            GeomData::Cylinder {
                radius,
                half_length,
            } => {
                s.push_str(&format!(
                    "{pad}  <geom type=\"cylinder\" pos=\"{pos_attr}\" size=\"{radius} {half_length}\" rgba=\"{} {} {} {}\"/>\n",
                    vis.color[0], vis.color[1], vis.color[2], vis.color[3]
                ));
            }
            GeomData::Sphere { radius } => {
                s.push_str(&format!(
                    "{pad}  <geom type=\"sphere\" pos=\"{pos_attr}\" size=\"{radius}\" rgba=\"{} {} {} {}\"/>\n",
                    vis.color[0], vis.color[1], vis.color[2], vis.color[3]
                ));
            }
            GeomData::Mesh { .. } => {
                let ptr = &vis.geometry as *const GeomData;
                if let Some(mesh_name) = geom_mesh_map.get(&ptr) {
                    s.push_str(&format!(
                        "{pad}  <geom type=\"mesh\" mesh=\"{mesh_name}\" pos=\"{pos_attr}\" rgba=\"{} {} {} {}\"/>\n",
                        vis.color[0], vis.color[1], vis.color[2], vis.color[3]
                    ));
                }
            }
        }
    }

    // Recurse children
    if let Some(child_joints) = model.children_joints.get(link_name) {
        for &ji in child_joints {
            let child_link = &model.joints[ji].child_link;
            write_mjcf_body(s, model, child_link, indent + 2, geom_mesh_map);
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
        "cylinder" | "capsule" => {
            let radius = size.first().copied().unwrap_or(0.05);
            let half_length = size.get(1).copied().unwrap_or(0.1);
            GeomData::Cylinder {
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

// Make GeomData Clone for MJCF import (geom used for both visual and collision)
impl Clone for GeomData {
    fn clone(&self) -> Self {
        match self {
            GeomData::Box { hx, hy, hz } => GeomData::Box {
                hx: *hx,
                hy: *hy,
                hz: *hz,
            },
            GeomData::Cylinder {
                radius,
                half_length,
            } => GeomData::Cylinder {
                radius: *radius,
                half_length: *half_length,
            },
            GeomData::Sphere { radius } => GeomData::Sphere { radius: *radius },
            GeomData::Mesh {
                vertices,
                filename,
                scale,
            } => GeomData::Mesh {
                vertices: vertices.clone(),
                filename: filename.clone(),
                scale: *scale,
            },
        }
    }
}
