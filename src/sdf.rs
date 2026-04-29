//! SDF (Simulation Description Format) import and export.
//!
//! SDF is used by Gazebo. We support a subset that maps to our RobotModel.

use nalgebra as na;
use std::collections::{HashMap, HashSet};
use std::path::{Path, PathBuf};

use crate::robot::*;

// ========== Import ==========

/// Parse an SDF file and return a RobotModel.
pub fn import_sdf(path: &Path) -> Result<RobotModel, String> {
    let xml = std::fs::read_to_string(path).map_err(|e| format!("Read SDF: {e}"))?;
    let doc = roxmltree::Document::parse(&xml).map_err(|e| format!("Parse SDF XML: {e}"))?;

    let sdf_dir = path.parent().unwrap_or(Path::new("."));

    // Find the <model> element
    let model_el = doc
        .descendants()
        .find(|n| n.tag_name().name() == "model")
        .ok_or("No <model> element found in SDF")?;

    let robot_name = model_el.attribute("name").unwrap_or("sdf_model").to_string();

    let mut links = Vec::new();
    let mut link_map = HashMap::new();
    let mut joints = Vec::new();
    let mut joint_map = HashMap::new();
    let mut children_joints: HashMap<String, Vec<usize>> = HashMap::new();
    let mut child_links: HashSet<String> = HashSet::new();

    // Parse links
    for (i, link_el) in model_el
        .children()
        .filter(|n| n.tag_name().name() == "link")
        .enumerate()
    {
        let name = link_el.attribute("name").unwrap_or("link").to_string();

        // Inertial
        let inertial = parse_sdf_inertial(link_el);

        // Visuals
        let visuals = link_el
            .children()
            .filter(|n| n.tag_name().name() == "visual")
            .map(|v| parse_sdf_visual(v, sdf_dir))
            .collect();

        // Collisions
        let collisions = link_el
            .children()
            .filter(|n| n.tag_name().name() == "collision")
            .map(|c| CollisionData {
                origin: parse_sdf_pose(c),
                geometry: parse_sdf_geometry(c, sdf_dir),
            })
            .collect();

        link_map.insert(name.clone(), i);
        links.push(LinkData {
            name,
            visuals,
            collisions,
            inertial,
        });
    }

    // Parse joints
    for (i, joint_el) in model_el
        .children()
        .filter(|n| n.tag_name().name() == "joint")
        .enumerate()
    {
        let name = joint_el.attribute("name").unwrap_or("joint").to_string();
        let jtype = joint_el
            .attribute("type")
            .unwrap_or("fixed")
            .to_string();

        let parent = joint_el
            .children()
            .find(|n| n.tag_name().name() == "parent")
            .and_then(|n| n.text())
            .unwrap_or("world")
            .to_string();

        let child = joint_el
            .children()
            .find(|n| n.tag_name().name() == "child")
            .and_then(|n| n.text())
            .unwrap_or("link")
            .to_string();

        let origin = parse_sdf_pose(joint_el);

        let mut axis = na::Vector3::new(0.0, 0.0, 1.0_f32);
        let mut lower = 0.0_f64;
        let mut upper = 0.0_f64;
        let mut effort = 0.0_f64;
        let mut velocity = 0.0_f64;

        if let Some(axis_el) = joint_el.children().find(|n| n.tag_name().name() == "axis") {
            if let Some(xyz) = axis_el.children().find(|n| n.tag_name().name() == "xyz") {
                axis = parse_vec3_text(xyz.text().unwrap_or("0 0 1"));
            }
            if let Some(limit_el) = axis_el.children().find(|n| n.tag_name().name() == "limit") {
                lower = get_child_f64(limit_el, "lower");
                upper = get_child_f64(limit_el, "upper");
                effort = get_child_f64(limit_el, "effort");
                velocity = get_child_f64(limit_el, "velocity");
            }
        }

        joint_map.insert(name.clone(), i);
        children_joints
            .entry(parent.clone())
            .or_default()
            .push(i);
        child_links.insert(child.clone());

        joints.push(JointData {
            name,
            joint_type: jtype,
            parent_link: parent,
            child_link: child,
            origin,
            axis,
            lower,
            upper,
            effort,
            velocity,
            actuator_mode: crate::rbd::model::ActuatorMode::default(),
            actuator_kp: 50.0,
            actuator_kv: 5.0,
        });
    }

    let root_link = links
        .iter()
        .find(|l| !child_links.contains(&l.name))
        .map(|l| l.name.clone())
        .unwrap_or_default();

    let joint_positions = vec![0.0_f64; joints.len()];

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
        loop_closures: Vec::new(),
        poses: Vec::new(),
        collision_pairs: Vec::new(),
            sequences: Vec::new(),
    };
    model.rebuild_misarta_model();
    Ok(model)
}

// ========== Export ==========

/// Export a RobotModel to SDF XML string.
pub fn export_sdf(model: &RobotModel) -> String {
    let mut s = String::new();
    s.push_str("<?xml version=\"1.0\"?>\n");
    s.push_str("<sdf version=\"1.7\">\n");
    s.push_str(&format!("  <model name=\"{}\">\n", model.name));

    // Links
    for link in &model.links {
        s.push_str(&format!("    <link name=\"{}\">\n", link.name));

        // Inertial
        s.push_str("      <inertial>\n");
        let t = &link.inertial.origin.translation;
        let (r, p, y) = link.inertial.origin.rotation.euler_angles();
        s.push_str(&format!(
            "        <pose>{} {} {} {} {} {}</pose>\n",
            t.x, t.y, t.z, r, p, y
        ));
        s.push_str(&format!("        <mass>{}</mass>\n", link.inertial.mass));
        s.push_str("        <inertia>\n");
        s.push_str(&format!("          <ixx>{}</ixx>\n", link.inertial.ixx));
        s.push_str(&format!("          <ixy>{}</ixy>\n", link.inertial.ixy));
        s.push_str(&format!("          <ixz>{}</ixz>\n", link.inertial.ixz));
        s.push_str(&format!("          <iyy>{}</iyy>\n", link.inertial.iyy));
        s.push_str(&format!("          <iyz>{}</iyz>\n", link.inertial.iyz));
        s.push_str(&format!("          <izz>{}</izz>\n", link.inertial.izz));
        s.push_str("        </inertia>\n");
        s.push_str("      </inertial>\n");

        // Visuals
        for (vi, vis) in link.visuals.iter().enumerate() {
            s.push_str(&format!("      <visual name=\"visual_{vi}\">\n"));
            write_sdf_pose(&mut s, &vis.origin, 8);
            write_sdf_geometry(&mut s, &vis.geometry, 8);
            s.push_str(&format!(
                "        <material>\n          <ambient>{} {} {} {}</ambient>\n        </material>\n",
                vis.color[0], vis.color[1], vis.color[2], vis.color[3]
            ));
            s.push_str("      </visual>\n");
        }

        // Collisions
        for (ci, col) in link.collisions.iter().enumerate() {
            s.push_str(&format!("      <collision name=\"collision_{ci}\">\n"));
            write_sdf_pose(&mut s, &col.origin, 8);
            write_sdf_geometry(&mut s, &col.geometry, 8);
            s.push_str("      </collision>\n");
        }

        s.push_str("    </link>\n");
    }

    // Joints
    for joint in &model.joints {
        s.push_str(&format!(
            "    <joint name=\"{}\" type=\"{}\">\n",
            joint.name, joint.joint_type
        ));
        s.push_str(&format!("      <parent>{}</parent>\n", joint.parent_link));
        s.push_str(&format!("      <child>{}</child>\n", joint.child_link));
        write_sdf_pose(&mut s, &joint.origin, 6);
        s.push_str("      <axis>\n");
        s.push_str(&format!(
            "        <xyz>{} {} {}</xyz>\n",
            joint.axis.x, joint.axis.y, joint.axis.z
        ));
        s.push_str("        <limit>\n");
        s.push_str(&format!("          <lower>{}</lower>\n", joint.lower));
        s.push_str(&format!("          <upper>{}</upper>\n", joint.upper));
        s.push_str(&format!("          <effort>{}</effort>\n", joint.effort));
        s.push_str(&format!("          <velocity>{}</velocity>\n", joint.velocity));
        s.push_str("        </limit>\n");
        s.push_str("      </axis>\n");
        s.push_str("    </joint>\n");
    }

    s.push_str("  </model>\n");
    s.push_str("</sdf>\n");
    s
}

/// Export SDF to a file and copy referenced mesh files.
pub fn export_sdf_to_file(model: &RobotModel, output_path: &Path) -> Result<(), String> {
    let xml = export_sdf(model);
    std::fs::write(output_path, &xml).map_err(|e| format!("Write SDF: {e}"))?;

    // Copy mesh files from source location
    let source = match model.source_path.as_ref() {
        Some(p) => p,
        None => {
            log::warn!("No source path — skipping mesh copy for SDF export");
            return Ok(());
        }
    };
    let source_dir = source.parent().unwrap_or(Path::new("."));
    let source_package_dir = source_dir.parent().unwrap_or(source_dir);
    let output_dir = output_path.parent().unwrap_or(Path::new("."));
    let output_package_dir = output_dir.parent().unwrap_or(output_dir);

    let mut copied: std::collections::HashSet<PathBuf> = std::collections::HashSet::new();
    let mut copy_count = 0u32;

    // Collect all mesh filenames from visuals and collisions
    for link in &model.links {
        let geom_iter = link.visuals.iter().map(|v| &v.geometry)
            .chain(link.collisions.iter().map(|c| &c.geometry));
        for geom in geom_iter {
            if let GeomData::Mesh { filename: Some(uri), .. } = geom {
                let src_abs = crate::robot::resolve_package_path(uri, source_package_dir);
                if copied.contains(&src_abs) || !src_abs.exists() {
                    if !src_abs.exists() {
                        log::warn!("Mesh file not found, skipping: {:?}", src_abs);
                    }
                    continue;
                }
                copied.insert(src_abs.clone());

                let dst_abs = crate::robot::resolve_package_path(uri, output_package_dir);
                if let Some(dst_parent) = dst_abs.parent() {
                    std::fs::create_dir_all(dst_parent)
                        .map_err(|e| format!("Create mesh dir {:?}: {e}", dst_parent))?;
                }
                if src_abs != dst_abs {
                    std::fs::copy(&src_abs, &dst_abs).map_err(|e| {
                        format!("Copy mesh {:?} -> {:?}: {e}",
                            src_abs.file_name().unwrap_or_default(), dst_abs)
                    })?;
                    copy_count += 1;
                }
            }
        }
    }

    log::info!("Exported SDF to {:?}, copied {} mesh file(s)", output_path, copy_count);
    Ok(())
}

// ========== Helpers ==========

fn parse_sdf_pose(node: roxmltree::Node) -> na::Isometry3<f32> {
    if let Some(pose) = node.children().find(|n| n.tag_name().name() == "pose") {
        let vals: Vec<f32> = pose
            .text()
            .unwrap_or("0 0 0 0 0 0")
            .split_whitespace()
            .filter_map(|s| s.parse().ok())
            .collect();
        if vals.len() >= 6 {
            let translation = na::Translation3::new(vals[0], vals[1], vals[2]);
            let rotation =
                na::UnitQuaternion::from_euler_angles(vals[3], vals[4], vals[5]);
            return na::Isometry3::from_parts(translation, rotation);
        }
    }
    na::Isometry3::identity()
}

fn parse_sdf_inertial(link_el: roxmltree::Node) -> InertialData {
    if let Some(inertial) = link_el.children().find(|n| n.tag_name().name() == "inertial") {
        let origin = parse_sdf_pose(inertial);
        let mass = get_child_f64(inertial, "mass");
        let (ixx, ixy, ixz, iyy, iyz, izz) =
            if let Some(i) = inertial.children().find(|n| n.tag_name().name() == "inertia") {
                (
                    get_child_f64(i, "ixx"),
                    get_child_f64(i, "ixy"),
                    get_child_f64(i, "ixz"),
                    get_child_f64(i, "iyy"),
                    get_child_f64(i, "iyz"),
                    get_child_f64(i, "izz"),
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

fn parse_sdf_visual(node: roxmltree::Node, sdf_dir: &Path) -> VisualData {
    let origin = parse_sdf_pose(node);
    let geometry = parse_sdf_geometry(node, sdf_dir);

    // Color from <material>
    let mut color = [0.8_f32, 0.8, 0.8, 1.0];
    if let Some(mat) = node.children().find(|n| n.tag_name().name() == "material") {
        for child in mat.children() {
            if child.tag_name().name() == "ambient" || child.tag_name().name() == "diffuse" {
                let vals: Vec<f32> = child
                    .text()
                    .unwrap_or("")
                    .split_whitespace()
                    .filter_map(|s| s.parse().ok())
                    .collect();
                if vals.len() >= 3 {
                    color = [vals[0], vals[1], vals[2], vals.get(3).copied().unwrap_or(1.0)];
                }
                break;
            }
        }
    }
    VisualData {
        origin,
        geometry,
        color,
    }
}

fn parse_sdf_geometry(node: roxmltree::Node, sdf_dir: &Path) -> GeomData {
    if let Some(geom) = node.children().find(|n| n.tag_name().name() == "geometry") {
        for child in geom.children() {
            match child.tag_name().name() {
                "box" => {
                    if let Some(size) = child.children().find(|n| n.tag_name().name() == "size") {
                        let v = parse_vec3_text(size.text().unwrap_or("0.1 0.1 0.1"));
                        return GeomData::Box {
                            hx: v.x / 2.0,
                            hy: v.y / 2.0,
                            hz: v.z / 2.0,
                        };
                    }
                }
                "cylinder" => {
                    let radius = get_child_f64(child, "radius") as f32;
                    let length = get_child_f64(child, "length") as f32;
                    return GeomData::Cylinder {
                        radius,
                        half_length: length / 2.0,
                    };
                }
                "sphere" => {
                    let radius = get_child_f64(child, "radius") as f32;
                    return GeomData::Sphere { radius };
                }
                "capsule" => {
                    let radius = get_child_f64(child, "radius") as f32;
                    let length = get_child_f64(child, "length") as f32;
                    return GeomData::Capsule {
                        radius,
                        half_length: length / 2.0,
                    };
                }
                "mesh" => {
                    if let Some(uri) = child.children().find(|n| n.tag_name().name() == "uri") {
                        let filename = uri.text().unwrap_or("").to_string();
                        let mesh_path = resolve_sdf_uri(&filename, sdf_dir);
                        let vertices = crate::robot::load_stl_mesh_public(&mesh_path);
                        // Read optional <scale>
                        let scale = child
                            .children()
                            .find(|n| n.tag_name().name() == "scale")
                            .and_then(|n| n.text())
                            .and_then(|t| {
                                let v: Vec<f32> = t.split_whitespace()
                                    .filter_map(|s| s.parse().ok())
                                    .collect();
                                if v.len() >= 3 { Some([v[0], v[1], v[2]]) } else { None }
                            });
                        return GeomData::Mesh {
                            vertices,
                            filename: Some(filename),
                            scale,
                        };
                    }
                }
                _ => {}
            }
        }
    }
    GeomData::Box {
        hx: 0.01,
        hy: 0.01,
        hz: 0.01,
    }
}

fn write_sdf_pose(s: &mut String, iso: &na::Isometry3<f32>, indent: usize) {
    let t = &iso.translation;
    let (r, p, y) = iso.rotation.euler_angles();
    let pad: String = " ".repeat(indent);
    s.push_str(&format!(
        "{pad}<pose>{} {} {} {} {} {}</pose>\n",
        t.x, t.y, t.z, r, p, y
    ));
}

fn write_sdf_geometry(s: &mut String, geom: &GeomData, indent: usize) {
    let pad: String = " ".repeat(indent);
    s.push_str(&format!("{pad}<geometry>\n"));
    match geom {
        GeomData::Box { hx, hy, hz } => {
            s.push_str(&format!(
                "{pad}  <box><size>{} {} {}</size></box>\n",
                hx * 2.0,
                hy * 2.0,
                hz * 2.0
            ));
        }
        GeomData::Cylinder {
            radius,
            half_length,
        } => {
            s.push_str(&format!(
                "{pad}  <cylinder><radius>{radius}</radius><length>{}</length></cylinder>\n",
                half_length * 2.0
            ));
        }
        GeomData::Sphere { radius } => {
            s.push_str(&format!(
                "{pad}  <sphere><radius>{radius}</radius></sphere>\n"
            ));
        }
        GeomData::Capsule { radius, half_length } => {
            s.push_str(&format!(
                "{pad}  <capsule><radius>{radius}</radius><length>{}</length></capsule>\n",
                half_length * 2.0
            ));
        }
        GeomData::Mesh { filename, scale, .. } => {
            let uri = filename.as_deref().unwrap_or("mesh.stl");
            s.push_str(&format!("{pad}  <mesh>\n"));
            s.push_str(&format!("{pad}    <uri>{uri}</uri>\n"));
            if let Some(sc) = scale {
                s.push_str(&format!(
                    "{pad}    <scale>{} {} {}</scale>\n",
                    sc[0], sc[1], sc[2]
                ));
            }
            s.push_str(&format!("{pad}  </mesh>\n"));
        }
    }
    s.push_str(&format!("{pad}</geometry>\n"));
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

fn get_child_f64(node: roxmltree::Node, tag: &str) -> f64 {
    node.children()
        .find(|n| n.tag_name().name() == tag)
        .and_then(|n| n.text())
        .and_then(|t| t.trim().parse().ok())
        .unwrap_or(0.0)
}

fn resolve_sdf_uri(uri: &str, sdf_dir: &Path) -> PathBuf {
    if let Some(rest) = uri.strip_prefix("package://") {
        // Same logic as URDF: package_dir is the parent of sdf_dir
        let package_dir = sdf_dir.parent().unwrap_or(sdf_dir);
        if let Some(slash_pos) = rest.find('/') {
            let rel_path = &rest[slash_pos + 1..];
            package_dir.join(rel_path)
        } else {
            package_dir.join(rest)
        }
    } else if let Some(rest) = uri.strip_prefix("model://") {
        sdf_dir.join(rest)
    } else if let Some(rest) = uri.strip_prefix("file://") {
        PathBuf::from(rest)
    } else {
        sdf_dir.join(uri)
    }
}
