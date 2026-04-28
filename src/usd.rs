//! USD ASCII (.usda) export for robot models.
//!
//! Generates a `.usda` file conforming to the USD specification
//! (<https://graphics.pixar.com/usd/docs/index.html>).
//! The output is suitable for loading in NVIDIA Isaac Sim / Omniverse.

use std::collections::HashMap;

use nalgebra as na;

use crate::robot::*;

// =========================================================================
//  Utility helpers
// =========================================================================

/// Sanitise a name for use as a USD prim-path component.
/// USD prim names must match `[a-zA-Z_][a-zA-Z0-9_]*`.
fn sanitize_name(name: &str) -> String {
    let mut out = String::with_capacity(name.len());
    for (i, c) in name.chars().enumerate() {
        if c.is_ascii_alphanumeric() || c == '_' {
            if i == 0 && c.is_ascii_digit() {
                out.push('_');
            }
            out.push(c);
        } else {
            out.push('_');
        }
    }
    if out.is_empty() {
        out.push_str("prim");
    }
    out
}

/// Hash a colour for material de-duplication.
fn color_key(color: &[f32; 4]) -> u64 {
    let r = (color[0] * 10000.0) as u64;
    let g = (color[1] * 10000.0) as u64;
    let b = (color[2] * 10000.0) as u64;
    let a = (color[3] * 10000.0) as u64;
    r | (g << 16) | (b << 32) | (a << 48)
}

/// Format `(x, y, z)`.
fn fmt_f3(x: f32, y: f32, z: f32) -> String {
    format!("({}, {}, {})", x, y, z)
}

/// Format `double3` value `(x, y, z)` with double-precision.
fn fmt_d3(x: f64, y: f64, z: f64) -> String {
    format!("({}, {}, {})", x, y, z)
}

/// Format a quaternion as USD `quatf (w, x, y, z)` or `quatd`.
fn fmt_quat(q: &na::UnitQuaternion<f32>) -> String {
    format!("({}, {}, {}, {})", q.w, q.i, q.j, q.k)
}

/// Determine which USD principal axis to use for a URDF joint axis.
///
/// Returns `(axis_token, extra_rotation)`.
/// `extra_rotation` aligns the chosen USD axis with the URDF axis direction
/// and must be composed with the joint origin rotation.
fn determine_usd_axis(axis: &na::Vector3<f32>) -> (&'static str, na::UnitQuaternion<f32>) {
    let a = if axis.norm() > 1e-6 {
        axis.normalize()
    } else {
        na::Vector3::z() // fallback
    };
    let eps = 1e-3;

    if (a.x.abs() - 1.0).abs() < eps && a.y.abs() < eps && a.z.abs() < eps {
        if a.x > 0.0 {
            ("X", na::UnitQuaternion::identity())
        } else {
            ("X", na::UnitQuaternion::from_axis_angle(
                &na::Vector3::z_axis(),
                std::f32::consts::PI,
            ))
        }
    } else if a.x.abs() < eps && (a.y.abs() - 1.0).abs() < eps && a.z.abs() < eps {
        // Fix: check y component
        if a.y > 0.0 {
            ("Y", na::UnitQuaternion::identity())
        } else {
            ("Y", na::UnitQuaternion::from_axis_angle(
                &na::Vector3::z_axis(),
                std::f32::consts::PI,
            ))
        }
    } else if a.x.abs() < eps && a.y.abs() < eps && (a.z.abs() - 1.0).abs() < eps {
        if a.z > 0.0 {
            ("Z", na::UnitQuaternion::identity())
        } else {
            ("Z", na::UnitQuaternion::from_axis_angle(
                &na::Vector3::x_axis(),
                std::f32::consts::PI,
            ))
        }
    } else {
        // Arbitrary axis — align Z to the URDF axis.
        let rot = na::UnitQuaternion::rotation_between(&na::Vector3::z(), &a)
            .unwrap_or(na::UnitQuaternion::identity());
        ("Z", rot)
    }
}

// =========================================================================
//  Writers
// =========================================================================

/// Write `xformOp:translate` + `xformOp:orient` + `xformOpOrder` for an
/// isometry.  Omits identity components for tidier output.
fn write_xform_ops(s: &mut String, iso: &na::Isometry3<f32>, indent: &str) {
    let t = iso.translation;
    let q = iso.rotation;
    let has_t =
        t.x.abs() > 1e-7 || t.y.abs() > 1e-7 || t.z.abs() > 1e-7;
    let has_r = (q.w - 1.0).abs() > 1e-7
        || q.i.abs() > 1e-7
        || q.j.abs() > 1e-7
        || q.k.abs() > 1e-7;

    if has_t {
        s.push_str(&format!(
            "{}double3 xformOp:translate = {}\n",
            indent,
            fmt_d3(t.x as f64, t.y as f64, t.z as f64)
        ));
    }
    if has_r {
        s.push_str(&format!(
            "{}quatd xformOp:orient = {}\n",
            indent,
            fmt_quat(&q)
        ));
    }
    if has_t || has_r {
        let mut ops: Vec<&str> = Vec::new();
        if has_t {
            ops.push("\"xformOp:translate\"");
        }
        if has_r {
            ops.push("\"xformOp:orient\"");
        }
        s.push_str(&format!(
            "{}uniform token[] xformOpOrder = [{}]\n",
            indent,
            ops.join(", ")
        ));
    }
}

/// Write `xformOps` for a visual/collision origin, optionally including a
/// scale component (used for Box geometry expressed via UsdGeom.Cube).
fn write_geom_xform_ops(
    s: &mut String,
    iso: &na::Isometry3<f32>,
    scale: Option<(f64, f64, f64)>,
    indent: &str,
) {
    let t = iso.translation;
    let q = iso.rotation;
    let has_t =
        t.x.abs() > 1e-7 || t.y.abs() > 1e-7 || t.z.abs() > 1e-7;
    let has_r = (q.w - 1.0).abs() > 1e-7
        || q.i.abs() > 1e-7
        || q.j.abs() > 1e-7
        || q.k.abs() > 1e-7;
    let has_s = scale.is_some();

    if has_t {
        s.push_str(&format!(
            "{}double3 xformOp:translate = {}\n",
            indent,
            fmt_d3(t.x as f64, t.y as f64, t.z as f64)
        ));
    }
    if has_r {
        s.push_str(&format!(
            "{}quatd xformOp:orient = {}\n",
            indent,
            fmt_quat(&q)
        ));
    }
    if let Some((sx, sy, sz)) = scale {
        s.push_str(&format!(
            "{}double3 xformOp:scale = {}\n",
            indent,
            fmt_d3(sx, sy, sz)
        ));
    }

    if has_t || has_r || has_s {
        let mut ops: Vec<&str> = Vec::new();
        if has_t {
            ops.push("\"xformOp:translate\"");
        }
        if has_r {
            ops.push("\"xformOp:orient\"");
        }
        if has_s {
            ops.push("\"xformOp:scale\"");
        }
        s.push_str(&format!(
            "{}uniform token[] xformOpOrder = [{}]\n",
            indent,
            ops.join(", ")
        ));
    }
}

/// Write geometry prim attributes for a given `GeomData`.
/// Returns the USD prim type name (e.g. `"Cube"`, `"Cylinder"`, `"Mesh"`).
fn write_geom_prim(
    s: &mut String,
    geom: &GeomData,
    origin: &na::Isometry3<f32>,
    name: &str,
    indent: &str,
    api_schemas: &str, // e.g. empty or "\"PhysicsCollisionAPI\""
    material_path: Option<&str>,
) {
    let (prim_type, body) = match geom {
        GeomData::Box { hx, hy, hz } => {
            let mut b = String::new();
            write_geom_xform_ops(
                &mut b,
                origin,
                Some((*hx as f64, *hy as f64, *hz as f64)),
                &format!("{indent}    "),
            );
            b.push_str(&format!("{}    double size = 2.0\n", indent));
            ("Cube", b)
        }
        GeomData::Cylinder {
            radius,
            half_length,
        } => {
            let mut b = String::new();
            write_geom_xform_ops(&mut b, origin, None, &format!("{indent}    "));
            b.push_str(&format!(
                "{}    double radius = {}\n",
                indent, *radius as f64
            ));
            b.push_str(&format!(
                "{}    double height = {}\n",
                indent,
                (*half_length * 2.0) as f64
            ));
            b.push_str(&format!("{}    token axis = \"Z\"\n", indent));
            ("Cylinder", b)
        }
        GeomData::Sphere { radius } => {
            let mut b = String::new();
            write_geom_xform_ops(&mut b, origin, None, &format!("{indent}    "));
            b.push_str(&format!(
                "{}    double radius = {}\n",
                indent, *radius as f64
            ));
            ("Sphere", b)
        }
        GeomData::Capsule { radius, half_length } => {
            let mut b = String::new();
            write_geom_xform_ops(&mut b, origin, None, &format!("{indent}    "));
            b.push_str(&format!(
                "{}    double radius = {}\n",
                indent, *radius as f64
            ));
            b.push_str(&format!(
                "{}    double height = {}\n",
                indent,
                (*half_length * 2.0 + *radius * 2.0) as f64
            ));
            b.push_str(&format!("{}    token axis = \"Z\"\n", indent));
            ("Capsule", b)
        }
        GeomData::Mesh { vertices, .. } => {
            let mut b = String::new();
            write_geom_xform_ops(&mut b, origin, None, &format!("{indent}    "));
            write_mesh_data(&mut b, vertices, &format!("{indent}    "));
            ("Mesh", b)
        }
    };

    // Opening
    if api_schemas.is_empty() {
        s.push_str(&format!(
            "{}def {} \"{}\"\n{}{{\n",
            indent, prim_type, name, indent
        ));
    } else {
        s.push_str(&format!(
            "{}def {} \"{}\" (\n",
            indent, prim_type, name
        ));
        s.push_str(&format!(
            "{}    prepend apiSchemas = [{}]\n",
            indent, api_schemas
        ));
        s.push_str(&format!("{})\n{}{{\n", indent, indent));
    }

    s.push_str(&body);

    // Material binding
    if let Some(mat_path) = material_path {
        s.push_str(&format!(
            "{}    rel material:binding = <{}>\n",
            indent, mat_path
        ));
    }

    s.push_str(&format!("{}}}\n\n", indent));
}

/// Write inline mesh data (points, normals, face indices).
fn write_mesh_data(s: &mut String, vertices: &[f32], indent: &str) {
    let num_verts = vertices.len() / 6;
    if num_verts == 0 {
        return;
    }
    let num_faces = num_verts / 3;

    // Points
    s.push_str(&format!("{}point3f[] points = [\n", indent));
    for i in 0..num_verts {
        let x = vertices[i * 6];
        let y = vertices[i * 6 + 1];
        let z = vertices[i * 6 + 2];
        let comma = if i + 1 < num_verts { "," } else { "" };
        s.push_str(&format!("{}    ({}, {}, {}){}\n", indent, x, y, z, comma));
    }
    s.push_str(&format!("{}]\n", indent));

    // Normals
    s.push_str(&format!("{}normal3f[] normals = [\n", indent));
    for i in 0..num_verts {
        let nx = vertices[i * 6 + 3];
        let ny = vertices[i * 6 + 4];
        let nz = vertices[i * 6 + 5];
        let comma = if i + 1 < num_verts { "," } else { "" };
        s.push_str(&format!("{}    ({}, {}, {}){}\n", indent, nx, ny, nz, comma));
    }
    s.push_str(&format!("{}]\n", indent));
    s.push_str(&format!(
        "{}uniform token normals:interpolation = \"vertex\"\n",
        indent
    ));

    // Face vertex counts (all triangles)
    s.push_str(&format!("{}int[] faceVertexCounts = [", indent));
    for i in 0..num_faces {
        if i > 0 {
            s.push_str(", ");
        }
        s.push_str("3");
    }
    s.push_str("]\n");

    // Face vertex indices (sequential)
    s.push_str(&format!("{}int[] faceVertexIndices = [", indent));
    for i in 0..(num_verts as u32) {
        if i > 0 {
            s.push_str(", ");
        }
        s.push_str(&i.to_string());
    }
    s.push_str("]\n");

    s.push_str(&format!(
        "{}uniform token subdivisionScheme = \"none\"\n",
        indent
    ));
}

/// Write a link prim with physics APIs, visuals, and collisions.
fn write_link(
    s: &mut String,
    link: &LinkData,
    rest_tf: &na::Isometry3<f32>,
    robot_path: &str,
    material_map: &HashMap<u64, usize>,
    indent: &str,
    // Other links this link's collision should be filtered against (i.e.
    // disabled-pair partners). Emitted as `rel physics:filteredPairs`.
    filter_partners: &[String],
) {
    let link_name = sanitize_name(&link.name);

    // Prim header with physics APIs
    s.push_str(&format!("{}def Xform \"{}\" (\n", indent, link_name));
    let api_schemas = if filter_partners.is_empty() {
        "[\"PhysicsRigidBodyAPI\", \"PhysicsMassAPI\"]"
    } else {
        "[\"PhysicsRigidBodyAPI\", \"PhysicsMassAPI\", \"PhysicsFilteredPairsAPI\"]"
    };
    s.push_str(&format!(
        "{}    prepend apiSchemas = {}\n",
        indent, api_schemas
    ));
    s.push_str(&format!("{})\n{}{{\n", indent, indent));

    let inner = format!("{indent}    ");

    // World transform (rest pose)
    write_xform_ops(s, rest_tf, &inner);

    // Filtered pair targets: USD uses absolute prim paths.
    if !filter_partners.is_empty() {
        let paths: Vec<String> = filter_partners
            .iter()
            .map(|n| format!("<{}/{}>", robot_path, sanitize_name(n)))
            .collect();
        s.push_str(&format!(
            "{}rel physics:filteredPairs = [{}]\n",
            inner,
            paths.join(", "),
        ));
    }

    // Mass / inertia
    let mass = link.inertial.mass;
    s.push_str(&format!("{}float physics:mass = {}\n", inner, mass));

    // Diagonal inertia (Ixx, Iyy, Izz)
    s.push_str(&format!(
        "{}float3 physics:diagonalInertia = {}\n",
        inner,
        fmt_f3(
            link.inertial.ixx as f32,
            link.inertial.iyy as f32,
            link.inertial.izz as f32
        )
    ));

    // Center of mass from inertial origin
    let com = link.inertial.origin.translation;
    if com.x.abs() > 1e-7 || com.y.abs() > 1e-7 || com.z.abs() > 1e-7 {
        s.push_str(&format!(
            "{}point3f physics:centerOfMass = {}\n",
            inner,
            fmt_f3(com.x, com.y, com.z)
        ));
    }

    s.push('\n');

    // ----- Visuals -----
    if !link.visuals.is_empty() {
        s.push_str(&format!("{}def Scope \"visuals\"\n{}{{\n", inner, inner));
        let vis_indent = format!("{inner}    ");
        for (i, vis) in link.visuals.iter().enumerate() {
            let mat_idx = material_map
                .get(&color_key(&vis.color))
                .copied()
                .unwrap_or(0);
            let mat_path =
                format!("{robot_path}/Materials/material_{mat_idx}");
            write_geom_prim(
                s,
                &vis.geometry,
                &vis.origin,
                &format!("visual_{i}"),
                &vis_indent,
                "",
                Some(&mat_path),
            );
        }
        s.push_str(&format!("{}}}\n\n", inner));
    }

    // ----- Collisions -----
    if !link.collisions.is_empty() {
        s.push_str(&format!("{}def Scope \"collisions\"\n{}{{\n", inner, inner));
        let col_indent = format!("{inner}    ");
        for (i, col) in link.collisions.iter().enumerate() {
            write_geom_prim(
                s,
                &col.geometry,
                &col.origin,
                &format!("collision_{i}"),
                &col_indent,
                "\"PhysicsCollisionAPI\"",
                None,
            );
        }
        s.push_str(&format!("{}}}\n\n", inner));
    }

    s.push_str(&format!("{}}}\n\n", indent)); // close link
}

/// Write a physics joint prim.
fn write_joint(
    s: &mut String,
    joint: &JointData,
    robot_path: &str,
    indent: &str,
) {
    let joint_name = sanitize_name(&joint.name);
    let parent_name = sanitize_name(&joint.parent_link);
    let child_name = sanitize_name(&joint.child_link);

    let (usd_prim_type, drive_kind) = match joint.joint_type.as_str() {
        "revolute" | "continuous" => ("PhysicsRevoluteJoint", Some("angular")),
        "prismatic" => ("PhysicsPrismaticJoint", Some("linear")),
        _ => ("PhysicsFixedJoint", None), // fixed and others
    };

    // API schemas
    let api_list = if let Some(dk) = drive_kind {
        format!("\"PhysicsDriveAPI:{}\"", dk)
    } else {
        String::new()
    };

    if api_list.is_empty() {
        s.push_str(&format!(
            "{}def {} \"{}\"\n{}{{\n",
            indent, usd_prim_type, joint_name, indent
        ));
    } else {
        s.push_str(&format!(
            "{}def {} \"{}\" (\n",
            indent, usd_prim_type, joint_name
        ));
        s.push_str(&format!(
            "{}    prepend apiSchemas = [{}]\n",
            indent, api_list
        ));
        s.push_str(&format!("{})\n{}{{\n", indent, indent));
    }

    let inner = format!("{indent}    ");

    // Body relationships
    s.push_str(&format!(
        "{}rel physics:body0 = <{}/{}>\n",
        inner, robot_path, parent_name
    ));
    s.push_str(&format!(
        "{}rel physics:body1 = <{}/{}>\n",
        inner, robot_path, child_name
    ));

    // Axis (for revolute / prismatic)
    if joint.joint_type != "fixed" {
        let (usd_axis, extra_rot) = determine_usd_axis(&joint.axis);
        s.push_str(&format!(
            "{}uniform token physics:axis = \"{}\"\n",
            inner, usd_axis
        ));

        // Local transforms
        let t = joint.origin.translation;
        let local_rot0 = joint.origin.rotation * extra_rot;
        s.push_str(&format!(
            "{}point3f physics:localPos0 = {}\n",
            inner,
            fmt_f3(t.x, t.y, t.z)
        ));
        s.push_str(&format!(
            "{}quatf physics:localRot0 = {}\n",
            inner,
            fmt_quat(&local_rot0)
        ));
        s.push_str(&format!(
            "{}point3f physics:localPos1 = (0, 0, 0)\n",
            inner
        ));
        s.push_str(&format!(
            "{}quatf physics:localRot1 = {}\n",
            inner,
            fmt_quat(&extra_rot)
        ));

        // Limits (revolute in degrees, prismatic in metres)
        match joint.joint_type.as_str() {
            "revolute" => {
                let lower_deg = joint.lower.to_degrees();
                let upper_deg = joint.upper.to_degrees();
                s.push_str(&format!(
                    "{}float physics:lowerLimit = {}\n",
                    inner, lower_deg
                ));
                s.push_str(&format!(
                    "{}float physics:upperLimit = {}\n",
                    inner, upper_deg
                ));
            }
            "continuous" => {
                // No hard limits for continuous joints
                s.push_str(&format!(
                    "{}float physics:lowerLimit = -360\n",
                    inner
                ));
                s.push_str(&format!(
                    "{}float physics:upperLimit = 360\n",
                    inner
                ));
            }
            "prismatic" => {
                s.push_str(&format!(
                    "{}float physics:lowerLimit = {}\n",
                    inner, joint.lower
                ));
                s.push_str(&format!(
                    "{}float physics:upperLimit = {}\n",
                    inner, joint.upper
                ));
            }
            _ => {}
        }

        // Drive properties
        if let Some(dk) = drive_kind {
            s.push_str(&format!(
                "{}float drive:{}:physics:damping = 1000\n",
                inner, dk
            ));
            s.push_str(&format!(
                "{}float drive:{}:physics:stiffness = 10000\n",
                inner, dk
            ));
            s.push_str(&format!(
                "{}token drive:{}:physics:type = \"force\"\n",
                inner, dk
            ));
        }
    } else {
        // Fixed joint — just need local transforms
        let t = joint.origin.translation;
        let q = joint.origin.rotation;
        s.push_str(&format!(
            "{}point3f physics:localPos0 = {}\n",
            inner,
            fmt_f3(t.x, t.y, t.z)
        ));
        s.push_str(&format!(
            "{}quatf physics:localRot0 = {}\n",
            inner,
            fmt_quat(&q)
        ));
        s.push_str(&format!(
            "{}point3f physics:localPos1 = (0, 0, 0)\n",
            inner
        ));
        s.push_str(&format!(
            "{}quatf physics:localRot1 = (1, 0, 0, 0)\n",
            inner
        ));
    }

    s.push_str(&format!("{}}}\n\n", indent)); // close joint
}

/// Write a UsdPreviewSurface material.
fn write_material(
    s: &mut String,
    idx: usize,
    color: &[f32; 4],
    robot_path: &str,
    indent: &str,
) {
    let mat_name = format!("material_{idx}");
    let inner = format!("{indent}    ");
    let shader_indent = format!("{inner}    ");

    s.push_str(&format!("{}def Material \"{}\"\n{}{{\n", indent, mat_name, indent));
    s.push_str(&format!(
        "{}token outputs:surface.connect = <{}/Materials/{}/PBRShader.outputs:surface>\n",
        inner,
        robot_path,
        mat_name
    ));

    // Shader
    s.push_str(&format!(
        "{}def Shader \"PBRShader\"\n{}{{\n",
        inner, inner
    ));
    s.push_str(&format!(
        "{}uniform token info:id = \"UsdPreviewSurface\"\n",
        shader_indent
    ));
    s.push_str(&format!(
        "{}color3f inputs:diffuseColor = ({}, {}, {})\n",
        shader_indent, color[0], color[1], color[2]
    ));
    if (color[3] - 1.0).abs() > 1e-4 {
        s.push_str(&format!(
            "{}float inputs:opacity = {}\n",
            shader_indent, color[3]
        ));
    }
    s.push_str(&format!(
        "{}float inputs:metallic = 0\n",
        shader_indent
    ));
    s.push_str(&format!(
        "{}float inputs:roughness = 0.5\n",
        shader_indent
    ));
    s.push_str(&format!(
        "{}token outputs:surface\n",
        shader_indent
    ));
    s.push_str(&format!("{}}}\n", inner)); // close Shader
    s.push_str(&format!("{}}}\n\n", indent)); // close Material
}

// =========================================================================
//  Public API
// =========================================================================

/// Export the robot model as a USD ASCII (.usda) string.
pub fn export_usda(model: &RobotModel) -> String {
    let mut s = String::with_capacity(16 * 1024);
    let robot_name = sanitize_name(&model.name);
    let robot_path = format!("/World/{robot_name}");

    // ---- Header ----
    s.push_str("#usda 1.0\n");
    s.push_str("(\n");
    s.push_str("    defaultPrim = \"World\"\n");
    s.push_str(&format!(
        "    doc = \"Generated by Articara — {}\"\n",
        model.name
    ));
    s.push_str("    metersPerUnit = 1.0\n");
    s.push_str("    upAxis = \"Z\"\n");
    s.push_str(")\n\n");

    // ---- Compute rest-pose world transforms ----
    let rest_transforms = model.compute_transforms();

    // ---- Collect unique materials ----
    let mut materials: Vec<[f32; 4]> = Vec::new();
    let mut material_map: HashMap<u64, usize> = HashMap::new();
    for link in &model.links {
        for vis in &link.visuals {
            let key = color_key(&vis.color);
            if !material_map.contains_key(&key) {
                let idx = materials.len();
                material_map.insert(key, idx);
                materials.push(vis.color);
            }
        }
    }

    // ---- World scope ----
    s.push_str("def Xform \"World\"\n{\n");

    // Physics scene
    s.push_str("    def PhysicsScene \"PhysicsScene\"\n    {\n");
    s.push_str("        vector3f physics:gravityDirection = (0, 0, -1)\n");
    s.push_str("        float physics:gravityMagnitude = 9.81\n");
    s.push_str("    }\n\n");

    // ---- Robot root prim ----
    s.push_str(&format!("    def Xform \"{}\" (\n", robot_name));
    s.push_str(
        "        prepend apiSchemas = [\"PhysicsArticulationRootAPI\"]\n",
    );
    s.push_str("    )\n    {\n");

    // Build the per-link "filter against" list once so each call to
    // write_link can attach a `physics:filteredPairs` rel for any disabled
    // collision pairs it participates in. We list each pair only on the
    // alphabetically-first link (which matches our normalised storage)
    // since `filteredPairs` is symmetric in USD physics semantics.
    let mut filter_map: HashMap<String, Vec<String>> = HashMap::new();
    for cp in &model.collision_pairs {
        if cp.enabled {
            continue;
        }
        if !model.link_map.contains_key(&cp.link_a) || !model.link_map.contains_key(&cp.link_b) {
            continue;
        }
        filter_map
            .entry(cp.link_a.clone())
            .or_default()
            .push(cp.link_b.clone());
    }

    // Links
    for link in &model.links {
        let tf = rest_transforms
            .get(&link.name)
            .copied()
            .unwrap_or(na::Isometry3::identity());
        let partners = filter_map
            .get(&link.name)
            .map(|v| v.as_slice())
            .unwrap_or(&[]);
        write_link(
            &mut s,
            link,
            &tf,
            &robot_path,
            &material_map,
            "        ",
            partners,
        );
    }

    // Joints
    for joint in &model.joints {
        write_joint(&mut s, joint, &robot_path, "        ");
    }

    // Materials
    if !materials.is_empty() {
        s.push_str("        def Scope \"Materials\"\n        {\n");
        for (i, color) in materials.iter().enumerate() {
            write_material(&mut s, i, color, &robot_path, "            ");
        }
        s.push_str("        }\n");
    }

    s.push_str("    }\n"); // close robot
    s.push_str("}\n"); // close World

    s
}

/// Export a USDA file to the given directory.
///
/// Writes `{model.name}.usda` inside `output_dir`.
pub fn export_usda_to_dir(
    model: &RobotModel,
    output_dir: &std::path::Path,
) -> Result<std::path::PathBuf, String> {
    std::fs::create_dir_all(output_dir)
        .map_err(|e| format!("Create dir: {e}"))?;

    let filename = format!("{}.usda", sanitize_name(&model.name));
    let path = output_dir.join(&filename);
    let usda = export_usda(model);
    std::fs::write(&path, &usda).map_err(|e| format!("Write USDA: {e}"))?;

    log::info!("USD ASCII export: {:?}", path);
    Ok(path)
}

// =========================================================================
//  Tests
// =========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sanitize_various_names() {
        assert_eq!(sanitize_name("base_link"), "base_link");
        assert_eq!(sanitize_name("link-1"), "link_1");
        assert_eq!(sanitize_name("123abc"), "_123abc");
        assert_eq!(sanitize_name(""), "prim");
        assert_eq!(sanitize_name("my link!"), "my_link_");
    }

    #[test]
    fn color_key_different_for_different_colors() {
        let a = color_key(&[1.0, 0.0, 0.0, 1.0]);
        let b = color_key(&[0.0, 1.0, 0.0, 1.0]);
        assert_ne!(a, b);
    }

    #[test]
    fn determine_axis_z() {
        let (axis, rot) = determine_usd_axis(&na::Vector3::new(0.0, 0.0, 1.0));
        assert_eq!(axis, "Z");
        assert!((rot.w - 1.0).abs() < 1e-4); // identity
    }

    #[test]
    fn determine_axis_x() {
        let (axis, _rot) = determine_usd_axis(&na::Vector3::new(1.0, 0.0, 0.0));
        assert_eq!(axis, "X");
    }

    #[test]
    fn determine_axis_arbitrary() {
        let (axis, rot) =
            determine_usd_axis(&na::Vector3::new(0.0, 0.707, 0.707));
        assert_eq!(axis, "Z");
        // The extra rotation should map Z to (0, 0.707, 0.707)
        let mapped = rot * na::Vector3::z();
        let target = na::Vector3::new(0.0, 0.707, 0.707).normalize();
        assert!((mapped - target).norm() < 1e-3);
    }

    #[test]
    fn export_empty_model() {
        let model = RobotModel::new_empty("test_robot");
        let usda = export_usda(&model);
        assert!(usda.starts_with("#usda 1.0"));
        assert!(usda.contains("defaultPrim = \"World\""));
        assert!(usda.contains("upAxis = \"Z\""));
        assert!(usda.contains("def Xform \"test_robot\""));
        assert!(usda.contains("PhysicsArticulationRootAPI"));
        assert!(usda.contains("def Xform \"base_link\""));
        assert!(usda.contains("PhysicsRigidBodyAPI"));
        assert!(usda.contains("PhysicsMassAPI"));
        assert!(usda.contains("def Cube \"visual_0\""));
        assert!(usda.contains("double size = 2.0"));
        assert!(usda.contains("def Material \"material_0\""));
        assert!(usda.contains("UsdPreviewSurface"));
    }
}
