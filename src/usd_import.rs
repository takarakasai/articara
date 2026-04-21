//! USD ASCII (.usda) import for robot models.
//!
//! Parses `.usda` files that follow our export conventions (see `usd.rs`)
//! and reconstructs a `RobotModel`.  The parser also handles typical
//! USDA files from other sources (e.g. Isaac Sim), although not all USD
//! features are supported.

use nalgebra as na;
use std::collections::{HashMap, HashSet};
use std::path::Path;

use crate::robot::*;

// =========================================================================
//  Tokeniser / Prim parser
// =========================================================================

/// A parsed USD prim with its type, name, properties, and children.
#[derive(Debug, Clone)]
struct UsdPrim {
    prim_type: String,       // e.g. "Xform", "Cube", "PhysicsRevoluteJoint"
    name: String,            // prim name
    api_schemas: Vec<String>,
    props: HashMap<String, String>, // property name → raw value string
    children: Vec<UsdPrim>,
}

/// Parse a USDA text into a tree of `UsdPrim`s.
fn parse_usda(text: &str) -> Vec<UsdPrim> {
    let lines: Vec<&str> = text.lines().collect();
    let mut pos = 0;
    // Skip header
    while pos < lines.len() {
        let trimmed = lines[pos].trim();
        if trimmed.starts_with("def ") {
            break;
        }
        pos += 1;
    }
    let mut prims = Vec::new();
    while pos < lines.len() {
        if let Some((prim, next)) = parse_prim(&lines, pos) {
            prims.push(prim);
            pos = next;
        } else {
            pos += 1;
        }
    }
    prims
}

/// Parse a single prim starting at `start`. Returns the parsed prim and the
/// line index after the closing brace.
fn parse_prim(lines: &[&str], start: usize) -> Option<(UsdPrim, usize)> {
    let trimmed = lines[start].trim();
    // Match: def <Type> "<Name>" ...
    if !trimmed.starts_with("def ") {
        return None;
    }
    let rest = &trimmed[4..];
    let (prim_type, rest) = split_first_word(rest);
    let name = extract_quoted(rest).unwrap_or_default();

    let mut api_schemas = Vec::new();
    let mut pos = start + 1;

    // Check for metadata block ( ... ) before {
    // The opening line might end with `(`, or the `(` is on its own line, or
    // the prim body `{` is on the opening line.
    let has_metadata = rest.contains('(') && !rest.contains('{');
    if has_metadata {
        // Read metadata lines until we hit `)`
        while pos < lines.len() {
            let t = lines[pos].trim();
            if t.contains("apiSchemas") {
                // Collect API schema names
                let combined = collect_bracket_content(lines, &mut pos, '[', ']');
                for part in combined.split(',') {
                    let s = part.trim().trim_matches('"').trim().to_string();
                    if !s.is_empty() {
                        api_schemas.push(s);
                    }
                }
                continue;
            }
            if t.starts_with(')') || t.ends_with(')') {
                pos += 1;
                break;
            }
            pos += 1;
        }
    }

    // Find opening brace
    while pos < lines.len() {
        let t = lines[pos].trim();
        if t.starts_with('{') || t == "{" {
            pos += 1;
            break;
        }
        // The opening def line might already contain {
        if lines.get(start).map_or(false, |l| l.contains('{')) && pos == start + 1 {
            break;
        }
        pos += 1;
    }

    let mut props = HashMap::new();
    let mut children = Vec::new();

    // Parse contents until matching }
    let mut depth = 1u32;
    while pos < lines.len() && depth > 0 {
        let t = lines[pos].trim();

        if t == "}" || t == "}," {
            depth -= 1;
            if depth == 0 {
                pos += 1;
                break;
            }
            pos += 1;
            continue;
        }

        // Nested prim?
        if t.starts_with("def ") {
            if let Some((child, next)) = parse_prim(lines, pos) {
                children.push(child);
                pos = next;
                continue;
            }
        }

        // Skip lines that only open/close braces deeper
        if t == "{" {
            depth += 1;
            pos += 1;
            continue;
        }

        // Property — key = value
        if let Some((key, value)) = parse_property_line(lines, &mut pos) {
            props.insert(key, value);
        } else {
            pos += 1;
        }
    }

    Some((
        UsdPrim {
            prim_type,
            name,
            api_schemas,
            props,
            children,
        },
        pos,
    ))
}

/// Split a string at the first whitespace.
fn split_first_word(s: &str) -> (String, &str) {
    let s = s.trim();
    if let Some(idx) = s.find(|c: char| c.is_whitespace()) {
        (s[..idx].to_string(), &s[idx..])
    } else {
        (s.to_string(), "")
    }
}

/// Extract the first double-quoted string.
fn extract_quoted(s: &str) -> Option<String> {
    let start = s.find('"')? + 1;
    let end = s[start..].find('"')? + start;
    Some(s[start..end].to_string())
}

/// Parse a property line: `[qualifiers] type key = value`
/// Handles multi-line arrays [ ... ] by reading ahead.
fn parse_property_line(lines: &[&str], pos: &mut usize) -> Option<(String, String)> {
    let line = lines[*pos].trim();
    // Skip blank lines, comments
    if line.is_empty() || line.starts_with('#') || line.starts_with("//") {
        return None;
    }
    // Must contain `=` or `rel ... = <...>` or `... .connect = <...>`
    if !line.contains('=') {
        return None;
    }

    // Split at first `=`
    let eq_idx = line.find('=')?;
    let lhs = line[..eq_idx].trim();
    let rhs_raw = line[eq_idx + 1..].trim();

    // Extract property key: last word before `=`
    let key = extract_prop_key(lhs);

    // Handle multi-line arrays
    if rhs_raw.starts_with('[') && !rhs_raw.contains(']') {
        let value = collect_bracket_content(lines, pos, '[', ']');
        return Some((key, value));
    }

    *pos += 1;
    Some((key, rhs_raw.to_string()))
}

/// Extract the property key from the LHS of an assignment.
/// E.g. "float physics:mass" → "physics:mass"
///      "rel material:binding" → "material:binding"
///      "uniform token physics:axis" → "physics:axis"
fn extract_prop_key(lhs: &str) -> String {
    // Take the last space-separated token
    lhs.split_whitespace().last().unwrap_or(lhs).to_string()
}

/// Collect content within matched brackets, possibly across multiple lines.
fn collect_bracket_content(lines: &[&str], pos: &mut usize, open: char, close: char) -> String {
    let mut result = String::new();
    let mut depth = 0i32;
    while *pos < lines.len() {
        let line = lines[*pos].trim();
        for c in line.chars() {
            if c == open {
                depth += 1;
            } else if c == close {
                depth -= 1;
            }
        }
        result.push_str(line);
        result.push(' ');
        *pos += 1;
        if depth <= 0 {
            break;
        }
    }
    // Strip outer brackets
    let trimmed = result.trim();
    if let (Some(s), Some(e)) = (trimmed.find(open), trimmed.rfind(close)) {
        trimmed[s + 1..e].to_string()
    } else {
        trimmed.to_string()
    }
}

// =========================================================================
//  Value parsers
// =========================================================================

/// Parse a float from a USD value string, stripping type prefixes.
fn parse_float(s: &str) -> f32 {
    let s = s.trim();
    // Could start with a number directly or be in parens
    s.parse::<f32>().unwrap_or(0.0)
}

/// Parse a double from a USD value string.
fn parse_f64(s: &str) -> f64 {
    s.trim().parse::<f64>().unwrap_or(0.0)
}

/// Parse a tuple `(x, y, z)` into 3 f32 values.
fn parse_f3(s: &str) -> (f32, f32, f32) {
    let s = s.trim().trim_start_matches('(').trim_end_matches(')');
    let parts: Vec<f32> = s.split(',').map(|p| p.trim().parse().unwrap_or(0.0)).collect();
    (
        parts.first().copied().unwrap_or(0.0),
        parts.get(1).copied().unwrap_or(0.0),
        parts.get(2).copied().unwrap_or(0.0),
    )
}

/// Parse a quaternion `(w, x, y, z)` → UnitQuaternion.
fn parse_quat(s: &str) -> na::UnitQuaternion<f32> {
    let s = s.trim().trim_start_matches('(').trim_end_matches(')');
    let parts: Vec<f32> = s.split(',').map(|p| p.trim().parse().unwrap_or(0.0)).collect();
    let w = parts.first().copied().unwrap_or(1.0);
    let x = parts.get(1).copied().unwrap_or(0.0);
    let y = parts.get(2).copied().unwrap_or(0.0);
    let z = parts.get(3).copied().unwrap_or(0.0);
    na::UnitQuaternion::from_quaternion(na::Quaternion::new(w, x, y, z))
}

/// Parse a `point3f[]` or `normal3f[]` array string into Vec<(f32,f32,f32)>.
fn parse_point_array(s: &str) -> Vec<(f32, f32, f32)> {
    let mut result = Vec::new();
    // Find all (x, y, z) tuples
    let mut i = 0;
    let bytes = s.as_bytes();
    while i < bytes.len() {
        if bytes[i] == b'(' {
            if let Some(end) = s[i..].find(')') {
                let tuple = &s[i..i + end + 1];
                result.push(parse_f3(tuple));
                i += end + 1;
            } else {
                break;
            }
        } else {
            i += 1;
        }
    }
    result
}

/// Parse an `int[]` array string into Vec<i32>.
#[allow(dead_code)]
fn parse_int_array(s: &str) -> Vec<i32> {
    let s = s.trim().trim_start_matches('[').trim_end_matches(']');
    s.split(',')
        .filter_map(|p| p.trim().parse::<i32>().ok())
        .collect()
}

/// Extract the last path component from a USD relationship path like
/// `</World/robot/link_name>`.
fn extract_rel_name(s: &str) -> String {
    let s = s.trim().trim_start_matches('<').trim_end_matches('>');
    s.rsplit('/').next().unwrap_or(s).to_string()
}

/// Extract a material path from `rel material:binding = </World/.../material_N>`.
fn extract_material_path(s: &str) -> Option<String> {
    let s = s.trim().trim_start_matches('<').trim_end_matches('>');
    if s.is_empty() {
        None
    } else {
        Some(s.to_string())
    }
}

// =========================================================================
//  Prim → model conversion
// =========================================================================

/// Parse a colour from a material prim.
fn parse_material_color(prim: &UsdPrim) -> [f32; 4] {
    let mut color = [0.7f32, 0.7, 0.7, 1.0];
    // Look for a PBRShader child
    for child in &prim.children {
        if let Some(diff) = child.props.get("inputs:diffuseColor") {
            let (r, g, b) = parse_f3(diff);
            color[0] = r;
            color[1] = g;
            color[2] = b;
        }
        if let Some(opacity) = child.props.get("inputs:opacity") {
            color[3] = parse_float(opacity);
        }
    }
    // Also check the prim itself (flat layout)
    if let Some(diff) = prim.props.get("inputs:diffuseColor") {
        let (r, g, b) = parse_f3(diff);
        color[0] = r;
        color[1] = g;
        color[2] = b;
    }
    if let Some(opacity) = prim.props.get("inputs:opacity") {
        color[3] = parse_float(opacity);
    }
    color
}

/// Convert a geometry prim (Cube/Cylinder/Sphere/Mesh) to GeomData + origin.
fn parse_geom_prim(prim: &UsdPrim) -> (GeomData, na::Isometry3<f32>) {
    let origin = parse_xform(prim);

    let geom = match prim.prim_type.as_str() {
        "Cube" => {
            // Scale encodes half-extents; size is always 2.0
            let (sx, sy, sz) = prim
                .props
                .get("xformOp:scale")
                .map(|s| parse_f3(s))
                .unwrap_or((0.05, 0.05, 0.05));
            // Strip the scale from the origin we already parsed
            GeomData::Box {
                hx: sx.abs(),
                hy: sy.abs(),
                hz: sz.abs(),
            }
        }
        "Cylinder" => {
            let radius = prim
                .props
                .get("radius")
                .map(|s| parse_float(s))
                .unwrap_or(0.02);
            let height = prim
                .props
                .get("height")
                .map(|s| parse_float(s))
                .unwrap_or(0.2);
            GeomData::Cylinder {
                radius,
                half_length: height / 2.0,
            }
        }
        "Sphere" => {
            let radius = prim
                .props
                .get("radius")
                .map(|s| parse_float(s))
                .unwrap_or(0.05);
            GeomData::Sphere { radius }
        }
        "Mesh" => {
            let points = prim
                .props
                .get("points")
                .map(|s| parse_point_array(s))
                .unwrap_or_default();
            let normals = prim
                .props
                .get("normals")
                .map(|s| parse_point_array(s))
                .unwrap_or_default();
            let num_verts = points.len();
            // Build flat vertex buffer [x, y, z, nx, ny, nz, ...]
            let mut vertices = Vec::with_capacity(num_verts * 6);
            for i in 0..num_verts {
                let (px, py, pz) = points[i];
                let (nx, ny, nz) = if i < normals.len() {
                    normals[i]
                } else {
                    (0.0, 0.0, 1.0)
                };
                vertices.push(px);
                vertices.push(py);
                vertices.push(pz);
                vertices.push(nx);
                vertices.push(ny);
                vertices.push(nz);
            }
            GeomData::Mesh {
                vertices,
                filename: None,
                scale: None,
            }
        }
        _ => {
            // Unknown geometry type — fallback to small box
            GeomData::Box {
                hx: 0.01,
                hy: 0.01,
                hz: 0.01,
            }
        }
    };

    // For Cube, the origin's scale component encodes half-extents, so we
    // need to remove the scale from the Isometry (Isometry has no scale).
    // The xform parsed above already only captures translate + orient.
    (geom, origin)
}

/// Parse xformOp:translate + xformOp:orient into an Isometry3.
fn parse_xform(prim: &UsdPrim) -> na::Isometry3<f32> {
    let translation = prim
        .props
        .get("xformOp:translate")
        .map(|s| {
            let (x, y, z) = parse_f3(s);
            na::Translation3::new(x, y, z)
        })
        .unwrap_or_else(|| na::Translation3::new(0.0, 0.0, 0.0));
    let rotation = prim
        .props
        .get("xformOp:orient")
        .map(|s| parse_quat(s))
        .unwrap_or_else(na::UnitQuaternion::identity);
    na::Isometry3::from_parts(translation, rotation)
}

/// Find a child prim by name.
fn find_child<'a>(prim: &'a UsdPrim, name: &str) -> Option<&'a UsdPrim> {
    prim.children.iter().find(|c| c.name == name)
}

/// Check whether a prim looks like a link (has PhysicsRigidBodyAPI or is an
/// Xform with visual/collision scopes).
fn is_link_prim(prim: &UsdPrim) -> bool {
    if prim.prim_type != "Xform" {
        return false;
    }
    prim.api_schemas.iter().any(|s| s.contains("PhysicsRigidBodyAPI"))
        || prim.children.iter().any(|c| c.name == "visuals" || c.name == "collisions")
}

/// Check whether a prim is a physics joint.
fn is_joint_prim(prim: &UsdPrim) -> bool {
    prim.prim_type.contains("PhysicsRevoluteJoint")
        || prim.prim_type.contains("PhysicsPrismaticJoint")
        || prim.prim_type.contains("PhysicsFixedJoint")
        || prim.prim_type.contains("Joint")
}

// =========================================================================
//  Public import API
// =========================================================================

/// Import a `.usda` file and return a `RobotModel`.
pub fn import_usda(path: &Path) -> Result<RobotModel, String> {
    let text = std::fs::read_to_string(path).map_err(|e| format!("Read USDA: {e}"))?;
    import_usda_from_str(&text, Some(path))
}

/// Import from a USDA string. `source_path` is optional metadata.
pub fn import_usda_from_str(
    text: &str,
    source_path: Option<&Path>,
) -> Result<RobotModel, String> {
    // Parse the prim tree
    let top_prims = parse_usda(text);
    if top_prims.is_empty() {
        return Err("No prims found in USDA file".into());
    }

    // Find the World prim (or use the first top-level prim)
    let world = top_prims
        .iter()
        .find(|p| p.name == "World")
        .or_else(|| top_prims.first())
        .ok_or("No World prim found")?;

    // Find the robot root prim (first Xform child of World that has
    // PhysicsArticulationRootAPI, or the first Xform with link children)
    let robot_prim = world
        .children
        .iter()
        .find(|p| {
            p.prim_type == "Xform"
                && (p.api_schemas.iter().any(|s| s.contains("ArticulationRoot"))
                    || p.children.iter().any(|c| is_link_prim(c)))
        })
        .or_else(|| {
            // Fallback: any Xform child that isn't PhysicsScene
            world.children.iter().find(|p| {
                p.prim_type == "Xform" && p.name != "PhysicsScene"
            })
        })
        .ok_or("No robot prim found under World")?;

    let robot_name = robot_prim.name.clone();

    // ---- Collect materials ----
    let mut material_colors: HashMap<String, [f32; 4]> = HashMap::new();
    if let Some(mat_scope) = find_child(robot_prim, "Materials") {
        for mat_prim in &mat_scope.children {
            if mat_prim.prim_type == "Material" {
                let color = parse_material_color(mat_prim);
                // Key: material name (e.g. "material_0")
                material_colors.insert(mat_prim.name.clone(), color);
            }
        }
    }

    // ---- Collect links ----
    let mut links = Vec::new();
    let mut link_map: HashMap<String, usize> = HashMap::new();

    for child in &robot_prim.children {
        if !is_link_prim(child) {
            continue;
        }

        let link_name = child.name.clone();
        let li = links.len();
        link_map.insert(link_name.clone(), li);

        // Rest-pose world transform (we'll compute the local joint origins
        // from this later, but for now just store visuals/collisions in local
        // frame — they are already relative to the link).

        // Mass / Inertial
        let mass = child
            .props
            .get("physics:mass")
            .map(|s| parse_f64(s))
            .unwrap_or(1.0);
        let diag_inertia = child
            .props
            .get("physics:diagonalInertia")
            .map(|s| parse_f3(s))
            .unwrap_or((0.001, 0.001, 0.001));
        let center_of_mass = child
            .props
            .get("physics:centerOfMass")
            .map(|s| {
                let (x, y, z) = parse_f3(s);
                na::Translation3::new(x, y, z)
            })
            .unwrap_or_else(|| na::Translation3::identity());

        let inertial = InertialData {
            origin: na::Isometry3::from_parts(
                center_of_mass,
                na::UnitQuaternion::identity(),
            ),
            mass,
            ixx: diag_inertia.0 as f64,
            ixy: 0.0,
            ixz: 0.0,
            iyy: diag_inertia.1 as f64,
            iyz: 0.0,
            izz: diag_inertia.2 as f64,
        };

        // ---- Visuals ----
        let mut visuals = Vec::new();
        if let Some(vis_scope) = find_child(child, "visuals") {
            for vis_prim in &vis_scope.children {
                let (geom, origin) = parse_geom_prim(vis_prim);
                // Resolve material color
                let color = vis_prim
                    .props
                    .get("material:binding")
                    .and_then(|path| extract_material_path(path))
                    .and_then(|full_path| {
                        // e.g. "/World/robot/Materials/material_0" → "material_0"
                        let mat_name = full_path.rsplit('/').next()?;
                        material_colors.get(mat_name).copied()
                    })
                    .unwrap_or([0.7, 0.7, 0.7, 1.0]);
                visuals.push(VisualData {
                    origin,
                    geometry: geom,
                    color,
                });
            }
        }

        // ---- Collisions ----
        let mut collisions = Vec::new();
        if let Some(col_scope) = find_child(child, "collisions") {
            for col_prim in &col_scope.children {
                let (geom, origin) = parse_geom_prim(col_prim);
                collisions.push(CollisionData {
                    origin,
                    geometry: geom,
                });
            }
        }

        links.push(LinkData {
            name: link_name,
            visuals,
            collisions,
            inertial,
        });
    }

    if links.is_empty() {
        return Err("No links found in USDA file".into());
    }

    // ---- Collect joints ----
    let mut joints = Vec::new();
    let mut joint_map: HashMap<String, usize> = HashMap::new();
    let mut children_joints: HashMap<String, Vec<usize>> = HashMap::new();
    let mut child_links: HashSet<String> = HashSet::new();

    for child in &robot_prim.children {
        if !is_joint_prim(child) {
            continue;
        }

        let joint_name = child.name.clone();
        let parent_link = child
            .props
            .get("physics:body0")
            .map(|s| extract_rel_name(s))
            .unwrap_or_default();
        let child_link = child
            .props
            .get("physics:body1")
            .map(|s| extract_rel_name(s))
            .unwrap_or_default();

        if parent_link.is_empty() || child_link.is_empty() {
            continue; // skip malformed joints
        }

        let joint_type = if child.prim_type.contains("Revolute") {
            // Distinguish revolute from continuous by limits
            let lower = child
                .props
                .get("physics:lowerLimit")
                .map(|s| parse_f64(s))
                .unwrap_or(0.0);
            let upper = child
                .props
                .get("physics:upperLimit")
                .map(|s| parse_f64(s))
                .unwrap_or(0.0);
            if lower <= -360.0 && upper >= 360.0 {
                "continuous".to_string()
            } else {
                "revolute".to_string()
            }
        } else if child.prim_type.contains("Prismatic") {
            "prismatic".to_string()
        } else {
            "fixed".to_string()
        };

        // Axis token
        let usd_axis_str = child
            .props
            .get("physics:axis")
            .map(|s| s.trim().trim_matches('"').to_string())
            .unwrap_or_else(|| "Z".to_string());

        // Local transforms
        let local_pos0 = child
            .props
            .get("physics:localPos0")
            .map(|s| {
                let (x, y, z) = parse_f3(s);
                na::Translation3::new(x, y, z)
            })
            .unwrap_or_else(|| na::Translation3::identity());
        let local_rot0 = child
            .props
            .get("physics:localRot0")
            .map(|s| parse_quat(s))
            .unwrap_or_else(na::UnitQuaternion::identity);
        let local_rot1 = child
            .props
            .get("physics:localRot1")
            .map(|s| parse_quat(s))
            .unwrap_or_else(na::UnitQuaternion::identity);

        // Reconstruct the URDF joint origin from the USD local transforms.
        // On export: localRot0 = origin.rotation * extra_rot
        //            localRot1 = extra_rot
        // So: origin.rotation = localRot0 * inv(localRot1)
        // For fixed joints: localRot1 = identity, so origin.rotation = localRot0
        let origin_rotation = local_rot0 * local_rot1.inverse();
        let origin = na::Isometry3::from_parts(local_pos0, origin_rotation);

        // Reconstruct the URDF axis from the USD axis token and extra_rot.
        // On export: extra_rot maps the URDF axis to the chosen USD principal axis.
        // So: urdf_axis = inv(extra_rot) * usd_principal_axis
        // But localRot1 = extra_rot, so urdf_axis = inv(localRot1) * usd_axis.
        let usd_principal = match usd_axis_str.as_str() {
            "X" => na::Vector3::x(),
            "Y" => na::Vector3::y(),
            _ => na::Vector3::z(),
        };
        let urdf_axis = if joint_type == "fixed" {
            na::Vector3::z() // doesn't matter for fixed
        } else {
            local_rot1.inverse() * usd_principal
        };

        // Limits — revolute stored in degrees, convert back to radians
        let (lower, upper) = if joint_type == "revolute" {
            let lower_deg = child
                .props
                .get("physics:lowerLimit")
                .map(|s| parse_f64(s))
                .unwrap_or(0.0);
            let upper_deg = child
                .props
                .get("physics:upperLimit")
                .map(|s| parse_f64(s))
                .unwrap_or(0.0);
            (lower_deg.to_radians(), upper_deg.to_radians())
        } else if joint_type == "continuous" {
            (
                -std::f64::consts::PI * 2.0,
                std::f64::consts::PI * 2.0,
            )
        } else if joint_type == "prismatic" {
            let lower = child
                .props
                .get("physics:lowerLimit")
                .map(|s| parse_f64(s))
                .unwrap_or(0.0);
            let upper = child
                .props
                .get("physics:upperLimit")
                .map(|s| parse_f64(s))
                .unwrap_or(0.0);
            (lower, upper)
        } else {
            (0.0, 0.0)
        };

        let ji = joints.len();
        joint_map.insert(joint_name.clone(), ji);
        children_joints
            .entry(parent_link.clone())
            .or_default()
            .push(ji);
        child_links.insert(child_link.clone());

        joints.push(JointData {
            name: joint_name,
            joint_type,
            parent_link,
            child_link,
            origin,
            axis: urdf_axis,
            lower,
            upper,
            effort: 0.0,
            velocity: 0.0,
        });
    }

    // Determine root link: link not appearing as any joint's child
    let root_link = links
        .iter()
        .find(|l| !child_links.contains(&l.name))
        .map(|l| l.name.clone())
        .unwrap_or_else(|| links[0].name.clone());

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
        source_path: source_path.map(|p| p.to_path_buf()),
        base_transform: na::Isometry3::identity(),
        misarta_cache: None,
        loop_closures: Vec::new(),
    };
    model.rebuild_misarta_model();
    Ok(model)
}

// =========================================================================
//  Tests
// =========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_f3_basic() {
        let (x, y, z) = parse_f3("(1.5, -2.0, 3.0)");
        assert!((x - 1.5).abs() < 1e-5);
        assert!((y + 2.0).abs() < 1e-5);
        assert!((z - 3.0).abs() < 1e-5);
    }

    #[test]
    fn parse_quat_identity() {
        let q = parse_quat("(1, 0, 0, 0)");
        assert!((q.w - 1.0).abs() < 1e-5);
    }

    #[test]
    fn extract_rel_name_from_path() {
        assert_eq!(
            extract_rel_name("</World/my_robot/base_link>"),
            "base_link"
        );
    }

    #[test]
    fn roundtrip_empty_model() {
        let model = RobotModel::new_empty("test_robot");
        let usda = crate::usd::export_usda(&model);
        let imported = import_usda_from_str(&usda, None).unwrap();

        assert_eq!(imported.name, "test_robot");
        assert_eq!(imported.links.len(), 1);
        assert_eq!(imported.links[0].name, "base_link");
        assert_eq!(imported.joints.len(), 0);
        assert_eq!(imported.root_link, "base_link");

        // Visual
        assert_eq!(imported.links[0].visuals.len(), 1);
        match &imported.links[0].visuals[0].geometry {
            GeomData::Box { hx, hy, hz } => {
                assert!((hx - 0.05).abs() < 1e-3);
                assert!((hy - 0.05).abs() < 1e-3);
                assert!((hz - 0.025).abs() < 1e-3);
            }
            _ => panic!("Expected Box geometry"),
        }
    }

    #[test]
    fn roundtrip_model_with_joints() {
        let mut model = RobotModel::new_empty("jointbot");
        // Add a child link + revolute joint
        model.add_child(
            "base_link",
            "arm_link",
            "arm_joint",
            "revolute",
            na::Isometry3::from_parts(
                na::Translation3::new(0.0, 0.0, 0.1),
                na::UnitQuaternion::identity(),
            ),
            na::Vector3::z(),
            GeomData::Box { hx: 0.05, hy: 0.05, hz: 0.05 },
            [0.7, 0.7, 0.7, 1.0],
            -1.57,
            1.57,
        ).unwrap();

        let usda = crate::usd::export_usda(&model);
        let imported = import_usda_from_str(&usda, None).unwrap();

        assert_eq!(imported.links.len(), 2);
        assert_eq!(imported.joints.len(), 1);
        assert_eq!(imported.joints[0].name, "arm_joint");
        assert_eq!(imported.joints[0].joint_type, "revolute");
        assert_eq!(imported.joints[0].parent_link, "base_link");
        assert_eq!(imported.joints[0].child_link, "arm_link");
        assert_eq!(imported.root_link, "base_link");

        // Check axis reconstruction (should be ~Z)
        let axis = imported.joints[0].axis.normalize();
        assert!(
            (axis.z.abs() - 1.0).abs() < 0.1,
            "Expected Z axis, got {:?}",
            axis
        );

        // Check limits (exported as degrees, re-imported as radians)
        assert!(
            (imported.joints[0].lower - (-1.57)).abs() < 0.02,
            "lower = {}",
            imported.joints[0].lower
        );
        assert!(
            (imported.joints[0].upper - 1.57).abs() < 0.02,
            "upper = {}",
            imported.joints[0].upper
        );

        // Check origin translation
        let t = imported.joints[0].origin.translation;
        assert!((t.z - 0.1).abs() < 1e-3, "origin z = {}", t.z);
    }

    #[test]
    fn roundtrip_fixed_joint() {
        let mut model = RobotModel::new_empty("fixbot");
        model.add_child(
            "base_link", "sensor_link", "sensor_joint", "fixed",
            na::Isometry3::from_parts(
                na::Translation3::new(0.05, 0.0, 0.02),
                na::UnitQuaternion::from_euler_angles(0.0, 0.0, std::f32::consts::FRAC_PI_4),
            ),
            na::Vector3::z(),
            GeomData::Box { hx: 0.02, hy: 0.02, hz: 0.02 },
            [0.5, 0.5, 0.5, 1.0],
            0.0, 0.0,
        ).unwrap();

        let usda = crate::usd::export_usda(&model);
        let imported = import_usda_from_str(&usda, None).unwrap();

        assert_eq!(imported.joints[0].joint_type, "fixed");
        let t = imported.joints[0].origin.translation;
        assert!((t.x - 0.05).abs() < 1e-3);
        assert!((t.z - 0.02).abs() < 1e-3);
    }

    #[test]
    fn roundtrip_prismatic_joint() {
        let mut model = RobotModel::new_empty("slidebot");
        model.add_child(
            "base_link", "slider", "slide_joint", "prismatic",
            na::Isometry3::identity(),
            na::Vector3::x(),
            GeomData::Box { hx: 0.03, hy: 0.03, hz: 0.03 },
            [0.7, 0.7, 0.7, 1.0],
            -0.5, 0.5,
        ).unwrap();

        let usda = crate::usd::export_usda(&model);
        let imported = import_usda_from_str(&usda, None).unwrap();

        assert_eq!(imported.joints[0].joint_type, "prismatic");
        assert!((imported.joints[0].lower - (-0.5)).abs() < 0.02);
        assert!((imported.joints[0].upper - 0.5).abs() < 0.02);

        // Axis should be ~X
        let axis = imported.joints[0].axis.normalize();
        assert!(
            (axis.x.abs() - 1.0).abs() < 0.1,
            "Expected X axis, got {:?}",
            axis
        );
    }

    #[test]
    fn roundtrip_material_colors() {
        let mut model = RobotModel::new_empty("colorbot");
        model.links[0].visuals[0].color = [1.0, 0.0, 0.0, 0.8];

        let usda = crate::usd::export_usda(&model);
        let imported = import_usda_from_str(&usda, None).unwrap();

        let color = imported.links[0].visuals[0].color;
        assert!((color[0] - 1.0).abs() < 0.01, "r = {}", color[0]);
        assert!((color[1]).abs() < 0.01, "g = {}", color[1]);
        assert!((color[2]).abs() < 0.01, "b = {}", color[2]);
        assert!((color[3] - 0.8).abs() < 0.01, "a = {}", color[3]);
    }

    #[test]
    fn roundtrip_cylinder_geometry() {
        let mut model = RobotModel::new_empty("cylbot");
        model.links[0].visuals[0].geometry = GeomData::Cylinder {
            radius: 0.03,
            half_length: 0.15,
        };

        let usda = crate::usd::export_usda(&model);
        let imported = import_usda_from_str(&usda, None).unwrap();

        match &imported.links[0].visuals[0].geometry {
            GeomData::Cylinder { radius, half_length } => {
                assert!(
                    (radius - 0.03).abs() < 1e-3,
                    "radius = {}",
                    radius
                );
                assert!(
                    (half_length - 0.15).abs() < 1e-3,
                    "half_length = {}",
                    half_length
                );
            }
            _ => panic!("Expected Cylinder geometry"),
        }
    }

    #[test]
    fn roundtrip_sphere_geometry() {
        let mut model = RobotModel::new_empty("sphbot");
        model.links[0].visuals[0].geometry = GeomData::Sphere { radius: 0.08 };

        let usda = crate::usd::export_usda(&model);
        let imported = import_usda_from_str(&usda, None).unwrap();

        match &imported.links[0].visuals[0].geometry {
            GeomData::Sphere { radius } => {
                assert!((radius - 0.08).abs() < 1e-3, "radius = {}", radius);
            }
            _ => panic!("Expected Sphere geometry"),
        }
    }

    #[test]
    fn roundtrip_inertial() {
        let mut model = RobotModel::new_empty("massbot");
        model.links[0].inertial.mass = 2.5;
        model.links[0].inertial.ixx = 0.01;
        model.links[0].inertial.iyy = 0.02;
        model.links[0].inertial.izz = 0.03;
        model.links[0].inertial.origin = na::Isometry3::from_parts(
            na::Translation3::new(0.01, 0.02, 0.03),
            na::UnitQuaternion::identity(),
        );

        let usda = crate::usd::export_usda(&model);
        let imported = import_usda_from_str(&usda, None).unwrap();

        let inertial = &imported.links[0].inertial;
        assert!((inertial.mass - 2.5).abs() < 0.01);
        assert!((inertial.ixx - 0.01).abs() < 1e-3);
        assert!((inertial.iyy - 0.02).abs() < 1e-3);
        assert!((inertial.izz - 0.03).abs() < 1e-3);
        assert!(
            (inertial.origin.translation.x - 0.01).abs() < 1e-3,
        );
    }

    #[test]
    fn roundtrip_multi_link_tree() {
        let mut model = RobotModel::new_empty("treebot");
        model.add_child(
            "base_link", "link_a", "joint_a", "revolute",
            na::Isometry3::identity(), na::Vector3::z(),
            GeomData::Box { hx: 0.05, hy: 0.05, hz: 0.05 },
            [0.7, 0.7, 0.7, 1.0], -1.57, 1.57,
        ).unwrap();
        model.add_child(
            "link_a", "link_b", "joint_b", "revolute",
            na::Isometry3::identity(), na::Vector3::z(),
            GeomData::Box { hx: 0.05, hy: 0.05, hz: 0.05 },
            [0.7, 0.7, 0.7, 1.0], -1.57, 1.57,
        ).unwrap();
        model.add_child(
            "base_link", "link_c", "joint_c", "fixed",
            na::Isometry3::identity(), na::Vector3::z(),
            GeomData::Box { hx: 0.05, hy: 0.05, hz: 0.05 },
            [0.7, 0.7, 0.7, 1.0], 0.0, 0.0,
        ).unwrap();

        let usda = crate::usd::export_usda(&model);
        let imported = import_usda_from_str(&usda, None).unwrap();

        assert_eq!(imported.links.len(), 4);
        assert_eq!(imported.joints.len(), 3);
        assert_eq!(imported.root_link, "base_link");

        // Check tree topology
        let names: Vec<_> = imported.links.iter().map(|l| l.name.as_str()).collect();
        assert!(names.contains(&"base_link"));
        assert!(names.contains(&"link_a"));
        assert!(names.contains(&"link_b"));
        assert!(names.contains(&"link_c"));
    }
}
