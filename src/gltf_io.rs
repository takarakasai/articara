//! glTF 2.0 (`.glb` / `.gltf`) import & export for robot models.
//!
//! ## Export
//!
//! glTF is a scene format with no notion of joints, so the kinematic tree
//! is laid out as a node hierarchy that a viewer can animate by rotating /
//! translating the child-link nodes:
//!
//! ```text
//! <robot>                 Z-up → Y-up conversion, extras.articara = .misa
//! └─ <root link>          base_transform
//!    ├─ <link>.visual_0   visual origin + mesh
//!    └─ <joint>           joint origin
//!       └─ <child link>   joint motion at the current pose
//! ```
//!
//! Primitives (box / cylinder / sphere / capsule) are tessellated. The full
//! `.misa` description is embedded as JSON in the robot node's
//! `extras.articara`, so articara → glTF → articara round-trips everything
//! `.misa` carries (joints, limits, inertia, sensors, poses, …). Mesh
//! triangles travel in the binary buffer on nodes tagged with
//! `extras.articara_geom`. Collision meshes live in a second, non-default
//! scene so viewers don't draw them.
//!
//! ## Import
//!
//! - **With `extras.articara`** (files written by articara): the model is
//!   rebuilt from the embedded `.misa` and the mesh triangles reattached.
//! - **Plain glTF** (Blender, CAD exporters, …): every node becomes a link
//!   attached to its parent by a fixed joint. Non-rigid parts of node
//!   transforms (scale / shear) are baked into the mesh vertices, and the
//!   Y-up scene is rotated into articara's Z-up frame. Each mesh primitive
//!   becomes one visual coloured by its material's base colour.

use std::collections::{HashMap, HashSet};
use std::path::Path;
use std::sync::Arc;

use base64::Engine as _;
use nalgebra as na;
use serde_json::{Value, json};

use crate::robot::*;
use misarta::mesh::MeshData;
use misarta::native as mn;

/// Key of the robot-node extras holding the embedded `.misa` description.
const META_KEY: &str = "articara";
/// Key of the per-geometry-node extras mapping a glTF mesh back to a
/// `(link, visual|collision, index)` slot.
const GEOM_KEY: &str = "articara_geom";
/// Version of the `extras.articara` layout.
const META_VERSION: u64 = 1;
/// Name of the non-default scene that carries collision meshes.
const COLLISION_SCENE: &str = "articara_collision";

const ARRAY_BUFFER: u32 = 34962;
const ELEMENT_ARRAY_BUFFER: u32 = 34963;
const FLOAT: u32 = 5126;
const UNSIGNED_INT: u32 = 5125;

/// Rotation taking articara's Z-up frame to glTF's Y-up frame (−90° about X).
fn z_up_to_y_up() -> na::UnitQuaternion<f64> {
    na::UnitQuaternion::from_axis_angle(&na::Vector3::x_axis(), -std::f64::consts::FRAC_PI_2)
}

// =========================================================================
//  Export
// =========================================================================

/// Export the model as a binary glTF (`.glb`) byte buffer.
pub fn export_glb(model: &RobotModel) -> Result<Vec<u8>, String> {
    let (doc, bin) = build_document(model)?;
    Ok(write_glb(&doc, &bin))
}

/// Export the model as a self-contained `.gltf` JSON string (the binary
/// buffer is embedded as a base64 data URI).
pub fn export_gltf_embedded(model: &RobotModel) -> Result<String, String> {
    let (mut doc, bin) = build_document(model)?;
    if !bin.is_empty() {
        let b64 = base64::engine::general_purpose::STANDARD.encode(&bin);
        doc["buffers"][0]["uri"] = json!(format!("data:application/octet-stream;base64,{b64}"));
    }
    serde_json::to_string_pretty(&doc).map_err(|e| format!("glTF JSON: {e}"))
}

/// Write the model to `path`. A `.gltf` extension produces embedded JSON;
/// anything else (normally `.glb`) produces binary glTF.
pub fn export_gltf_to_file(model: &RobotModel, path: &Path) -> Result<(), String> {
    let is_json = path
        .extension()
        .and_then(|e| e.to_str())
        .is_some_and(|e| e.eq_ignore_ascii_case("gltf"));
    let bytes = if is_json {
        export_gltf_embedded(model)?.into_bytes()
    } else {
        export_glb(model)?
    };
    std::fs::write(path, bytes).map_err(|e| format!("Write glTF: {e}"))?;
    log::info!("glTF export: {:?}", path);
    Ok(())
}

/// Accumulates glTF arrays plus the single binary buffer they index into.
#[derive(Default)]
struct GltfBuilder {
    bin: Vec<u8>,
    buffer_views: Vec<Value>,
    accessors: Vec<Value>,
    meshes: Vec<Value>,
    materials: Vec<Value>,
    nodes: Vec<Value>,
    /// RGBA bit pattern → material index, so equal colours share a material.
    material_cache: HashMap<[u32; 4], usize>,
}

impl GltfBuilder {
    fn push_view(&mut self, bytes: &[u8], target: u32) -> usize {
        while self.bin.len() % 4 != 0 {
            self.bin.push(0);
        }
        let offset = self.bin.len();
        self.bin.extend_from_slice(bytes);
        self.buffer_views.push(json!({
            "buffer": 0,
            "byteOffset": offset,
            "byteLength": bytes.len(),
            "target": target,
        }));
        self.buffer_views.len() - 1
    }

    fn push_vec3(&mut self, data: &[[f32; 3]], with_bounds: bool) -> usize {
        let bytes: Vec<u8> = data
            .iter()
            .flat_map(|v| v.iter().flat_map(|c| c.to_le_bytes()))
            .collect();
        let view = self.push_view(&bytes, ARRAY_BUFFER);
        let mut acc = json!({
            "bufferView": view,
            "componentType": FLOAT,
            "count": data.len(),
            "type": "VEC3",
        });
        // POSITION accessors must carry min / max (glTF 2.0 §3.7.2.1).
        if with_bounds {
            let mut min = [f32::INFINITY; 3];
            let mut max = [f32::NEG_INFINITY; 3];
            for v in data {
                for k in 0..3 {
                    min[k] = min[k].min(v[k]);
                    max[k] = max[k].max(v[k]);
                }
            }
            acc["min"] = json!(min);
            acc["max"] = json!(max);
        }
        self.accessors.push(acc);
        self.accessors.len() - 1
    }

    fn push_indices(&mut self, indices: &[u32]) -> usize {
        let bytes: Vec<u8> = indices.iter().flat_map(|i| i.to_le_bytes()).collect();
        let view = self.push_view(&bytes, ELEMENT_ARRAY_BUFFER);
        self.accessors.push(json!({
            "bufferView": view,
            "componentType": UNSIGNED_INT,
            "count": indices.len(),
            "type": "SCALAR",
        }));
        self.accessors.len() - 1
    }

    fn material(&mut self, rgba: [f32; 4]) -> usize {
        let key = rgba.map(f32::to_bits);
        if let Some(&i) = self.material_cache.get(&key) {
            return i;
        }
        let mut mat = json!({
            "name": format!("material_{}", self.materials.len()),
            "pbrMetallicRoughness": {
                "baseColorFactor": rgba,
                "metallicFactor": 0.0,
                "roughnessFactor": 0.8,
            },
        });
        if rgba[3] < 1.0 {
            mat["alphaMode"] = json!("BLEND");
        }
        self.materials.push(mat);
        let i = self.materials.len() - 1;
        self.material_cache.insert(key, i);
        i
    }

    /// Add a single-primitive mesh. Returns `None` for empty geometry.
    fn add_mesh(&mut self, name: &str, tris: &Triangles, material: usize) -> Option<usize> {
        if tris.positions.is_empty() {
            return None;
        }
        let pos = self.push_vec3(&tris.positions, true);
        let mut attributes = json!({ "POSITION": pos });
        if let Some(normals) = &tris.normals {
            attributes["NORMAL"] = json!(self.push_vec3(normals, false));
        }
        let mut prim = json!({ "attributes": attributes, "material": material });
        if let Some(indices) = &tris.indices {
            prim["indices"] = json!(self.push_indices(indices));
        }
        self.meshes.push(json!({ "name": name, "primitives": [prim] }));
        Some(self.meshes.len() - 1)
    }

    fn add_node(&mut self, node: Value) -> usize {
        self.nodes.push(node);
        self.nodes.len() - 1
    }
}

/// Triangle soup ready for a glTF primitive. `indices: None` means
/// consecutive vertex triples; `normals: None` lets the viewer compute
/// flat normals (allowed by the glTF spec).
struct Triangles {
    positions: Vec<[f32; 3]>,
    normals: Option<Vec<[f32; 3]>>,
    indices: Option<Vec<u32>>,
}

impl Triangles {
    /// From an interleaved `[x, y, z, nx, ny, nz, …]` buffer as produced
    /// by [`crate::primitives`].
    fn from_flat(flat: &[f32]) -> Self {
        let mut positions = Vec::with_capacity(flat.len() / 6);
        let mut normals = Vec::with_capacity(flat.len() / 6);
        for v in flat.chunks_exact(6) {
            positions.push([v[0], v[1], v[2]]);
            normals.push([v[3], v[4], v[5]]);
        }
        Self { positions, normals: Some(normals), indices: None }
    }

    fn from_mesh(mesh: &MeshData) -> Self {
        let p = |v: &na::Point3<f64>| [v.x as f32, v.y as f32, v.z as f32];
        let positions = mesh.vertices.iter().map(p).collect();
        let normals = (mesh.vertex_normals.len() == mesh.vertices.len()
            && !mesh.vertex_normals.is_empty())
        .then(|| {
            mesh.vertex_normals
                .iter()
                .map(|n| [n.x as f32, n.y as f32, n.z as f32])
                .collect()
        });
        let indices = mesh.indices.iter().flatten().copied().collect();
        Self { positions, normals, indices: Some(indices) }
    }
}

fn geom_triangles(geom: &GeomData) -> Triangles {
    use crate::primitives::*;
    match geom {
        GeomData::Box { hx, hy, hz } => Triangles::from_flat(&generate_box(*hx, *hy, *hz)),
        GeomData::Cylinder { radius, half_length } => {
            Triangles::from_flat(&generate_cylinder(*radius, *half_length, 32))
        }
        GeomData::Sphere { radius } => Triangles::from_flat(&generate_sphere(*radius, 32, 16)),
        GeomData::Capsule { radius, half_length } => {
            Triangles::from_flat(&generate_capsule(*radius, *half_length, 32, 8))
        }
        GeomData::Mesh { mesh, .. } => Triangles::from_mesh(mesh),
    }
}

/// Write translation / rotation of `iso` into a node, skipping identity parts.
fn set_trs(node: &mut Value, iso: &na::Isometry3<f64>) {
    let t = iso.translation.vector;
    if t.norm_squared() > 0.0 {
        node["translation"] = json!([t.x, t.y, t.z]);
    }
    let q = iso.rotation.quaternion();
    if iso.rotation.angle() > 0.0 {
        node["rotation"] = json!([q.i, q.j, q.k, q.w]);
    }
}

fn geom_tag(link: &str, kind: &str, index: usize) -> Value {
    json!({ GEOM_KEY: { "link": link, "kind": kind, "index": index } })
}

fn build_document(model: &RobotModel) -> Result<(Value, Vec<u8>), String> {
    let misa = model.to_misa()?;
    let misa_json = serde_json::to_value(&misa).map_err(|e| format!("glTF: encode .misa: {e}"))?;

    // Posed world transforms from the editor's FK; per-link motion is
    // recovered from these so mimic / custom joint types stay consistent
    // with what the viewport shows.
    let world: HashMap<String, na::Isometry3<f64>> = model
        .compute_transforms()
        .into_iter()
        .map(|(name, iso)| (name, iso.cast::<f64>()))
        .collect();

    let mut b = GltfBuilder::default();
    let mut collision_nodes = Vec::new();
    let root_pose = world.get(&model.root_link).copied().unwrap_or_else(na::Isometry3::identity);
    let root_node = export_link(model, &model.root_link, &root_pose, &world, &mut b, &mut collision_nodes)?;

    let mut robot = json!({
        "name": model.name,
        "children": [root_node],
        "extras": { META_KEY: { "version": META_VERSION, "misa": misa_json } },
    });
    set_trs(&mut robot, &na::Isometry3::from_parts(na::Translation3::identity(), z_up_to_y_up()));
    let robot_node = b.add_node(robot);

    let mut scenes = vec![json!({ "name": model.name, "nodes": [robot_node] })];
    if !collision_nodes.is_empty() {
        scenes.push(json!({ "name": COLLISION_SCENE, "nodes": collision_nodes }));
    }

    let mut doc = json!({
        "asset": { "version": "2.0", "generator": "articara" },
        "scene": 0,
        "scenes": scenes,
        "nodes": b.nodes,
    });
    if !b.meshes.is_empty() {
        doc["meshes"] = json!(b.meshes);
        doc["materials"] = json!(b.materials);
        doc["accessors"] = json!(b.accessors);
        doc["bufferViews"] = json!(b.buffer_views);
        doc["buffers"] = json!([{ "byteLength": b.bin.len() }]);
    }
    Ok((doc, b.bin))
}

/// Emit the node subtree for `link_name` (placed at `pose` relative to its
/// parent node) and return its node index.
fn export_link(
    model: &RobotModel,
    link_name: &str,
    pose: &na::Isometry3<f64>,
    world: &HashMap<String, na::Isometry3<f64>>,
    b: &mut GltfBuilder,
    collision_nodes: &mut Vec<usize>,
) -> Result<usize, String> {
    let li = *model
        .link_map
        .get(link_name)
        .ok_or_else(|| format!("glTF export: unknown link '{link_name}'"))?;
    let link = &model.links[li];
    let mut children = Vec::new();

    for (i, vis) in link.visuals.iter().enumerate() {
        let name = format!("{}.visual_{i}", link.name);
        let material = b.material(vis.color);
        let Some(mesh) = b.add_mesh(&name, &geom_triangles(&vis.geometry), material) else {
            continue;
        };
        let mut node = json!({ "name": name, "mesh": mesh, "extras": geom_tag(&link.name, "visual", i) });
        set_trs(&mut node, &vis.origin.cast::<f64>());
        children.push(b.add_node(node));
    }

    // Collision primitives are fully described by the embedded `.misa`;
    // only meshes need their triangles carried in the buffer.
    for (i, col) in link.collisions.iter().enumerate() {
        if !matches!(col.geometry, GeomData::Mesh { .. }) {
            continue;
        }
        let name = format!("{}.collision_{i}", link.name);
        let material = b.material([0.9, 0.5, 0.1, 0.5]);
        if let Some(mesh) = b.add_mesh(&name, &geom_triangles(&col.geometry), material) {
            let node = json!({ "name": name, "mesh": mesh, "extras": geom_tag(&link.name, "collision", i) });
            collision_nodes.push(b.add_node(node));
        }
    }

    let parent_world = world.get(link_name).copied().unwrap_or_else(na::Isometry3::identity);
    for &ji in model.children_joints.get(link_name).map(Vec::as_slice).unwrap_or(&[]) {
        let joint = &model.joints[ji];
        let origin = joint.origin.cast::<f64>();
        // Child link pose relative to the joint frame = joint motion at q.
        let motion = world
            .get(&joint.child_link)
            .map(|w| origin.inverse() * parent_world.inverse() * w)
            .unwrap_or_else(na::Isometry3::identity);
        let child_idx = export_link(model, &joint.child_link, &motion, world, b, collision_nodes)?;
        let mut jnode = json!({ "name": joint.name, "children": [child_idx] });
        set_trs(&mut jnode, &origin);
        children.push(b.add_node(jnode));
    }

    let mut node = json!({ "name": link.name });
    if !children.is_empty() {
        node["children"] = json!(children);
    }
    set_trs(&mut node, pose);
    Ok(b.add_node(node))
}

/// Pack a glTF JSON document and binary buffer into a GLB container.
fn write_glb(doc: &Value, bin: &[u8]) -> Vec<u8> {
    let mut json = serde_json::to_vec(doc).expect("serialising a serde_json::Value cannot fail");
    while json.len() % 4 != 0 {
        json.push(b' ');
    }
    let mut bin = bin.to_vec();
    while bin.len() % 4 != 0 {
        bin.push(0);
    }
    let total = 12 + 8 + json.len() + if bin.is_empty() { 0 } else { 8 + bin.len() };

    let mut out = Vec::with_capacity(total);
    out.extend_from_slice(b"glTF");
    out.extend_from_slice(&2u32.to_le_bytes());
    out.extend_from_slice(&(total as u32).to_le_bytes());
    out.extend_from_slice(&(json.len() as u32).to_le_bytes());
    out.extend_from_slice(b"JSON");
    out.extend_from_slice(&json);
    if !bin.is_empty() {
        out.extend_from_slice(&(bin.len() as u32).to_le_bytes());
        out.extend_from_slice(b"BIN\0");
        out.extend_from_slice(&bin);
    }
    out
}

// =========================================================================
//  Import
// =========================================================================

/// Import a `.glb` / `.gltf` file.
pub fn import_gltf(path: &Path) -> Result<RobotModel, String> {
    let bytes = std::fs::read(path).map_err(|e| format!("Read glTF: {e}"))?;
    import_gltf_from_slice(&bytes, Some(path))
}

/// Import from in-memory `.glb` / `.gltf` bytes. `source_path` resolves
/// external buffer URIs and is stored on the model.
pub fn import_gltf_from_slice(bytes: &[u8], source_path: Option<&Path>) -> Result<RobotModel, String> {
    let gltf = gltf::Gltf::from_slice(bytes).map_err(|e| format!("Parse glTF: {e}"))?;
    let buffers = load_buffers(&gltf, source_path.and_then(Path::parent))?;
    let doc = &gltf.document;

    if doc.skins().next().is_some() || doc.animations().next().is_some() {
        log::warn!("glTF import: skins / animations are ignored");
    }

    let meta = doc.nodes().find_map(|n| {
        let mut extras = parse_extras(n.extras())?;
        extras.get_mut(META_KEY).map(Value::take)
    });
    let mut model = match meta {
        Some(meta) => import_articara(doc, &buffers, meta, source_path)?,
        None => import_generic(doc, &buffers, source_path)?,
    };
    model.source_path = source_path.map(Path::to_path_buf);
    Ok(model)
}

fn parse_extras(extras: &gltf::json::Extras) -> Option<Value> {
    serde_json::from_str(extras.as_ref()?.get()).ok()
}

fn load_buffers(gltf: &gltf::Gltf, base_dir: Option<&Path>) -> Result<Vec<Vec<u8>>, String> {
    gltf.document
        .buffers()
        .map(|buf| {
            let data = match buf.source() {
                gltf::buffer::Source::Bin => gltf
                    .blob
                    .clone()
                    .ok_or("glTF: BIN chunk referenced but missing")?,
                gltf::buffer::Source::Uri(uri) => {
                    if let Some(rest) = uri.strip_prefix("data:") {
                        let (_, payload) = rest
                            .split_once(";base64,")
                            .ok_or_else(|| format!("glTF: unsupported data URI '{}…'", &uri[..uri.len().min(40)]))?;
                        base64::engine::general_purpose::STANDARD
                            .decode(payload)
                            .map_err(|e| format!("glTF: bad base64 buffer: {e}"))?
                    } else {
                        let path = base_dir.unwrap_or(Path::new(".")).join(uri);
                        std::fs::read(&path).map_err(|e| format!("glTF: read buffer {path:?}: {e}"))?
                    }
                }
            };
            if data.len() < buf.length() {
                return Err(format!(
                    "glTF: buffer {} is {} bytes, expected {}",
                    buf.index(),
                    data.len(),
                    buf.length()
                ));
            }
            Ok(data)
        })
        .collect()
}

/// Read one primitive as triangles, applying `linear` to the vertices.
fn primitive_mesh(
    prim: &gltf::Primitive<'_>,
    buffers: &[Vec<u8>],
    linear: &na::Matrix3<f64>,
) -> Option<MeshData> {
    use gltf::mesh::Mode;
    let reader = prim.reader(|b| buffers.get(b.index()).map(Vec::as_slice));
    let positions: Vec<[f32; 3]> = reader.read_positions()?.collect();
    let normals: Option<Vec<[f32; 3]>> = reader.read_normals().map(Iterator::collect);
    let idx: Vec<u32> = match reader.read_indices() {
        Some(i) => i.into_u32().collect(),
        None => (0..positions.len() as u32).collect(),
    };

    let mut tris: Vec<[u32; 3]> = match prim.mode() {
        Mode::Triangles => idx.chunks_exact(3).map(|t| [t[0], t[1], t[2]]).collect(),
        Mode::TriangleStrip => (0..idx.len().saturating_sub(2))
            .map(|i| {
                if i % 2 == 0 {
                    [idx[i], idx[i + 1], idx[i + 2]]
                } else {
                    [idx[i + 1], idx[i], idx[i + 2]]
                }
            })
            .collect(),
        Mode::TriangleFan => (1..idx.len().saturating_sub(1))
            .map(|i| [idx[0], idx[i], idx[i + 1]])
            .collect(),
        other => {
            log::warn!("glTF import: skipping non-triangle primitive ({other:?})");
            return None;
        }
    };
    tris.retain(|t| t.iter().all(|&i| (i as usize) < positions.len()));
    // A mirroring transform flips the winding; restore counter-clockwise.
    if linear.determinant() < 0.0 {
        for t in &mut tris {
            t.swap(1, 2);
        }
    }

    let vertices: Vec<na::Point3<f64>> = positions
        .iter()
        .map(|p| na::Point3::from(linear * na::Vector3::new(p[0] as f64, p[1] as f64, p[2] as f64)))
        .collect();
    let normal_xf = linear.try_inverse().map(|m| m.transpose()).unwrap_or_else(na::Matrix3::identity);
    let vertex_normals = match normals {
        Some(ns) if ns.len() == vertices.len() => ns
            .iter()
            .map(|n| {
                (normal_xf * na::Vector3::new(n[0] as f64, n[1] as f64, n[2] as f64))
                    .try_normalize(1e-30)
                    .unwrap_or_else(na::Vector3::zeros)
            })
            .collect(),
        _ => Vec::new(),
    };
    let face_normals = tris
        .iter()
        .map(|t| {
            let [a, b, c] = t.map(|i| vertices[i as usize]);
            (b - a).cross(&(c - a)).try_normalize(1e-30).unwrap_or_else(na::Vector3::zeros)
        })
        .collect();

    Some(MeshData {
        vertices,
        indices: tris,
        face_normals,
        vertex_normals,
        texcoords: Vec::new(),
        materials: Vec::new(),
        submeshes: Vec::new(),
    })
}

/// Concatenate meshes into one (indices re-based).
fn merge_meshes(parts: Vec<MeshData>) -> MeshData {
    let mut parts = parts.into_iter();
    let mut out = parts.next().unwrap_or_else(|| MeshData::from_flat_vertices_f32(&[]));
    for m in parts {
        let base = out.vertices.len() as u32;
        // Keep per-vertex normals only if every part has them.
        if out.vertex_normals.len() == out.vertices.len() && m.vertex_normals.len() == m.vertices.len() {
            out.vertex_normals.extend(m.vertex_normals);
        } else {
            out.vertex_normals.clear();
        }
        out.vertices.extend(m.vertices);
        out.indices.extend(m.indices.iter().map(|t| t.map(|i| i + base)));
        out.face_normals.extend(m.face_normals);
    }
    if out.vertex_normals.len() != out.vertices.len() {
        out.vertex_normals.clear();
    }
    out
}

fn mesh_geom(mesh: MeshData) -> GeomData {
    GeomData::Mesh { mesh: Arc::new(mesh), filename: None, scale: None }
}

/// Stand-in geometry for slots whose triangles are attached after the
/// `.misa` build (avoids load warnings for a mesh file that doesn't exist).
fn placeholder_geom() -> mn::Geom {
    mn::Geom::Sphere { radius: 1e-6 }
}

#[derive(Clone, Copy, PartialEq, Eq, Hash)]
enum Slot {
    Visual(usize),
    Collision(usize),
}

fn set_slot_geometry(model: &mut RobotModel, link: &str, slot: Slot, geometry: GeomData) {
    let Some(&li) = model.link_map.get(link) else {
        return;
    };
    let l = &mut model.links[li];
    let target = match slot {
        Slot::Visual(i) => l.visuals.get_mut(i).map(|v| &mut v.geometry),
        Slot::Collision(i) => l.collisions.get_mut(i).map(|c| &mut c.geometry),
    };
    if let Some(g) = target {
        *g = geometry;
    }
}

/// Rebuild a model exported by articara from its embedded `.misa`.
fn import_articara(
    doc: &gltf::Document,
    buffers: &[Vec<u8>],
    meta: Value,
    source_path: Option<&Path>,
) -> Result<RobotModel, String> {
    if let Some(v) = meta.get("version").and_then(Value::as_u64) {
        if v > META_VERSION {
            log::warn!("glTF import: articara metadata version {v} is newer than supported ({META_VERSION})");
        }
    }
    let misa_json = meta.get("misa").cloned().ok_or("glTF: extras.articara has no 'misa' entry")?;
    let mut file: mn::MisaFile =
        serde_json::from_value(misa_json).map_err(|e| format!("glTF: decode embedded .misa: {e}"))?;

    // Collect triangles from tagged geometry nodes (all scenes, so the
    // collision scene is included).
    let mut meshes: HashMap<(String, Slot), MeshData> = HashMap::new();
    for node in doc.nodes() {
        let (Some(mesh), Some(extras)) = (node.mesh(), parse_extras(node.extras())) else {
            continue;
        };
        let Some(tag) = extras.get(GEOM_KEY) else {
            continue;
        };
        let (Some(link), Some(kind), Some(index)) = (
            tag.get("link").and_then(Value::as_str),
            tag.get("kind").and_then(Value::as_str),
            tag.get("index").and_then(Value::as_u64),
        ) else {
            continue;
        };
        let slot = match kind {
            "visual" => Slot::Visual(index as usize),
            "collision" => Slot::Collision(index as usize),
            _ => continue,
        };
        let parts = mesh
            .primitives()
            .filter_map(|p| primitive_mesh(&p, buffers, &na::Matrix3::identity()))
            .collect();
        meshes.insert((link.to_string(), slot), merge_meshes(parts));
    }

    // Mesh slots backed by glTF triangles get a placeholder; the rest keep
    // their file reference and load relative to the glTF file as usual.
    let mut attach = Vec::new();
    for link in &mut file.link {
        let geoms = link
            .visual
            .iter_mut()
            .enumerate()
            .map(|(i, v)| (Slot::Visual(i), &mut v.geom))
            .chain(link.collision.iter_mut().enumerate().map(|(i, c)| (Slot::Collision(i), &mut c.geom)));
        for (slot, geom) in geoms {
            if !matches!(geom, mn::Geom::Mesh { .. }) {
                continue;
            }
            if let Some(mesh) = meshes.remove(&(link.name.clone(), slot)) {
                *geom = placeholder_geom();
                attach.push((link.name.clone(), slot, mesh));
            }
        }
    }

    let base = source_path.unwrap_or(Path::new(""));
    let mut model = RobotModel::from_misa_file(&file, base)?;
    for (link, slot, mesh) in attach {
        set_slot_geometry(&mut model, &link, slot, mesh_geom(mesh));
    }
    model.rebuild_misarta_model();
    Ok(model)
}

/// Closest rigid transform to an affine matrix: `(rigid, residual)` with
/// `world = rigid · residual`, where `residual` is the remaining 3×3
/// linear part (scale / shear / mirroring) to be baked into vertices.
fn split_rigid(world: &na::Matrix4<f64>) -> (na::Isometry3<f64>, na::Matrix3<f64>) {
    let m: na::Matrix3<f64> = world.fixed_view::<3, 3>(0, 0).into_owned();
    let t = na::Vector3::new(world[(0, 3)], world[(1, 3)], world[(2, 3)]);
    let svd = m.svd(true, true);
    let (Some(mut u), Some(v_t)) = (svd.u, svd.v_t) else {
        return (na::Isometry3::from_parts(t.into(), na::UnitQuaternion::identity()), m);
    };
    if (u * v_t).determinant() < 0.0 {
        u.column_mut(2).neg_mut();
    }
    let rot = na::Rotation3::from_matrix_unchecked(u * v_t);
    let residual = rot.matrix().transpose() * m;
    (
        na::Isometry3::from_parts(t.into(), na::UnitQuaternion::from_rotation_matrix(&rot)),
        residual,
    )
}

fn misa_origin(iso: &na::Isometry3<f64>) -> mn::Origin {
    let t = iso.translation.vector;
    let q = iso.rotation.quaternion();
    mn::Origin {
        xyz: [t.x, t.y, t.z],
        rpy: None,
        quat: (iso.rotation.angle() > 0.0).then_some([q.i, q.j, q.k, q.w]),
    }
}

fn unique_name(base: &str, used: &mut HashSet<String>) -> String {
    let mut name = base.to_string();
    let mut n = 1;
    while !used.insert(name.clone()) {
        name = format!("{base}_{n}");
        n += 1;
    }
    name
}

struct GenericImport<'a> {
    buffers: &'a [Vec<u8>],
    file: mn::MisaFile,
    link_names: HashSet<String>,
    joint_names: HashSet<String>,
    meshes: Vec<(String, Slot, MeshData)>,
}

impl GenericImport<'_> {
    fn visit(
        &mut self,
        node: gltf::Node<'_>,
        parent_world: &na::Matrix4<f64>,
        parent_rigid: &na::Isometry3<f64>,
        parent_link: &str,
    ) {
        let m = node.transform().matrix();
        let local = na::Matrix4::from_fn(|r, c| m[c][r] as f64);
        let world = parent_world * local;
        let (rigid, residual) = split_rigid(&world);

        let base = node.name().map(str::to_string).unwrap_or_else(|| format!("node_{}", node.index()));
        let link_name = unique_name(&base, &mut self.link_names);
        let mut link = mn::Link {
            name: link_name.clone(),
            description: String::new(),
            inertial: mn::Inertial::default(),
            visual: Vec::new(),
            collision: Vec::new(),
            collision_enabled: true,
        };
        if let Some(mesh) = node.mesh() {
            for prim in mesh.primitives() {
                let Some(data) = primitive_mesh(&prim, self.buffers, &residual) else {
                    continue;
                };
                let rgba = prim.material().pbr_metallic_roughness().base_color_factor();
                self.meshes.push((link_name.clone(), Slot::Visual(link.visual.len()), data));
                link.visual.push(mn::Visual {
                    origin: mn::Origin::default(),
                    geom: placeholder_geom(),
                    color: Some(mn::ColorSpec::Rgba(rgba)),
                    material: None,
                });
            }
        }
        self.file.link.push(link);

        let joint_name = unique_name(&format!("{link_name}_joint"), &mut self.joint_names);
        self.file.joint.push(mn::Joint {
            name: joint_name,
            kind: mn::JointKind::Fixed,
            parent: parent_link.to_string(),
            child: link_name.clone(),
            axis: [0.0, 0.0, 1.0],
            origin: misa_origin(&(parent_rigid.inverse() * rigid)),
            limit: mn::JointLimit::default(),
            dynamics: mn::JointDynamics::default(),
        });

        for child in node.children() {
            self.visit(child, &world, &rigid, &link_name);
        }
    }
}

/// Import an arbitrary glTF scene: nodes → links on fixed joints.
fn import_generic(
    doc: &gltf::Document,
    buffers: &[Vec<u8>],
    source_path: Option<&Path>,
) -> Result<RobotModel, String> {
    let scene = doc
        .default_scene()
        .or_else(|| doc.scenes().next())
        .ok_or("glTF: file has no scene")?;
    let name = source_path
        .and_then(Path::file_stem)
        .map(|s| s.to_string_lossy().into_owned())
        .or_else(|| scene.name().map(str::to_string))
        .unwrap_or_else(|| "gltf_model".into());

    // Reserve node names first so the synthetic root never shadows one.
    let mut link_names: HashSet<String> = HashSet::new();
    let root = {
        let node_names: HashSet<&str> = doc.nodes().filter_map(|n| n.name()).collect();
        let mut root = "gltf_root".to_string();
        while node_names.contains(root.as_str()) {
            root.push('_');
        }
        root
    };
    link_names.insert(root.clone());

    let mut file = mn::MisaFile::new(&name, &root);
    file.link.push(mn::Link {
        name: root.clone(),
        description: String::new(),
        inertial: mn::Inertial::default(),
        visual: Vec::new(),
        collision: Vec::new(),
        collision_enabled: true,
    });

    let mut imp = GenericImport {
        buffers,
        file,
        link_names,
        joint_names: HashSet::new(),
        meshes: Vec::new(),
    };
    // glTF is Y-up; articara is Z-up.
    let y_up_to_z_up = z_up_to_y_up().inverse().to_homogeneous();
    for node in scene.nodes() {
        imp.visit(node, &y_up_to_z_up, &na::Isometry3::identity(), &root);
    }

    let base = source_path.unwrap_or(Path::new(""));
    let mut model = RobotModel::from_misa_file(&imp.file, base)?;
    for (link, slot, mesh) in imp.meshes {
        set_slot_geometry(&mut model, &link, slot, mesh_geom(mesh));
    }
    model.rebuild_misarta_model();
    Ok(model)
}

// =========================================================================
//  Tests
// =========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    fn arm_model() -> RobotModel {
        let mut model = RobotModel::new_empty("armbot");
        model
            .add_child(
                "base_link",
                "arm_link",
                "arm_joint",
                "revolute",
                na::Isometry3::from_parts(
                    na::Translation3::new(0.0, 0.0, 0.1),
                    na::UnitQuaternion::from_euler_angles(0.0, 0.0, 0.3),
                ),
                na::Vector3::y(),
                GeomData::Cylinder { radius: 0.02, half_length: 0.1 },
                [1.0, 0.0, 0.0, 1.0],
                -1.0,
                1.5,
            )
            .unwrap();
        model
    }

    fn one_triangle() -> GeomData {
        mesh_geom(MeshData::from_flat_vertices_f32(&[
            0.0, 0.0, 0.0, 0.0, 0.0, 1.0, //
            1.0, 0.0, 0.0, 0.0, 0.0, 1.0, //
            0.0, 1.0, 0.0, 0.0, 0.0, 1.0,
        ]))
    }

    #[test]
    fn glb_container_layout() {
        let glb = export_glb(&arm_model()).unwrap();
        assert_eq!(&glb[0..4], b"glTF");
        assert_eq!(u32::from_le_bytes(glb[4..8].try_into().unwrap()), 2);
        assert_eq!(u32::from_le_bytes(glb[8..12].try_into().unwrap()) as usize, glb.len());
        assert_eq!(&glb[16..20], b"JSON");
        let doc = gltf::Gltf::from_slice(&glb).unwrap();
        assert!(doc.blob.is_some());
        // Robot node + 2 links + 1 joint + 2 visuals.
        assert_eq!(doc.nodes().count(), 6);
    }

    #[test]
    fn roundtrip_preserves_kinematics() {
        let mut model = arm_model();
        model.joint_positions[0] = 0.7;
        let glb = export_glb(&model).unwrap();
        let imported = import_gltf_from_slice(&glb, None).unwrap();

        assert_eq!(imported.name, "armbot");
        assert_eq!(imported.root_link, "base_link");
        assert_eq!(imported.links.len(), 2);
        assert_eq!(imported.joints.len(), 1);
        let j = &imported.joints[0];
        assert_eq!(j.name, "arm_joint");
        assert_eq!(j.joint_type, "revolute");
        assert!((j.lower + 1.0).abs() < 1e-9 && (j.upper - 1.5).abs() < 1e-9);
        assert!((j.axis - na::Vector3::y()).norm() < 1e-6);
        assert!((j.origin.translation.vector.z - 0.1).abs() < 1e-6);

        let arm = &imported.links[imported.link_map["arm_link"]];
        match arm.visuals[0].geometry {
            GeomData::Cylinder { radius, half_length } => {
                assert!((radius - 0.02).abs() < 1e-6);
                assert!((half_length - 0.1).abs() < 1e-6);
            }
            _ => panic!("expected cylinder"),
        }
        assert_eq!(arm.visuals[0].color, [1.0, 0.0, 0.0, 1.0]);
    }

    #[test]
    fn roundtrip_meshes_visual_and_collision() {
        let mut model = RobotModel::new_empty("meshbot");
        model.links[0].visuals[0].geometry = one_triangle();
        model.links[0].collisions.push(CollisionData {
            origin: na::Isometry3::identity(),
            geometry: one_triangle(),
            physics: None,
        });

        for bytes in [
            export_glb(&model).unwrap(),
            export_gltf_embedded(&model).unwrap().into_bytes(),
        ] {
            let imported = import_gltf_from_slice(&bytes, None).unwrap();
            let link = &imported.links[0];
            for g in [&link.visuals[0].geometry, &link.collisions[0].geometry] {
                match g {
                    GeomData::Mesh { mesh, filename, .. } => {
                        assert_eq!(mesh.num_triangles(), 1);
                        assert!(filename.is_none());
                    }
                    _ => panic!("expected mesh"),
                }
            }
        }
    }

    #[test]
    fn export_node_pose_matches_fk() {
        let mut model = arm_model();
        model.joint_positions[0] = 0.5;
        let glb = export_glb(&model).unwrap();
        let gltf = gltf::Gltf::from_slice(&glb).unwrap();

        // Walk the scene and find arm_link's world transform (Y-up).
        fn find(node: gltf::Node, parent: na::Matrix4<f64>, out: &mut HashMap<String, na::Matrix4<f64>>) {
            let m = node.transform().matrix();
            let w = parent * na::Matrix4::from_fn(|r, c| m[c][r] as f64);
            out.insert(node.name().unwrap_or("").to_string(), w);
            for c in node.children() {
                find(c, w, out);
            }
        }
        let mut worlds = HashMap::new();
        for n in gltf.default_scene().unwrap().nodes() {
            find(n, na::Matrix4::identity(), &mut worlds);
        }
        let to_z_up = z_up_to_y_up().inverse().to_homogeneous();
        let got = to_z_up * worlds["arm_link"];
        let want = model.compute_transforms()["arm_link"].cast::<f64>().to_homogeneous();
        assert!((got - want).norm() < 1e-5, "got {got}, want {want}");
    }

    /// Minimal hand-written glTF: a scaled parent node with a child mesh
    /// node, no articara metadata.
    #[test]
    fn import_generic_scene() {
        let positions: [f32; 9] = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0];
        let bin: Vec<u8> = positions.iter().flat_map(|f| f.to_le_bytes()).collect();
        let b64 = base64::engine::general_purpose::STANDARD.encode(&bin);
        let doc = json!({
            "asset": { "version": "2.0" },
            "scene": 0,
            "scenes": [{ "nodes": [0] }],
            "nodes": [
                { "name": "body", "scale": [2.0, 2.0, 2.0], "translation": [0.0, 1.0, 0.0], "children": [1] },
                { "name": "part", "mesh": 0, "translation": [1.0, 0.0, 0.0] },
            ],
            "meshes": [{ "primitives": [{ "attributes": { "POSITION": 0 }, "material": 0 }] }],
            "materials": [{ "pbrMetallicRoughness": { "baseColorFactor": [0.1, 0.2, 0.3, 1.0] } }],
            "accessors": [{
                "bufferView": 0, "componentType": FLOAT, "count": 3, "type": "VEC3",
                "min": [0.0, 0.0, 0.0], "max": [1.0, 1.0, 0.0],
            }],
            "bufferViews": [{ "buffer": 0, "byteLength": bin.len() }],
            "buffers": [{ "byteLength": bin.len(), "uri": format!("data:application/octet-stream;base64,{b64}") }],
        });
        let bytes = serde_json::to_vec(&doc).unwrap();
        let model = import_gltf_from_slice(&bytes, None).unwrap();

        assert_eq!(model.root_link, "gltf_root");
        assert_eq!(model.links.len(), 3);
        assert_eq!(model.joints.len(), 2);
        assert!(model.joints.iter().all(|j| j.joint_type == "fixed"));

        // body sits at glTF (0, 1, 0) → Z-up (0, 0, 1); part at body + 2·(1,0,0).
        let tf = model.compute_transforms();
        assert!((tf["body"].translation.vector - na::Vector3::new(0.0, 0.0, 1.0)).norm() < 1e-5);
        assert!((tf["part"].translation.vector - na::Vector3::new(2.0, 0.0, 1.0)).norm() < 1e-5);

        // Scale baked into vertices (2×), in the link's Z-up-rotated frame.
        let part = &model.links[model.link_map["part"]];
        assert_eq!(part.visuals[0].color, [0.1, 0.2, 0.3, 1.0]);
        let GeomData::Mesh { mesh, .. } = &part.visuals[0].geometry else {
            panic!("expected mesh");
        };
        let (min, max) = mesh.aabb().unwrap();
        assert!(((max - min).norm() - 8f64.sqrt()).abs() < 1e-5);
    }
}
