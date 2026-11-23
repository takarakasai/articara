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
//! Named poses and sequences are also written as glTF animations
//! (`pose/<name>`: from the current pose to that pose; `sequence/<name>`:
//! the steps chained from the current pose), sampled at
//! [`ANIMATION_FPS`] so viewers can play them back. They are derived data:
//! import ignores them and restores poses / sequences from the `.misa`.
//!
//! ## Import
//!
//! - **With `extras.articara`** (files written by articara): the model is
//!   rebuilt from the embedded `.misa` and the mesh triangles reattached.
//! - **Plain glTF** (Blender, CAD exporters, …): every node becomes a link
//!   attached to its parent by a fixed joint. Non-rigid parts of node
//!   transforms (scale / shear) are baked into the mesh vertices, and the
//!   Y-up scene is rotated into articara's Z-up frame. Each mesh primitive
//!   becomes one visual coloured by its material's base colour. Custom
//!   properties on a node (see [`NodeProps`]) turn its joint into a
//!   revolute / continuous / prismatic one and set limits, mass and
//!   collision, so a robot can be assembled in Blender. Animations that
//!   move those joints become poses (`<animation>/<k>`, one per keyframe)
//!   chained by a sequence `<animation>`.

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
    /// Joint index → node of the child link it moves (animation target).
    joint_child_node: HashMap<usize, usize>,
    animations: Vec<Value>,
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

    /// Float accessor of `N`-component elements (no buffer-view target, as
    /// for animation samplers). `with_bounds` adds the min / max that
    /// sampler inputs require.
    fn push_floats<const N: usize>(&mut self, data: &[[f32; N]], with_bounds: bool) -> usize {
        let bytes: Vec<u8> = data
            .iter()
            .flat_map(|v| v.iter().flat_map(|c| c.to_le_bytes()))
            .collect();
        while self.bin.len() % 4 != 0 {
            self.bin.push(0);
        }
        self.buffer_views.push(json!({
            "buffer": 0,
            "byteOffset": self.bin.len(),
            "byteLength": bytes.len(),
        }));
        self.bin.extend_from_slice(&bytes);
        let ty = match N {
            1 => "SCALAR",
            3 => "VEC3",
            4 => "VEC4",
            _ => unreachable!("unsupported accessor width {N}"),
        };
        let mut acc = json!({
            "bufferView": self.buffer_views.len() - 1,
            "componentType": FLOAT,
            "count": data.len(),
            "type": ty,
        });
        if with_bounds {
            let mut min = [f32::INFINITY; N];
            let mut max = [f32::NEG_INFINITY; N];
            for v in data {
                for k in 0..N {
                    min[k] = min[k].min(v[k]);
                    max[k] = max[k].max(v[k]);
                }
            }
            acc["min"] = json!(min.as_slice());
            acc["max"] = json!(max.as_slice());
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

    export_animations(model, &mut b);

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
    }
    if !b.animations.is_empty() {
        doc["animations"] = json!(b.animations);
    }
    if !b.bin.is_empty() {
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

    for &ji in model.children_joints.get(link_name).map(Vec::as_slice).unwrap_or(&[]) {
        let joint = &model.joints[ji];
        let origin = joint.origin.cast::<f64>();
        let motion = joint_motion(model, ji, world);
        let child_idx = export_link(model, &joint.child_link, &motion, world, b, collision_nodes)?;
        b.joint_child_node.insert(ji, child_idx);
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

/// Child-link pose relative to joint `ji`'s frame (= the joint motion at
/// the configuration `world` was computed for).
fn joint_motion(
    model: &RobotModel,
    ji: usize,
    world: &HashMap<String, na::Isometry3<f64>>,
) -> na::Isometry3<f64> {
    let joint = &model.joints[ji];
    match (world.get(&joint.parent_link), world.get(&joint.child_link)) {
        (Some(p), Some(c)) => joint.origin.cast::<f64>().inverse() * p.inverse() * c,
        _ => na::Isometry3::identity(),
    }
}

/// Sampling rate of exported animations. Samples are joined with glTF
/// `LINEAR` interpolation, so the smooth (cubic / quintic) profiles are
/// reproduced closely without the viewer knowing about them.
const ANIMATION_FPS: f64 = 30.0;

/// One joint-space segment of an exported animation.
struct Segment {
    q_end: Vec<f64>,
    duration: f64,
    kind: misarta::trajectory::InterpolationKind,
}

/// Emit one glTF animation per named pose (a transition from the current
/// pose) and per sequence (its steps chained from the current pose), so
/// viewers such as Blender or three.js can play them back.
fn export_animations(model: &RobotModel, b: &mut GltfBuilder) {
    let current = model.joint_positions.clone();
    for pose in &model.poses {
        let segment = Segment {
            q_end: pose.to_vector(model, &current),
            duration: pose.duration,
            kind: pose.kind,
        };
        add_animation(model, b, &format!("pose/{}", pose.name), &current, &[segment]);
    }
    for seq in &model.sequences {
        let mut q = current.clone();
        let mut segments = Vec::new();
        for step in &seq.steps {
            let Some(pose) = model.poses.iter().find(|p| p.name == step.pose_name) else {
                log::warn!(
                    "glTF export: sequence '{}' references unknown pose '{}', step skipped",
                    seq.name,
                    step.pose_name
                );
                continue;
            };
            let q_end = pose.to_vector(model, &q);
            segments.push(Segment { q_end: q_end.clone(), duration: step.duration, kind: step.kind });
            q = q_end;
        }
        add_animation(model, b, &format!("sequence/{}", seq.name), &current, &segments);
    }
}

fn add_animation(model: &RobotModel, b: &mut GltfBuilder, name: &str, q_start: &[f64], segments: &[Segment]) {
    // Movable joints and the node property each one drives.
    let channels: Vec<(usize, &'static str)> = model
        .joints
        .iter()
        .enumerate()
        .filter(|(ji, _)| b.joint_child_node.contains_key(ji))
        .flat_map(|(ji, j)| match j.joint_type.as_str() {
            "fixed" => vec![],
            "prismatic" => vec![(ji, "translation")],
            "revolute" | "continuous" => vec![(ji, "rotation")],
            _ => vec![(ji, "translation"), (ji, "rotation")],
        })
        .collect();
    if channels.is_empty() {
        return;
    }

    // Sample the joint trajectory: t = 0, then each segment at the fixed
    // rate including its end point.
    let mut samples: Vec<(f64, Vec<f64>)> = vec![(0.0, q_start.to_vec())];
    let mut t0 = 0.0;
    let mut q0 = q_start.to_vec();
    for seg in segments {
        let duration = seg.duration.max(0.0);
        let transition =
            misarta::trajectory::PoseTransition::new(q0.clone(), seg.q_end.clone(), duration, seg.kind);
        let n = ((duration * ANIMATION_FPS).ceil() as usize).max(1);
        for k in 1..=n {
            let t = duration * k as f64 / n as f64;
            samples.push((t0 + t, transition.evaluate(t)));
        }
        t0 += duration;
        q0 = seg.q_end.clone();
    }

    // Through FK so mimic joints and joint conventions match the viewport.
    let mut posed = model.clone();
    if posed.misarta_cache.is_none() {
        posed.rebuild_misarta_model();
    }
    let mut times = Vec::with_capacity(samples.len());
    let mut values: Vec<Vec<[f32; 4]>> = vec![Vec::with_capacity(samples.len()); channels.len()];
    for (t, q) in &samples {
        posed.joint_positions = q.clone();
        posed.enforce_mimic();
        let world: HashMap<String, na::Isometry3<f64>> = posed
            .compute_transforms()
            .into_iter()
            .map(|(name, iso)| (name, iso.cast::<f64>()))
            .collect();
        times.push([*t as f32]);
        for (ci, &(ji, path)) in channels.iter().enumerate() {
            let m = joint_motion(model, ji, &world);
            values[ci].push(match path {
                "translation" => {
                    let v = m.translation.vector;
                    [v.x as f32, v.y as f32, v.z as f32, 0.0]
                }
                _ => {
                    let q = m.rotation.quaternion();
                    [q.i as f32, q.j as f32, q.k as f32, q.w as f32]
                }
            });
        }
    }
    // Keep consecutive quaternions in the same hemisphere so LINEAR (slerp)
    // takes the short way round.
    for (ci, &(_, path)) in channels.iter().enumerate() {
        if path == "rotation" {
            for k in 1..values[ci].len() {
                let (prev, cur) = (values[ci][k - 1], values[ci][k]);
                if prev.iter().zip(&cur).map(|(a, b)| a * b).sum::<f32>() < 0.0 {
                    values[ci][k] = cur.map(|c| -c);
                }
            }
        }
    }

    let input = b.push_floats(&times, true);
    let mut samplers = Vec::new();
    let mut anim_channels = Vec::new();
    for (ci, &(ji, path)) in channels.iter().enumerate() {
        let output = if path == "translation" {
            let v3: Vec<[f32; 3]> = values[ci].iter().map(|v| [v[0], v[1], v[2]]).collect();
            b.push_floats(&v3, false)
        } else {
            b.push_floats(&values[ci], false)
        };
        samplers.push(json!({ "input": input, "output": output, "interpolation": "LINEAR" }));
        anim_channels.push(json!({
            "sampler": samplers.len() - 1,
            "target": { "node": b.joint_child_node[&ji], "path": path },
        }));
    }
    b.animations.push(json!({ "name": name, "channels": anim_channels, "samplers": samplers }));
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

    let meta = doc.nodes().find_map(|n| {
        let mut extras = parse_extras(n.extras())?;
        extras.get_mut(META_KEY).map(Value::take)
    });
    // articara's own animations are derived from the poses / sequences in
    // the embedded `.misa`, so only foreign ones are worth a warning.
    if doc.skins().next().is_some() || (meta.is_none() && doc.animations().next().is_some()) {
        log::warn!("glTF import: skins / animations are ignored");
    }
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

/// Joint / link description read from a plain glTF node's custom
/// properties (`extras`). Blender writes an object's custom properties
/// there when exported with *Include → Custom Properties*.
///
/// | property         | meaning                                              |
/// |------------------|------------------------------------------------------|
/// | `joint_type`     | `revolute` / `continuous` / `prismatic` / `fixed`     |
/// | `joint_axis`     | `[x, y, z]` in the node's local Z-up frame (default `[0, 0, 1]`) |
/// | `joint_lower` / `joint_upper` | limits in rad or m (revolute default ±π) |
/// | `joint_effort` / `joint_velocity` | actuator limits (0 = undeclared)  |
/// | `joint_name`     | joint name (default `<link>_joint`)                   |
/// | `mass`           | link mass in kg; inertia is computed from its meshes |
/// | `collision`      | `true` to reuse the node's meshes as collision geometry |
///
/// The joint connects the parent node's link to this node's link, with
/// the node's transform in the file as the zero position. Link frames are
/// the nodes' frames brought back to Z-up, i.e. exactly the Blender
/// object's local axes, so `joint_axis` is written as seen in Blender:
/// `[0, 0, 1]` is the object's local Z.
#[derive(Default)]
struct NodeProps {
    joint_type: Option<String>,
    joint_axis: Option<[f64; 3]>,
    joint_lower: Option<f64>,
    joint_upper: Option<f64>,
    joint_effort: Option<f64>,
    joint_velocity: Option<f64>,
    joint_name: Option<String>,
    mass: Option<f64>,
    collision: bool,
}

impl NodeProps {
    fn from_node(node: &gltf::Node<'_>) -> Self {
        let Some(extras) = parse_extras(node.extras()) else {
            return Self::default();
        };
        let num = |k: &str| extras.get(k).and_then(Value::as_f64);
        let text = |k: &str| extras.get(k).and_then(Value::as_str).map(str::to_string);
        let axis = extras.get("joint_axis").and_then(Value::as_array).and_then(|a| {
            let v: Vec<f64> = a.iter().filter_map(Value::as_f64).collect();
            (v.len() == 3).then(|| [v[0], v[1], v[2]])
        });
        Self {
            joint_type: text("joint_type"),
            joint_axis: axis,
            joint_lower: num("joint_lower"),
            joint_upper: num("joint_upper"),
            joint_effort: num("joint_effort"),
            joint_velocity: num("joint_velocity"),
            joint_name: text("joint_name"),
            mass: num("mass"),
            // Blender stores booleans as 0 / 1 integers.
            collision: extras
                .get("collision")
                .is_some_and(|v| v.as_bool().unwrap_or(false) || v.as_f64().is_some_and(|x| x != 0.0)),
        }
    }

    fn joint_kind(&self, node_name: &str) -> mn::JointKind {
        match self.joint_type.as_deref().map(str::to_ascii_lowercase).as_deref() {
            None | Some("fixed") => mn::JointKind::Fixed,
            Some("revolute") => mn::JointKind::Revolute,
            Some("continuous") => mn::JointKind::Continuous,
            Some("prismatic") => mn::JointKind::Prismatic,
            Some(other) => {
                log::warn!("glTF import: node '{node_name}': unsupported joint_type '{other}', using fixed");
                mn::JointKind::Fixed
            }
        }
    }
}

/// Link frame of a node from its world matrix: the rigid part brought
/// back to Z-up, plus the residual linear map (scale / shear) that takes
/// node-local glTF coordinates into that frame.
fn link_frame(world: &na::Matrix4<f64>) -> (na::Isometry3<f64>, na::Matrix3<f64>) {
    let (rigid, residual) = split_rigid(world);
    // A Y-up exporter (Blender) conjugates every node's frame by the axis
    // conversion; undo it per node so link frames are Z-up again and match
    // the object axes the author saw.
    (
        rigid * na::Isometry3::from_parts(na::Translation3::identity(), z_up_to_y_up()),
        z_up_to_y_up().inverse().to_rotation_matrix().matrix() * residual,
    )
}

/// Link frames of every node under `scene`, given each node's local matrix.
fn scene_link_frames(
    scene: &gltf::Scene<'_>,
    local: &dyn Fn(&gltf::Node<'_>) -> na::Matrix4<f64>,
) -> HashMap<usize, na::Isometry3<f64>> {
    fn walk(
        node: gltf::Node<'_>,
        parent_world: &na::Matrix4<f64>,
        local: &dyn Fn(&gltf::Node<'_>) -> na::Matrix4<f64>,
        out: &mut HashMap<usize, na::Isometry3<f64>>,
    ) {
        let world = parent_world * local(&node);
        out.insert(node.index(), link_frame(&world).0);
        for child in node.children() {
            walk(child, &world, local, out);
        }
    }
    let mut out = HashMap::new();
    // glTF is Y-up; articara is Z-up.
    let y_up_to_z_up = z_up_to_y_up().inverse().to_homogeneous();
    for node in scene.nodes() {
        walk(node, &y_up_to_z_up, local, &mut out);
    }
    out
}

/// glTF's column-major matrix → nalgebra.
fn gltf_matrix(m: [[f32; 4]; 4]) -> na::Matrix4<f64> {
    na::Matrix4::from_fn(|r, c| m[c][r] as f64)
}

fn node_matrix(node: &gltf::Node<'_>) -> na::Matrix4<f64> {
    gltf_matrix(node.transform().matrix())
}

/// A movable joint created from a node, kept to convert animations.
struct NodeJoint {
    node: usize,
    /// Node of the parent link; `None` for the synthetic root.
    parent_node: Option<usize>,
    joint: String,
    kind: mn::JointKind,
    axis: na::Vector3<f64>,
    origin: na::Isometry3<f64>,
}

struct GenericImport<'a> {
    buffers: &'a [Vec<u8>],
    file: mn::MisaFile,
    link_names: HashSet<String>,
    joint_names: HashSet<String>,
    meshes: Vec<(String, Slot, MeshData)>,
    /// Link name → mass from a `mass` custom property.
    masses: Vec<(String, f64)>,
    joints: Vec<NodeJoint>,
}

impl GenericImport<'_> {
    fn visit(
        &mut self,
        node: gltf::Node<'_>,
        parent_world: &na::Matrix4<f64>,
        parent_rigid: &na::Isometry3<f64>,
        parent: (&str, Option<usize>),
    ) {
        let (parent_link, parent_node) = parent;
        let world = parent_world * node_matrix(&node);
        let (rigid, residual) = link_frame(&world);

        let base = node.name().map(str::to_string).unwrap_or_else(|| format!("node_{}", node.index()));
        let link_name = unique_name(&base, &mut self.link_names);
        let props = NodeProps::from_node(&node);
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
                if props.collision {
                    self.meshes.push((link_name.clone(), Slot::Collision(link.collision.len()), data.clone()));
                    link.collision.push(mn::Collision {
                        origin: mn::Origin::default(),
                        geom: placeholder_geom(),
                        physics: None,
                    });
                }
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

        if let Some(mass) = props.mass {
            self.masses.push((link_name.clone(), mass));
        }

        let kind = props.joint_kind(&link_name);
        // The axis is given in the node's local (Z-up) frame; map it into
        // the link frame through the residual so scale doesn't skew it.
        let axis_local = props.joint_axis.map(na::Vector3::from).unwrap_or_else(na::Vector3::z);
        let axis = (residual * (z_up_to_y_up() * axis_local))
            .try_normalize(1e-12)
            .unwrap_or_else(na::Vector3::z);
        let default_range = match kind {
            mn::JointKind::Revolute => (-std::f64::consts::PI, std::f64::consts::PI),
            _ => (0.0, 0.0),
        };
        let joint_base = props.joint_name.clone().unwrap_or_else(|| format!("{link_name}_joint"));
        let joint_name = unique_name(&joint_base, &mut self.joint_names);
        let origin = parent_rigid.inverse() * rigid;
        if kind != mn::JointKind::Fixed {
            self.joints.push(NodeJoint {
                node: node.index(),
                parent_node,
                joint: joint_name.clone(),
                kind,
                axis,
                origin,
            });
        }
        self.file.joint.push(mn::Joint {
            name: joint_name,
            kind,
            parent: parent_link.to_string(),
            child: link_name.clone(),
            axis: [axis.x, axis.y, axis.z],
            origin: misa_origin(&origin),
            limit: mn::JointLimit {
                lower: props.joint_lower.unwrap_or(default_range.0),
                upper: props.joint_upper.unwrap_or(default_range.1),
                effort: props.joint_effort.unwrap_or(0.0),
                velocity: props.joint_velocity.unwrap_or(0.0),
            },
            dynamics: mn::JointDynamics::default(),
        });

        for child in node.children() {
            self.visit(child, &world, &rigid, (&link_name, Some(node.index())));
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
        masses: Vec::new(),
        joints: Vec::new(),
    };
    // glTF is Y-up; articara is Z-up.
    let y_up_to_z_up = z_up_to_y_up().inverse().to_homogeneous();
    for node in scene.nodes() {
        imp.visit(node, &y_up_to_z_up, &na::Isometry3::identity(), (&root, None));
    }

    let base = source_path.unwrap_or(Path::new(""));
    let mut model = RobotModel::from_misa_file(&imp.file, base)?;
    for (link, slot, mesh) in imp.meshes {
        set_slot_geometry(&mut model, &link, slot, mesh_geom(mesh));
    }
    for (link, mass) in imp.masses {
        if let Some(&li) = model.link_map.get(&link) {
            let l = &mut model.links[li];
            l.inertial = compute_link_inertia(&l.visuals, mass);
        }
    }
    model.rebuild_misarta_model();
    import_animations(doc, &scene, buffers, &imp.joints, &mut model);
    Ok(model)
}

// ─── Animations → poses / sequences ─────────────────────────────────────

/// Most keyframes kept per imported animation; denser ones (baked
/// exports) are resampled uniformly to this many.
const MAX_ANIMATION_KEYS: usize = 120;

/// One animation channel, read into memory.
struct Track {
    node: usize,
    property: gltf::animation::Property,
    interpolation: gltf::animation::Interpolation,
    times: Vec<f32>,
    /// Translation `[x, y, z, _]`, rotation `[x, y, z, w]` or scale
    /// `[x, y, z, _]`. For CUBICSPLINE: `[in-tangent, value, out-tangent]`
    /// triples, as stored.
    values: Vec<[f32; 4]>,
}

impl Track {
    fn read(channel: &gltf::animation::Channel<'_>, buffers: &[Vec<u8>]) -> Option<Self> {
        use gltf::animation::util::ReadOutputs;
        let reader = channel.reader(|b| buffers.get(b.index()).map(Vec::as_slice));
        let times: Vec<f32> = reader.read_inputs()?.collect();
        let values: Vec<[f32; 4]> = match reader.read_outputs()? {
            ReadOutputs::Translations(it) | ReadOutputs::Scales(it) => it.map(|v| [v[0], v[1], v[2], 0.0]).collect(),
            ReadOutputs::Rotations(it) => it.into_f32().collect(),
            ReadOutputs::MorphTargetWeights(_) => return None,
        };
        let interpolation = channel.sampler().interpolation();
        let per_key = if interpolation == gltf::animation::Interpolation::CubicSpline { 3 } else { 1 };
        if times.is_empty() || values.len() != times.len() * per_key {
            return None;
        }
        Some(Self {
            node: channel.target().node().index(),
            property: channel.target().property(),
            interpolation,
            times,
            values,
        })
    }

    /// Value at time `t` (clamped to the keyframe range).
    fn sample(&self, t: f32) -> [f32; 4] {
        use gltf::animation::Interpolation;
        let cubic = self.interpolation == Interpolation::CubicSpline;
        let value = |k: usize| if cubic { self.values[3 * k + 1] } else { self.values[k] };
        let n = self.times.len();
        if t <= self.times[0] {
            return value(0);
        }
        if t >= self.times[n - 1] {
            return value(n - 1);
        }
        let k = self.times.partition_point(|&x| x <= t) - 1;
        let dt = self.times[k + 1] - self.times[k];
        let u = if dt > 0.0 { (t - self.times[k]) / dt } else { 0.0 };
        let rotation = self.property == gltf::animation::Property::Rotation;
        let out = match self.interpolation {
            Interpolation::Step => value(k),
            Interpolation::Linear if rotation => {
                let q = |v: [f32; 4]| na::UnitQuaternion::from_quaternion(na::Quaternion::new(v[3], v[0], v[1], v[2]));
                let r = q(value(k)).slerp(&q(value(k + 1)), u);
                let c = r.quaternion();
                [c.i, c.j, c.k, c.w]
            }
            Interpolation::Linear => {
                let (a, b) = (value(k), value(k + 1));
                std::array::from_fn(|i| a[i] + (b[i] - a[i]) * u)
            }
            Interpolation::CubicSpline => {
                let (v0, b0) = (self.values[3 * k + 1], self.values[3 * k + 2]);
                let (a1, v1) = (self.values[3 * (k + 1)], self.values[3 * (k + 1) + 1]);
                let (u2, u3) = (u * u, u * u * u);
                std::array::from_fn(|i| {
                    (2.0 * u3 - 3.0 * u2 + 1.0) * v0[i]
                        + (u3 - 2.0 * u2 + u) * dt * b0[i]
                        + (-2.0 * u3 + 3.0 * u2) * v1[i]
                        + (u3 - u2) * dt * a1[i]
                })
            }
        };
        if rotation {
            let n = out.iter().map(|c| c * c).sum::<f32>().sqrt();
            if n > 0.0 {
                return out.map(|c| c / n);
            }
        }
        out
    }
}

/// Turn the file's animations into articara poses and sequences: each
/// animation's keyframe times become poses `<animation>/<k>` (joint values
/// recovered from the animated node transforms) chained by a sequence
/// `<animation>` with linear steps. Only nodes that became movable joints
/// can be represented; motion of other nodes is reported and dropped.
fn import_animations(
    doc: &gltf::Document,
    scene: &gltf::Scene<'_>,
    buffers: &[Vec<u8>],
    joints: &[NodeJoint],
    model: &mut RobotModel,
) {
    let joint_nodes: HashSet<usize> = joints.iter().map(|j| j.node).collect();
    let mut pose_names: HashSet<String> = model.poses.iter().map(|p| p.name.clone()).collect();
    let mut seq_names: HashSet<String> = model.sequences.iter().map(|s| s.name.clone()).collect();

    for (ai, anim) in doc.animations().enumerate() {
        let tracks: Vec<Track> = anim.channels().filter_map(|c| Track::read(&c, buffers)).collect();
        let dropped: HashSet<usize> = tracks
            .iter()
            .filter(|t| !joint_nodes.contains(&t.node))
            .map(|t| t.node)
            .collect();
        let anim_name = anim.name().map(str::to_string).unwrap_or_else(|| format!("animation_{ai}"));
        if !dropped.is_empty() {
            log::warn!(
                "glTF import: animation '{anim_name}' moves {} node(s) that are not movable joints; that motion is dropped",
                dropped.len()
            );
        }
        if !tracks.iter().any(|t| joint_nodes.contains(&t.node)) {
            continue;
        }

        let mut times: Vec<f32> = tracks.iter().flat_map(|t| t.times.iter().copied()).collect();
        times.sort_by(f32::total_cmp);
        times.dedup_by(|a, b| (*a - *b).abs() < 1e-6);
        if times.len() > MAX_ANIMATION_KEYS {
            let (t0, t1) = (times[0], times[times.len() - 1]);
            let n = MAX_ANIMATION_KEYS - 1;
            times = (0..=n).map(|k| t0 + (t1 - t0) * k as f32 / n as f32).collect();
        }

        // Joint values at each keyframe time.
        let mut keys: Vec<std::collections::BTreeMap<String, f64>> = Vec::with_capacity(times.len());
        let mut previous: HashMap<&str, f64> = HashMap::new();
        for &t in &times {
            let local = |node: &gltf::Node<'_>| -> na::Matrix4<f64> {
                let (mut tr, mut rot, mut sc) = node.transform().decomposed();
                for track in tracks.iter().filter(|tk| tk.node == node.index()) {
                    let v = track.sample(t);
                    match track.property {
                        gltf::animation::Property::Translation => tr = [v[0], v[1], v[2]],
                        gltf::animation::Property::Rotation => rot = v,
                        gltf::animation::Property::Scale => sc = [v[0], v[1], v[2]],
                        gltf::animation::Property::MorphTargetWeights => {}
                    }
                }
                gltf_matrix(gltf::scene::Transform::Decomposed { translation: tr, rotation: rot, scale: sc }.matrix())
            };
            let frames = scene_link_frames(scene, &local);
            let mut angles = std::collections::BTreeMap::new();
            for j in joints {
                let (Some(child), parent) = (frames.get(&j.node), j.parent_node.map(|p| frames.get(&p))) else {
                    continue;
                };
                let parent = match parent {
                    Some(Some(p)) => *p,
                    Some(None) => continue,
                    None => na::Isometry3::identity(),
                };
                let motion = j.origin.inverse() * parent.inverse() * child;
                let mut q = match j.kind {
                    mn::JointKind::Prismatic => motion.translation.vector.dot(&j.axis),
                    _ => {
                        let r = motion.rotation.quaternion();
                        2.0 * r.imag().dot(&j.axis).atan2(r.w)
                    }
                };
                if j.kind != mn::JointKind::Prismatic {
                    // Keep consecutive angles continuous (no ±2π jumps).
                    q = q.rem_euclid(std::f64::consts::TAU);
                    if q > std::f64::consts::PI {
                        q -= std::f64::consts::TAU;
                    }
                    if let Some(&prev) = previous.get(j.joint.as_str()) {
                        q += ((prev - q) / std::f64::consts::TAU).round() * std::f64::consts::TAU;
                    }
                }
                previous.insert(&j.joint, q);
                angles.insert(j.joint.clone(), q);
            }
            keys.push(angles);
        }

        use misarta::trajectory::InterpolationKind;
        if keys.len() == 1 {
            let name = unique_name(&anim_name, &mut pose_names);
            model.poses.push(NamedPose {
                name,
                angles: keys.pop().unwrap_or_default(),
                duration: 1.0,
                kind: InterpolationKind::QuinticSmooth,
            });
            continue;
        }
        let mut steps = Vec::with_capacity(keys.len());
        let mut t_prev = 0.0;
        for (k, (angles, &t)) in keys.into_iter().zip(&times).enumerate() {
            let pose_name = unique_name(&format!("{anim_name}/{k}"), &mut pose_names);
            let duration = (t - t_prev).max(0.0) as f64;
            t_prev = t;
            model.poses.push(NamedPose {
                name: pose_name.clone(),
                angles,
                duration,
                kind: InterpolationKind::Linear,
            });
            steps.push(SequenceStep { pose_name, duration, kind: InterpolationKind::Linear });
        }
        model.sequences.push(Sequence { name: unique_name(&anim_name, &mut seq_names), steps });
    }
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

    fn arm_model_with_motions() -> RobotModel {
        use misarta::trajectory::InterpolationKind;
        let mut model = arm_model();
        let mut raised = NamedPose::snapshot("raised", &model, 1.0, InterpolationKind::QuinticSmooth);
        raised.angles.insert("arm_joint".into(), 1.2);
        let mut lowered = raised.clone();
        lowered.name = "lowered".into();
        lowered.angles.insert("arm_joint".into(), -0.8);
        lowered.duration = 0.5;
        model.poses = vec![raised, lowered];
        model.sequences = vec![Sequence {
            name: "wave".into(),
            steps: vec![
                SequenceStep { pose_name: "raised".into(), duration: 1.0, kind: InterpolationKind::Linear },
                SequenceStep { pose_name: "lowered".into(), duration: 0.5, kind: InterpolationKind::CubicSmooth },
            ],
        }];
        model
    }

    #[test]
    fn poses_and_sequences_become_animations() {
        let model = arm_model_with_motions();
        let glb = export_glb(&model).unwrap();
        let gltf = gltf::Gltf::from_slice(&glb).unwrap();
        let buffers = load_buffers(&gltf, None).unwrap();

        let names: Vec<_> = gltf.animations().map(|a| a.name().unwrap().to_string()).collect();
        assert_eq!(names, ["pose/raised", "pose/lowered", "sequence/wave"]);

        let wave = gltf.animations().find(|a| a.name() == Some("sequence/wave")).unwrap();
        let channel = wave.channels().next().unwrap();
        assert_eq!(channel.target().node().name(), Some("arm_link"));
        assert_eq!(channel.target().property(), gltf::animation::Property::Rotation);

        let reader = channel.reader(|b| buffers.get(b.index()).map(Vec::as_slice));
        let times: Vec<f32> = reader.read_inputs().unwrap().collect();
        assert_eq!(times[0], 0.0);
        assert!((times.last().unwrap() - 1.5).abs() < 1e-6);
        let gltf::animation::util::ReadOutputs::Rotations(rot) = reader.read_outputs().unwrap() else {
            panic!("expected rotations");
        };
        let rot: Vec<[f32; 4]> = rot.into_f32().collect();
        // Rotation about the joint's Y axis: angle at the end of each step.
        let angle_at = |t: f32| {
            let k = times.iter().position(|&x| (x - t).abs() < 1e-6).unwrap();
            let [x, y, z, w] = rot[k];
            assert!(x.abs() < 1e-5 && z.abs() < 1e-5);
            2.0 * y.atan2(w)
        };
        assert!(angle_at(0.0).abs() < 1e-5);
        assert!((angle_at(1.0) - 1.2).abs() < 1e-4);
        assert!((angle_at(1.5) + 0.8).abs() < 1e-4);
    }

    #[test]
    fn animations_do_not_disturb_roundtrip() {
        let model = arm_model_with_motions();
        let imported = import_gltf_from_slice(&export_glb(&model).unwrap(), None).unwrap();
        assert_eq!(imported.poses.len(), 2);
        assert_eq!(imported.sequences.len(), 1);
        assert_eq!(imported.sequences[0].steps.len(), 2);
    }

    /// glTF with one triangle mesh and the given nodes / scene roots.
    fn triangle_gltf(nodes: Value, roots: Value) -> Vec<u8> {
        let positions: [f32; 9] = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0];
        let bin: Vec<u8> = positions.iter().flat_map(|f| f.to_le_bytes()).collect();
        let b64 = base64::engine::general_purpose::STANDARD.encode(&bin);
        serde_json::to_vec(&json!({
            "asset": { "version": "2.0" },
            "scene": 0,
            "scenes": [{ "nodes": roots }],
            "nodes": nodes,
            "meshes": [{ "primitives": [{ "attributes": { "POSITION": 0 } }] }],
            "accessors": [{
                "bufferView": 0, "componentType": FLOAT, "count": 3, "type": "VEC3",
                "min": [0.0, 0.0, 0.0], "max": [1.0, 1.0, 0.0],
            }],
            "bufferViews": [{ "buffer": 0, "byteLength": bin.len() }],
            "buffers": [{ "byteLength": bin.len(), "uri": format!("data:application/octet-stream;base64,{b64}") }],
        }))
        .unwrap()
    }

    #[test]
    fn import_joints_from_custom_properties() {
        let bytes = triangle_gltf(
            json!([
                { "name": "base", "mesh": 0, "children": [1, 2], "extras": { "mass": 2.0, "collision": 1 } },
                // Elbow 0.5 up (glTF Y = articara Z), hinge about the
                // vertical: local Z in the Z-up convention.
                { "name": "arm", "mesh": 0, "translation": [0.0, 0.5, 0.0], "extras": {
                    "joint_type": "revolute", "joint_axis": [0.0, 0.0, 1.0],
                    "joint_lower": -0.5, "joint_upper": 2.0, "joint_name": "elbow",
                    "joint_effort": 10.0,
                } },
                { "name": "slider", "scale": [2.0, 2.0, 2.0], "extras": {
                    "joint_type": "PRISMATIC", "joint_axis": [1.0, 0.0, 0.0], "joint_upper": 0.3,
                } },
            ]),
            json!([0]),
        );
        let model = import_gltf_from_slice(&bytes, None).unwrap();

        // Unrotated nodes give world-aligned (Z-up) link frames.
        let tf = model.compute_transforms();
        assert!(tf["arm"].rotation.angle() < 1e-6);

        let elbow = &model.joints[model.joint_map["elbow"]];
        assert_eq!(elbow.joint_type, "revolute");
        assert_eq!((elbow.parent_link.as_str(), elbow.child_link.as_str()), ("base", "arm"));
        assert!((elbow.axis - na::Vector3::z()).norm() < 1e-6, "{:?}", elbow.axis);
        assert_eq!((elbow.lower, elbow.upper, elbow.effort), (-0.5, 2.0, 10.0));
        assert!((elbow.origin.translation.vector - na::Vector3::new(0.0, 0.0, 0.5)).norm() < 1e-6);

        let slider = &model.joints[model.joint_map["slider_joint"]];
        assert_eq!(slider.joint_type, "prismatic");
        // Scale must not leak into the axis length.
        assert!((slider.axis - na::Vector3::x()).norm() < 1e-6, "{:?}", slider.axis);
        assert_eq!((slider.lower, slider.upper), (0.0, 0.3));

        // The joint actually moves the child.
        let mut posed = model.clone();
        posed.joint_positions[model.joint_map["elbow"]] = std::f64::consts::FRAC_PI_2;
        let before = model.compute_transforms()["arm"].rotation;
        let after = posed.compute_transforms()["arm"].rotation;
        assert!((before.angle_to(&after) - std::f32::consts::FRAC_PI_2).abs() < 1e-5);

        let base = &model.links[model.link_map["base"]];
        assert_eq!(base.inertial.mass, 2.0);
        assert_eq!(base.collisions.len(), 1);
        assert!(matches!(base.collisions[0].geometry, GeomData::Mesh { .. }));
        // No properties → fixed joint, no collision.
        assert!(model.links[model.link_map["arm"]].collisions.is_empty());
        assert_eq!(model.joints[model.joint_map["base_joint"]].joint_type, "fixed");
    }

    #[test]
    fn import_animation_as_poses_and_sequence() {
        use std::f32::consts::PI;
        // Rotation keys about glTF Y (= the nodes' local Z-up axis Z).
        let about_y = |deg: f32| {
            let h = deg.to_radians() / 2.0;
            [0.0, h.sin(), 0.0, h.cos()]
        };
        let times: [f32; 3] = [0.0, 0.5, 1.0];
        let elbow: Vec<[f32; 4]> = [0.0, 45.0, 90.0].map(about_y).to_vec();
        // Continuous spin 0 → 170 → 340°: the raw angle wraps past 180°.
        let spin: Vec<[f32; 4]> = [0.0, 170.0, 340.0].map(about_y).to_vec();
        let mut bin: Vec<u8> = times.iter().flat_map(|f| f.to_le_bytes()).collect();
        for q in elbow.iter().chain(&spin) {
            bin.extend(q.iter().flat_map(|f| f.to_le_bytes()));
        }
        let b64 = base64::engine::general_purpose::STANDARD.encode(&bin);
        let doc = json!({
            "asset": { "version": "2.0" },
            "scene": 0,
            "scenes": [{ "nodes": [0] }],
            "nodes": [
                { "name": "base", "children": [1, 2] },
                { "name": "arm", "translation": [0.0, 0.5, 0.0],
                  "extras": { "joint_type": "revolute", "joint_axis": [0.0, 0.0, 1.0], "joint_name": "elbow" } },
                { "name": "wheel", "extras": { "joint_type": "continuous", "joint_axis": [0.0, 0.0, 1.0] } },
            ],
            "accessors": [
                { "bufferView": 0, "componentType": FLOAT, "count": 3, "type": "SCALAR", "min": [0.0], "max": [1.0] },
                { "bufferView": 1, "componentType": FLOAT, "count": 3, "type": "VEC4" },
                { "bufferView": 2, "componentType": FLOAT, "count": 3, "type": "VEC4" },
            ],
            "bufferViews": [
                { "buffer": 0, "byteOffset": 0, "byteLength": 12 },
                { "buffer": 0, "byteOffset": 12, "byteLength": 48 },
                { "buffer": 0, "byteOffset": 60, "byteLength": 48 },
            ],
            "buffers": [{ "byteLength": bin.len(), "uri": format!("data:application/octet-stream;base64,{b64}") }],
            "animations": [{
                "name": "wave",
                "samplers": [
                    { "input": 0, "output": 1, "interpolation": "LINEAR" },
                    { "input": 0, "output": 2, "interpolation": "LINEAR" },
                ],
                "channels": [
                    { "sampler": 0, "target": { "node": 1, "path": "rotation" } },
                    { "sampler": 1, "target": { "node": 2, "path": "rotation" } },
                ],
            }],
        });
        let model = import_gltf_from_slice(&serde_json::to_vec(&doc).unwrap(), None).unwrap();

        assert_eq!(model.sequences.len(), 1);
        let seq = &model.sequences[0];
        assert_eq!(seq.name, "wave");
        let names: Vec<_> = seq.steps.iter().map(|s| s.pose_name.as_str()).collect();
        assert_eq!(names, ["wave/0", "wave/1", "wave/2"]);
        let durations: Vec<f64> = seq.steps.iter().map(|s| s.duration).collect();
        assert_eq!(durations, [0.0, 0.5, 0.5]);

        let angle = |pose: &str, joint: &str| {
            model.poses.iter().find(|p| p.name == pose).unwrap().angles[joint] as f32
        };
        let deg = |d: f32| d * PI / 180.0;
        for (k, (e, w)) in [(0.0, 0.0), (45.0, 170.0), (90.0, 340.0)].into_iter().enumerate() {
            let pose = format!("wave/{k}");
            assert!((angle(&pose, "elbow") - deg(e)).abs() < 1e-4, "{pose} elbow");
            assert!((angle(&pose, "wheel_joint") - deg(w)).abs() < 1e-4, "{pose} wheel {}", angle(&pose, "wheel_joint"));
        }

        // Replaying the last pose reproduces the animated end state.
        let last = model.poses.iter().find(|p| p.name == "wave/2").unwrap();
        let mut posed = model.clone();
        posed.joint_positions = last.to_vector(&model, &model.joint_positions);
        let r = posed.compute_transforms()["arm"].rotation;
        assert!((r.angle() - PI / 2.0).abs() < 1e-4);
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
