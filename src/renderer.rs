use glow::HasContext;
use nalgebra as na;
use std::collections::HashMap;

use articara::camera::OrbitCamera;
use articara::primitives;
use articara::robot::{GeomData, RobotModel};

// ========== Shaders ==========

const VERTEX_SHADER: &str = r#"#version 330 core
layout(location = 0) in vec3 a_position;
layout(location = 1) in vec3 a_normal;
uniform mat4 u_mvp;
uniform mat3 u_normal_mat;
out vec3 v_normal;
void main() {
    gl_Position = u_mvp * vec4(a_position, 1.0);
    v_normal = u_normal_mat * a_normal;
}
"#;

const FRAGMENT_SHADER: &str = r#"#version 330 core
in vec3 v_normal;
uniform vec4 u_color;
uniform vec3 u_light_dir;
uniform bool u_flat;
out vec4 frag_color;
void main() {
    if (u_flat) {
        frag_color = u_color;
    } else {
        vec3 n = normalize(v_normal);
        float diff = max(dot(n, u_light_dir), 0.0);
        float ambient = 0.3;
        float light = ambient + 0.7 * diff;
        frag_color = vec4(u_color.rgb * light, u_color.a);
    }
}
"#;

// ========== GL Mesh Entry ==========

/// Whether a mesh entry is from <visual> or <collision>.
#[derive(Clone, Copy, PartialEq, Eq, Hash)]
pub enum MeshKind {
    Visual,
    Collision,
}

/// Display mode for a geometry category.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum DisplayMode {
    Off,
    Solid,
    Wireframe,
    Transparent,
    FlatShading,
    Points,
}

impl DisplayMode {
    pub const ALL: [DisplayMode; 6] = [
        DisplayMode::Off,
        DisplayMode::Solid,
        DisplayMode::Wireframe,
        DisplayMode::Transparent,
        DisplayMode::FlatShading,
        DisplayMode::Points,
    ];

    pub fn label(self) -> &'static str {
        match self {
            DisplayMode::Off => "Off",
            DisplayMode::Solid => "Solid",
            DisplayMode::Wireframe => "Wireframe",
            DisplayMode::Transparent => "Transparent",
            DisplayMode::FlatShading => "Flat Shading",
            DisplayMode::Points => "Points",
        }
    }

    /// Cycle through the basic display modes (for viewport toggle):
    /// Off → Solid → Wireframe → Off.
    pub fn next(self) -> Self {
        match self {
            DisplayMode::Off => DisplayMode::Solid,
            DisplayMode::Solid => DisplayMode::Wireframe,
            // Any non-basic mode also cycles to Off
            _ => DisplayMode::Off,
        }
    }

    /// Cycle for collision viewport toggle:
    /// Off → Transparent → Wireframe → Off.
    pub fn next_collision(self) -> Self {
        match self {
            DisplayMode::Off => DisplayMode::Transparent,
            DisplayMode::Transparent => DisplayMode::Wireframe,
            _ => DisplayMode::Off,
        }
    }
}

struct GlMeshEntry {
    vao: glow::VertexArray,
    vbo: glow::Buffer,
    num_vertices: i32,
    color: [f32; 4],
    link_name: String,
    visual_origin: na::Isometry3<f32>,
    kind: MeshKind,
}

// ========== Renderer ==========

pub struct GlRenderer {
    program: glow::Program,
    u_mvp: glow::UniformLocation,
    u_normal_mat: glow::UniformLocation,
    u_color: glow::UniformLocation,
    u_light_dir: glow::UniformLocation,
    u_flat: glow::UniformLocation,
    mesh_entries: Vec<GlMeshEntry>,
    grid_vao: glow::VertexArray,
    grid_vbo: glow::Buffer,
    grid_num_vertices: i32,
    axes_vao: glow::VertexArray,
    axes_vbo: glow::Buffer,
    #[allow(dead_code)]
    axes_num_vertices: i32,
    transforms: HashMap<String, na::Isometry3<f32>>,
    pub highlight_link: Option<String>,
    /// Small sphere mesh for CoM markers.
    com_sphere_vao: glow::VertexArray,
    com_sphere_vbo: glow::Buffer,
    com_sphere_num_vertices: i32,
    /// CoM entries: (link_name, local_com_position, mass).
    com_entries: Vec<(String, na::Isometry3<f32>, f64)>,
    /// Whether to draw CoM markers.
    pub show_com: bool,
    /// Scale factor for CoM sphere size (radius = mass * com_scale).
    pub com_scale: f32,
    /// Wireframe mode for robot meshes.
    pub wireframe: bool,
    /// Global display mode for visual geometry.
    pub visual_mode: DisplayMode,
    /// Global display mode for collision geometry.
    pub collision_mode: DisplayMode,
    /// Per-link display mode overrides. Key=(link_name, MeshKind), value=DisplayMode.
    /// If absent for a (link, kind), use the corresponding global mode.
    pub link_display_modes: HashMap<(String, MeshKind), DisplayMode>,
    // --- Gizmo (offset adjustment) ---
    gizmo_arrow_vao: glow::VertexArray,
    gizmo_arrow_vbo: glow::Buffer,
    gizmo_arrow_num_vertices: i32,
    /// Transform at which to draw the gizmo. `None` = hidden.
    pub gizmo_transform: Option<na::Isometry3<f32>>,
    /// Which axis arrow is hovered (0=X, 1=Y, 2=Z). `None` = no hover.
    pub gizmo_hovered_axis: Option<u8>,
    /// Which axis arrow is being dragged (0=X, 1=Y, 2=Z). `None` = no drag.
    pub gizmo_dragged_axis: Option<u8>,
    // --- Gizmo ring (rotation) ---
    gizmo_ring_vao: glow::VertexArray,
    gizmo_ring_vbo: glow::Buffer,
    gizmo_ring_num_vertices: i32,
    // --- Gizmo scale handle ---
    gizmo_scale_vao: glow::VertexArray,
    gizmo_scale_vbo: glow::Buffer,
    gizmo_scale_num_vertices: i32,
    /// Current gizmo operation (0=translate arrows, 1=rotate rings, 2=scale handles).
    pub gizmo_op: u8, // 0=translate, 1=rotate, 2=scale
    // --- Joint axis arrows ---
    joint_arrow_vao: glow::VertexArray,
    joint_arrow_vbo: glow::Buffer,
    joint_arrow_num_vertices: i32,
    /// Whether to draw joint axis arrows.
    pub show_joint_axes: bool,
    /// Joint axis definitions: (parent_link, local_origin, local_axis, is_revolute).
    /// World positions are resolved at render time from `transforms`.
    joint_axis_entries: Vec<(String, na::Isometry3<f32>, na::Vector3<f32>, bool)>,
    // --- Ground plane (checkerboard) ---
    ground_light_vao: glow::VertexArray,
    ground_light_vbo: glow::Buffer,
    ground_light_num_vertices: i32,
    ground_dark_vao: glow::VertexArray,
    ground_dark_vbo: glow::Buffer,
    ground_dark_num_vertices: i32,
    /// Whether to show a semi-transparent ground plate.
    pub show_ground_plane: bool,
    /// Z height of the ground plane (default 0.0).
    pub ground_z: f32,
    /// Size (half-extent) of the ground plate.
    pub ground_size: f32,
    /// Ground plane rotation about X axis (rad).
    pub ground_plane_roll: f32,
    /// Ground plane rotation about Y axis (rad).
    pub ground_plane_pitch: f32,
    /// Whether to show the gravity/bias direction arrow.
    pub show_gravity_arrow: bool,
    /// Gravity (bias) direction (unit vector).
    pub gravity_dir: [f32; 3],
}

impl GlRenderer {
    pub fn new(gl: &glow::Context) -> Self {
        unsafe {
            let program = compile_program(gl, VERTEX_SHADER, FRAGMENT_SHADER);

            let u_mvp = gl.get_uniform_location(program, "u_mvp").unwrap();
            let u_normal_mat = gl.get_uniform_location(program, "u_normal_mat").unwrap();
            let u_color = gl.get_uniform_location(program, "u_color").unwrap();
            let u_light_dir = gl.get_uniform_location(program, "u_light_dir").unwrap();
            let u_flat = gl.get_uniform_location(program, "u_flat").unwrap();

            // Grid
            let grid_data = primitives::generate_grid(0.5, 20);
            let grid_num = (grid_data.len() / 6) as i32;
            let (grid_vao, grid_vbo) = upload_mesh_data(gl, &grid_data);

            // Axes
            let axes_data = primitives::generate_axes(0.15);
            let axes_num = (axes_data.len() / 6) as i32;
            let (axes_vao, axes_vbo) = upload_mesh_data(gl, &axes_data);

            // CoM sphere (unit sphere at origin, radius 1.0; scaled at draw time)
            let com_sphere_data = primitives::generate_sphere(1.0, 12, 6);
            let com_sphere_num = (com_sphere_data.len() / 6) as i32;
            let (com_sphere_vao, com_sphere_vbo) = upload_mesh_data(gl, &com_sphere_data);

            // Gizmo arrow (along +Z; rotated at draw time for each axis)
            let gizmo_arrow_data =
                primitives::generate_arrow(0.003, 0.06, 0.009, 0.02, 12);
            let gizmo_arrow_num = (gizmo_arrow_data.len() / 6) as i32;
            let (gizmo_arrow_vao, gizmo_arrow_vbo) = upload_mesh_data(gl, &gizmo_arrow_data);

            // Gizmo ring (around Z axis; rotated at draw time for each axis)
            let gizmo_ring_data =
                primitives::generate_ring(0.05, 0.003, 48, 8);
            let gizmo_ring_num = (gizmo_ring_data.len() / 6) as i32;
            let (gizmo_ring_vao, gizmo_ring_vbo) = upload_mesh_data(gl, &gizmo_ring_data);

            // Gizmo scale handle (shaft + cube along +Z; rotated for each axis)
            let gizmo_scale_data =
                primitives::generate_scale_handle(0.003, 0.06, 0.006, 12);
            let gizmo_scale_num = (gizmo_scale_data.len() / 6) as i32;
            let (gizmo_scale_vao, gizmo_scale_vbo) = upload_mesh_data(gl, &gizmo_scale_data);

            // Joint axis arrow (smaller than gizmo; shaft_r=0.002, shaft_l=0.04, head_r=0.007, head_l=0.015)
            let joint_arrow_data =
                primitives::generate_arrow(0.002, 0.04, 0.007, 0.015, 12);
            let joint_arrow_num = (joint_arrow_data.len() / 6) as i32;
            let (joint_arrow_vao, joint_arrow_vbo) = upload_mesh_data(gl, &joint_arrow_data);

            // Ground plane checkerboard tiles (two meshes: light/dark)
            let (ground_light_data, ground_dark_data) = generate_checkerboard_tiles();
            let ground_light_num = (ground_light_data.len() / 6) as i32;
            let ground_dark_num = (ground_dark_data.len() / 6) as i32;
            let (ground_light_vao, ground_light_vbo) = upload_mesh_data(gl, &ground_light_data);
            let (ground_dark_vao, ground_dark_vbo) = upload_mesh_data(gl, &ground_dark_data);

            Self {
                program,
                u_mvp,
                u_normal_mat,
                u_color,
                u_light_dir,
                u_flat,
                mesh_entries: Vec::new(),
                grid_vao,
                grid_vbo,
                grid_num_vertices: grid_num,
                axes_vao,
                axes_vbo,
                axes_num_vertices: axes_num,
                transforms: HashMap::new(),
                highlight_link: None,
                com_sphere_vao,
                com_sphere_vbo,
                com_sphere_num_vertices: com_sphere_num,
                com_entries: Vec::new(),
                show_com: false,
                com_scale: 0.01,
                wireframe: false,
                visual_mode: DisplayMode::Solid,
                collision_mode: DisplayMode::Off,
                link_display_modes: HashMap::new(),
                gizmo_arrow_vao,
                gizmo_arrow_vbo,
                gizmo_arrow_num_vertices: gizmo_arrow_num,
                gizmo_transform: None,
                gizmo_hovered_axis: None,
                gizmo_dragged_axis: None,
                gizmo_ring_vao,
                gizmo_ring_vbo,
                gizmo_ring_num_vertices: gizmo_ring_num,
                gizmo_scale_vao,
                gizmo_scale_vbo,
                gizmo_scale_num_vertices: gizmo_scale_num,
                gizmo_op: 0,
                joint_arrow_vao,
                joint_arrow_vbo,
                joint_arrow_num_vertices: joint_arrow_num,
                show_joint_axes: false,
                joint_axis_entries: Vec::new(),
                ground_light_vao,
                ground_light_vbo,
                ground_light_num_vertices: ground_light_num,
                ground_dark_vao,
                ground_dark_vbo,
                ground_dark_num_vertices: ground_dark_num,
                show_ground_plane: false,
                ground_z: 0.0,
                ground_size: 2.0,
                ground_plane_roll: 0.0,
                ground_plane_pitch: 0.0,
                show_gravity_arrow: true,
                gravity_dir: [0.0, 0.0, -1.0],
            }
        }
    }

    /// Upload all visual meshes for the robot model.
    pub fn upload_robot(&mut self, gl: &glow::Context, model: &RobotModel) {
        // Clear old mesh entries
        unsafe {
            for entry in self.mesh_entries.drain(..) {
                gl.delete_vertex_array(entry.vao);
                gl.delete_buffer(entry.vbo);
            }
        }

        // Build CoM entries from link inertial data
        self.com_entries.clear();
        for link in &model.links {
            if link.inertial.mass > 1e-12 {
                self.com_entries.push((
                    link.name.clone(),
                    link.inertial.origin,
                    link.inertial.mass,
                ));
            }
        }

        for link in &model.links {
            for visual in &link.visuals {
                let vertex_data = match &visual.geometry {
                    GeomData::Box { hx, hy, hz } => primitives::generate_box(*hx, *hy, *hz),
                    GeomData::Cylinder {
                        radius,
                        half_length,
                    } => primitives::generate_cylinder(*radius, *half_length, 16),
                    GeomData::Sphere { radius } => primitives::generate_sphere(*radius, 16, 8),
                    GeomData::Capsule { radius, half_length } => primitives::generate_capsule(*radius, *half_length, 16, 8),
                    GeomData::Mesh { vertices, .. } => vertices.clone(),
                };

                if vertex_data.is_empty() {
                    continue;
                }

                let num_vertices = (vertex_data.len() / 6) as i32;
                let (vao, vbo) = unsafe { upload_mesh_data(gl, &vertex_data) };

                self.mesh_entries.push(GlMeshEntry {
                    vao,
                    vbo,
                    num_vertices,
                    color: visual.color,
                    link_name: link.name.clone(),
                    visual_origin: visual.origin,
                    kind: MeshKind::Visual,
                });
            }

            // Upload collision meshes
            for col in &link.collisions {
                let vertex_data = match &col.geometry {
                    GeomData::Box { hx, hy, hz } => primitives::generate_box(*hx, *hy, *hz),
                    GeomData::Cylinder { radius, half_length } => {
                        primitives::generate_cylinder(*radius, *half_length, 16)
                    }
                    GeomData::Sphere { radius } => primitives::generate_sphere(*radius, 16, 8),
                    GeomData::Capsule { radius, half_length } => primitives::generate_capsule(*radius, *half_length, 16, 8),
                    GeomData::Mesh { vertices, .. } => vertices.clone(),
                };
                if vertex_data.is_empty() {
                    continue;
                }
                let num_vertices = (vertex_data.len() / 6) as i32;
                let (vao, vbo) = unsafe { upload_mesh_data(gl, &vertex_data) };
                self.mesh_entries.push(GlMeshEntry {
                    vao,
                    vbo,
                    num_vertices,
                    color: [0.0, 1.0, 0.5, 0.4], // green-ish for collision
                    link_name: link.name.clone(),
                    visual_origin: col.origin,
                    kind: MeshKind::Collision,
                });
            }
        }

        // Build joint axis entries
        self.joint_axis_entries.clear();
        for joint in &model.joints {
            if joint.joint_type == "fixed" {
                continue;
            }
            let is_revolute = joint.joint_type == "revolute" || joint.joint_type == "continuous";
            self.joint_axis_entries.push((
                joint.parent_link.clone(),
                joint.origin,
                joint.axis,
                is_revolute,
            ));
        }

        // Compute initial transforms
        self.transforms = model.compute_transforms();
    }

    /// Update link transforms (call when joint positions change).
    pub fn update_transforms(&mut self, transforms: HashMap<String, na::Isometry3<f32>>) {
        self.transforms = transforms;
    }

    /// Render the scene.
    /// Draw a single mesh entry with transform, lighting, and highlight.
    unsafe fn draw_mesh_entry(
        &self,
        gl: &glow::Context,
        entry: &GlMeshEntry,
        vp: &na::Matrix4<f32>,
        _light_dir: &na::Vector3<f32>,
    ) {
        unsafe {
            let world_tf = self
                .transforms
                .get(&entry.link_name)
                .copied()
                .unwrap_or(na::Isometry3::identity());
            let model_mat = (world_tf * entry.visual_origin).to_homogeneous();
            let mvp = vp * model_mat;

            gl.uniform_matrix_4_f32_slice(Some(&self.u_mvp), false, mvp.as_slice());

            let model3 = model_mat.fixed_view::<3, 3>(0, 0).into_owned();
            let normal_mat = model3
                .try_inverse()
                .map(|inv| inv.transpose())
                .unwrap_or(na::Matrix3::identity());
            gl.uniform_matrix_3_f32_slice(
                Some(&self.u_normal_mat),
                false,
                normal_mat.as_slice(),
            );

            // Highlight hovered/dragged link with a bright tint
            let is_highlighted = self
                .highlight_link
                .as_ref()
                .map(|h| h == &entry.link_name)
                .unwrap_or(false);
            if is_highlighted {
                let tint = 0.4;
                gl.uniform_4_f32(
                    Some(&self.u_color),
                    (entry.color[0] + tint).min(1.0),
                    (entry.color[1] + tint).min(1.0),
                    (entry.color[2] + tint).min(1.0),
                    entry.color[3],
                );
            } else {
                gl.uniform_4_f32(
                    Some(&self.u_color),
                    entry.color[0],
                    entry.color[1],
                    entry.color[2],
                    entry.color[3],
                );
            }

            gl.bind_vertex_array(Some(entry.vao));
            gl.draw_arrays(glow::TRIANGLES, 0, entry.num_vertices);
        }
    }

    /// Draw a mesh entry in transparent mode (same as draw_mesh_entry but with reduced alpha).
    unsafe fn draw_mesh_entry_transparent(
        &self,
        gl: &glow::Context,
        entry: &GlMeshEntry,
        vp: &na::Matrix4<f32>,
        _light_dir: &na::Vector3<f32>,
    ) {
        unsafe {
            let world_tf = self
                .transforms
                .get(&entry.link_name)
                .copied()
                .unwrap_or(na::Isometry3::identity());
            let model_mat = (world_tf * entry.visual_origin).to_homogeneous();
            let mvp = vp * model_mat;

            gl.uniform_matrix_4_f32_slice(Some(&self.u_mvp), false, mvp.as_slice());

            let model3 = model_mat.fixed_view::<3, 3>(0, 0).into_owned();
            let normal_mat = model3
                .try_inverse()
                .map(|inv| inv.transpose())
                .unwrap_or(na::Matrix3::identity());
            gl.uniform_matrix_3_f32_slice(
                Some(&self.u_normal_mat),
                false,
                normal_mat.as_slice(),
            );

            let alpha = 0.4_f32;
            let is_highlighted = self
                .highlight_link
                .as_ref()
                .map(|h| h == &entry.link_name)
                .unwrap_or(false);
            if is_highlighted {
                let tint = 0.4;
                gl.uniform_4_f32(
                    Some(&self.u_color),
                    (entry.color[0] + tint).min(1.0),
                    (entry.color[1] + tint).min(1.0),
                    (entry.color[2] + tint).min(1.0),
                    alpha,
                );
            } else {
                gl.uniform_4_f32(
                    Some(&self.u_color),
                    entry.color[0],
                    entry.color[1],
                    entry.color[2],
                    alpha,
                );
            }

            gl.bind_vertex_array(Some(entry.vao));
            gl.draw_arrays(glow::TRIANGLES, 0, entry.num_vertices);
        }
    }

    pub fn render(&self, gl: &glow::Context, camera: &OrbitCamera, viewport: [i32; 4]) {
        let w = viewport[2].max(1);
        let h = viewport[3].max(1);
        let aspect = w as f32 / h as f32;
        let view = camera.view_matrix();
        let proj = camera.projection_matrix(aspect);
        let vp = proj * view;
        let light_dir = na::Vector3::new(0.3, 0.5, 0.8).normalize();

        unsafe {
            gl.viewport(viewport[0], viewport[1], w, h);
            gl.enable(glow::DEPTH_TEST);
            gl.depth_func(glow::LESS);
            gl.depth_mask(true);
            gl.enable(glow::SCISSOR_TEST);
            gl.scissor(viewport[0], viewport[1], w, h);
            gl.clear_color(0.15, 0.15, 0.20, 1.0);
            gl.clear(glow::COLOR_BUFFER_BIT | glow::DEPTH_BUFFER_BIT);

            gl.use_program(Some(self.program));
            gl.uniform_3_f32(
                Some(&self.u_light_dir),
                light_dir.x,
                light_dir.y,
                light_dir.z,
            );

            // Draw grid
            gl.uniform_matrix_4_f32_slice(Some(&self.u_mvp), false, vp.as_slice());
            gl.uniform_matrix_3_f32_slice(
                Some(&self.u_normal_mat),
                false,
                na::Matrix3::<f32>::identity().as_slice(),
            );
            gl.uniform_4_f32(Some(&self.u_color), 0.3, 0.3, 0.3, 1.0);
            gl.uniform_1_i32(Some(&self.u_flat), 1);
            gl.bind_vertex_array(Some(self.grid_vao));
            gl.draw_arrays(glow::LINES, 0, self.grid_num_vertices);

            // Draw axes
            gl.line_width(2.0);
            // X axis - red
            gl.uniform_4_f32(Some(&self.u_color), 1.0, 0.2, 0.2, 1.0);
            gl.bind_vertex_array(Some(self.axes_vao));
            gl.draw_arrays(glow::LINES, 0, 2);
            // Y axis - green
            gl.uniform_4_f32(Some(&self.u_color), 0.2, 1.0, 0.2, 1.0);
            gl.draw_arrays(glow::LINES, 2, 2);
            // Z axis - blue
            gl.uniform_4_f32(Some(&self.u_color), 0.2, 0.2, 1.0, 1.0);
            gl.draw_arrays(glow::LINES, 4, 2);
            gl.line_width(1.0);

            // Draw ground plane (checkerboard)
            if self.show_ground_plane {
                gl.enable(glow::BLEND);
                gl.blend_func(glow::SRC_ALPHA, glow::ONE_MINUS_SRC_ALPHA);
                gl.depth_mask(false); // don't write depth for transparent surface

                let translate = na::Matrix4::new_translation(&na::Vector3::new(0.0, 0.0, self.ground_z));
                // Roll (X-axis) then pitch (Y-axis) rotation
                let rot_x = na::Rotation3::from_axis_angle(
                    &na::Vector3::x_axis(),
                    self.ground_plane_roll,
                );
                let rot_y = na::Rotation3::from_axis_angle(
                    &na::Vector3::y_axis(),
                    self.ground_plane_pitch,
                );
                let rot = (rot_y * rot_x).to_homogeneous();
                // Tile size is baked as 1.0 in the mesh; scale by ground_size
                // to map [0..N tiles] to the desired world extent.
                let tile_scale = self.ground_size / CHECKER_HALF_TILES as f32;
                let scale = na::Matrix4::new_nonuniform_scaling(&na::Vector3::new(tile_scale, tile_scale, 1.0));
                let mvp = vp * translate * rot * scale;
                gl.uniform_matrix_4_f32_slice(Some(&self.u_mvp), false, mvp.as_slice());

                // Compute rotated normal for proper lighting
                let normal3 = (rot_y * rot_x) * na::Vector3::new(0.0, 0.0, 1.0);
                let normal_mat = na::Matrix3::from_columns(&[
                    na::Vector3::x(),
                    na::Vector3::y(),
                    normal3,
                ]);
                gl.uniform_matrix_3_f32_slice(
                    Some(&self.u_normal_mat),
                    false,
                    normal_mat.as_slice(),
                );
                gl.uniform_1_i32(Some(&self.u_flat), 0);

                // Light tiles
                gl.uniform_4_f32(Some(&self.u_color), 0.45, 0.47, 0.50, 0.60);
                gl.bind_vertex_array(Some(self.ground_light_vao));
                gl.draw_arrays(glow::TRIANGLES, 0, self.ground_light_num_vertices);

                // Dark tiles
                gl.uniform_4_f32(Some(&self.u_color), 0.30, 0.32, 0.35, 0.60);
                gl.bind_vertex_array(Some(self.ground_dark_vao));
                gl.draw_arrays(glow::TRIANGLES, 0, self.ground_dark_num_vertices);

                gl.depth_mask(true);
                gl.disable(glow::BLEND);
            }

            // Draw robot meshes — iterate all entries, resolve per-link display mode
            gl.uniform_1_i32(Some(&self.u_flat), 0);
            for entry in &self.mesh_entries {
                // Resolve effective display mode: per-link override > global
                let global_mode = match entry.kind {
                    MeshKind::Visual => self.visual_mode,
                    MeshKind::Collision => self.collision_mode,
                };
                let mode = self
                    .link_display_modes
                    .get(&(entry.link_name.clone(), entry.kind))
                    .copied()
                    .unwrap_or(global_mode);

                match mode {
                    DisplayMode::Off => continue,
                    DisplayMode::Solid => {
                        gl.polygon_mode(glow::FRONT_AND_BACK, glow::FILL);
                        gl.uniform_1_i32(Some(&self.u_flat), 0);
                        gl.disable(glow::BLEND);
                        gl.depth_mask(true);
                    }
                    DisplayMode::Wireframe => {
                        gl.polygon_mode(glow::FRONT_AND_BACK, glow::LINE);
                        gl.uniform_1_i32(Some(&self.u_flat), 0);
                        gl.disable(glow::BLEND);
                        gl.depth_mask(true);
                    }
                    DisplayMode::Transparent => {
                        gl.polygon_mode(glow::FRONT_AND_BACK, glow::FILL);
                        gl.uniform_1_i32(Some(&self.u_flat), 0);
                        gl.enable(glow::BLEND);
                        gl.blend_func(glow::SRC_ALPHA, glow::ONE_MINUS_SRC_ALPHA);
                        gl.depth_mask(false);
                    }
                    DisplayMode::FlatShading => {
                        gl.polygon_mode(glow::FRONT_AND_BACK, glow::FILL);
                        gl.uniform_1_i32(Some(&self.u_flat), 1);
                        gl.disable(glow::BLEND);
                        gl.depth_mask(true);
                    }
                    DisplayMode::Points => {
                        gl.polygon_mode(glow::FRONT_AND_BACK, glow::POINT);
                        gl.uniform_1_i32(Some(&self.u_flat), 1);
                        gl.disable(glow::BLEND);
                        gl.depth_mask(true);
                    }
                }
                // For Transparent mode, reduce alpha of the entry color
                if mode == DisplayMode::Transparent {
                    // Temporarily override color alpha — draw_mesh_entry sets u_color,
                    // so we handle alpha inside a wrapper.
                    self.draw_mesh_entry_transparent(gl, entry, &vp, &light_dir);
                } else {
                    self.draw_mesh_entry(gl, entry, &vp, &light_dir);
                }
            }
            // Ensure fill mode and state are restored
            gl.polygon_mode(glow::FRONT_AND_BACK, glow::FILL);
            gl.uniform_1_i32(Some(&self.u_flat), 0);
            gl.disable(glow::BLEND);
            gl.depth_mask(true);


            // Draw CoM markers
            if self.show_com {
                gl.uniform_1_i32(Some(&self.u_flat), 0);
                gl.bind_vertex_array(Some(self.com_sphere_vao));
                for (link_name, com_origin, mass) in &self.com_entries {
                    let world_tf = self
                        .transforms
                        .get(link_name)
                        .copied()
                        .unwrap_or(na::Isometry3::identity());
                    let com_world = (world_tf * *com_origin).to_homogeneous();
                    // Scale sphere by mass: radius = mass * com_scale
                    let r = (*mass as f32 * self.com_scale).max(0.002);
                    let scale = na::Matrix4::new_nonuniform_scaling(&na::Vector3::new(r, r, r));
                    let mvp = vp * com_world * scale;

                    gl.uniform_matrix_4_f32_slice(Some(&self.u_mvp), false, mvp.as_slice());
                    gl.uniform_matrix_3_f32_slice(
                        Some(&self.u_normal_mat),
                        false,
                        na::Matrix3::<f32>::identity().as_slice(),
                    );
                    // Bright magenta color for CoM spheres
                    gl.uniform_4_f32(Some(&self.u_color), 1.0, 0.0, 0.8, 1.0);
                    gl.draw_arrays(glow::TRIANGLES, 0, self.com_sphere_num_vertices);
                }
            }

            // Draw joint axis arrows
            if self.show_joint_axes && !self.joint_axis_entries.is_empty() {
                gl.uniform_1_i32(Some(&self.u_flat), 0);
                gl.bind_vertex_array(Some(self.joint_arrow_vao));

                for (parent_link, local_origin, local_axis, is_revolute) in &self.joint_axis_entries {
                    let parent_tf = self
                        .transforms
                        .get(parent_link)
                        .copied()
                        .unwrap_or(na::Isometry3::identity());
                    let joint_world = parent_tf * *local_origin;
                    let world_axis = (joint_world.rotation * local_axis).normalize();

                    // Build a rotation that aligns +Z with the joint axis
                    let axis_rot = na::UnitQuaternion::rotation_between(
                        &na::Vector3::z(),
                        &world_axis,
                    )
                    .unwrap_or(na::UnitQuaternion::identity());

                    // Position arrow so it is centered on the joint:
                    // shift back by half the arrow length (0.04+0.015=0.055 total, half≈0.0275)
                    let arrow_center_offset = world_axis * (-0.0275);
                    let arrow_pos = joint_world.translation.vector + arrow_center_offset;
                    let arrow_tf = na::Isometry3::from_parts(
                        na::Translation3::from(arrow_pos),
                        axis_rot,
                    );

                    let model_mat = arrow_tf.to_homogeneous();
                    let mvp = vp * model_mat;
                    gl.uniform_matrix_4_f32_slice(
                        Some(&self.u_mvp),
                        false,
                        mvp.as_slice(),
                    );
                    let model3 = model_mat.fixed_view::<3, 3>(0, 0).into_owned();
                    let normal_mat = model3
                        .try_inverse()
                        .map(|inv| inv.transpose())
                        .unwrap_or(na::Matrix3::identity());
                    gl.uniform_matrix_3_f32_slice(
                        Some(&self.u_normal_mat),
                        false,
                        normal_mat.as_slice(),
                    );

                    // Revolute = orange, Prismatic = cyan
                    if *is_revolute {
                        gl.uniform_4_f32(Some(&self.u_color), 1.0, 0.6, 0.0, 1.0);
                    } else {
                        gl.uniform_4_f32(Some(&self.u_color), 0.0, 0.8, 1.0, 1.0);
                    }

                    gl.draw_arrays(glow::TRIANGLES, 0, self.joint_arrow_num_vertices);
                }
            }

            // Draw gizmo (offset adjustment mode): arrows or rings
            if let Some(gizmo_tf) = self.gizmo_transform {
                // Draw on top of everything
                gl.clear(glow::DEPTH_BUFFER_BIT);
                gl.uniform_1_i32(Some(&self.u_flat), 0);

                let is_rotate = self.gizmo_op == 1;
                let is_scale = self.gizmo_op == 2;
                let gizmo_vao = if is_rotate {
                    self.gizmo_ring_vao
                } else if is_scale {
                    self.gizmo_scale_vao
                } else {
                    self.gizmo_arrow_vao
                };
                let gizmo_num = if is_rotate {
                    self.gizmo_ring_num_vertices
                } else if is_scale {
                    self.gizmo_scale_num_vertices
                } else {
                    self.gizmo_arrow_num_vertices
                };
                gl.bind_vertex_array(Some(gizmo_vao));

                // Axis colors: X=red, Y=green, Z=blue
                let axis_colors: [[f32; 4]; 3] = [
                    [1.0, 0.2, 0.2, 1.0],
                    [0.2, 1.0, 0.2, 1.0],
                    [0.2, 0.4, 1.0, 1.0],
                ];
                // Rotations to orient +Z arrow to each axis
                let axis_rotations: [na::UnitQuaternion<f32>; 3] = [
                    // +Z → +X : rotate +90° around Y
                    na::UnitQuaternion::from_axis_angle(
                        &na::Vector3::y_axis(),
                        std::f32::consts::FRAC_PI_2,
                    ),
                    // +Z → +Y : rotate −90° around X
                    na::UnitQuaternion::from_axis_angle(
                        &na::Vector3::x_axis(),
                        -std::f32::consts::FRAC_PI_2,
                    ),
                    // +Z stays +Z
                    na::UnitQuaternion::identity(),
                ];

                for (i, (color, rot)) in
                    axis_colors.iter().zip(axis_rotations.iter()).enumerate()
                {
                    let axis_tf = gizmo_tf
                        * na::Isometry3::from_parts(na::Translation3::identity(), *rot);
                    let model_mat = axis_tf.to_homogeneous();
                    let mvp = vp * model_mat;
                    gl.uniform_matrix_4_f32_slice(
                        Some(&self.u_mvp),
                        false,
                        mvp.as_slice(),
                    );

                    let model3 = model_mat.fixed_view::<3, 3>(0, 0).into_owned();
                    let normal_mat = model3
                        .try_inverse()
                        .map(|inv| inv.transpose())
                        .unwrap_or(na::Matrix3::identity());
                    gl.uniform_matrix_3_f32_slice(
                        Some(&self.u_normal_mat),
                        false,
                        normal_mat.as_slice(),
                    );

                    let iu8 = i as u8;
                    let is_active = self.gizmo_dragged_axis == Some(iu8)
                        || (self.gizmo_dragged_axis.is_none()
                            && self.gizmo_hovered_axis == Some(iu8));
                    if is_active {
                        // Bright yellow highlight
                        gl.uniform_4_f32(Some(&self.u_color), 1.0, 1.0, 0.2, 1.0);
                    } else {
                        gl.uniform_4_f32(
                            Some(&self.u_color),
                            color[0],
                            color[1],
                            color[2],
                            color[3],
                        );
                    }

                    gl.draw_arrays(glow::TRIANGLES, 0, gizmo_num);
                }
            }

            // Restore state
            gl.disable(glow::SCISSOR_TEST);
            gl.disable(glow::DEPTH_TEST);
            gl.bind_vertex_array(None);
            gl.use_program(None);
        }
    }

    /// Get world-space CoM positions and their masses (for drawing labels).
    pub fn com_world_positions(&self) -> Vec<(na::Point3<f32>, f64)> {
        if !self.show_com {
            return Vec::new();
        }
        self.com_entries
            .iter()
            .map(|(link_name, com_origin, mass)| {
                let world_tf = self
                    .transforms
                    .get(link_name)
                    .copied()
                    .unwrap_or(na::Isometry3::identity());
                let com_world = world_tf * com_origin * na::Point3::origin();
                (com_world, *mass)
            })
            .collect()
    }

    /// Clean up GL resources.
    pub fn destroy(&self, gl: &glow::Context) {
        unsafe {
            for entry in &self.mesh_entries {
                gl.delete_vertex_array(entry.vao);
                gl.delete_buffer(entry.vbo);
            }
            gl.delete_vertex_array(self.grid_vao);
            gl.delete_buffer(self.grid_vbo);
            gl.delete_vertex_array(self.axes_vao);
            gl.delete_buffer(self.axes_vbo);
            gl.delete_vertex_array(self.com_sphere_vao);
            gl.delete_buffer(self.com_sphere_vbo);
            gl.delete_vertex_array(self.gizmo_arrow_vao);
            gl.delete_buffer(self.gizmo_arrow_vbo);
            gl.delete_vertex_array(self.gizmo_ring_vao);
            gl.delete_buffer(self.gizmo_ring_vbo);
            gl.delete_vertex_array(self.gizmo_scale_vao);
            gl.delete_buffer(self.gizmo_scale_vbo);
            gl.delete_vertex_array(self.joint_arrow_vao);
            gl.delete_buffer(self.joint_arrow_vbo);
            gl.delete_vertex_array(self.ground_light_vao);
            gl.delete_buffer(self.ground_light_vbo);
            gl.delete_vertex_array(self.ground_dark_vao);
            gl.delete_buffer(self.ground_dark_vbo);
            gl.delete_program(self.program);
        }
    }
}

// ========== GL Helpers ==========

unsafe fn compile_program(gl: &glow::Context, vs_src: &str, fs_src: &str) -> glow::Program {
    unsafe {
        let vs = gl.create_shader(glow::VERTEX_SHADER).unwrap();
        gl.shader_source(vs, vs_src);
        gl.compile_shader(vs);
        if !gl.get_shader_compile_status(vs) {
            panic!("Vertex shader error: {}", gl.get_shader_info_log(vs));
        }

        let fs = gl.create_shader(glow::FRAGMENT_SHADER).unwrap();
        gl.shader_source(fs, fs_src);
        gl.compile_shader(fs);
        if !gl.get_shader_compile_status(fs) {
            panic!("Fragment shader error: {}", gl.get_shader_info_log(fs));
        }

        let program = gl.create_program().unwrap();
        gl.attach_shader(program, vs);
        gl.attach_shader(program, fs);
        gl.link_program(program);
        if !gl.get_program_link_status(program) {
            panic!("Program link error: {}", gl.get_program_info_log(program));
        }

        gl.delete_shader(vs);
        gl.delete_shader(fs);
        program
    }
}

unsafe fn upload_mesh_data(gl: &glow::Context, data: &[f32]) -> (glow::VertexArray, glow::Buffer) {
    unsafe {
        let vao = gl.create_vertex_array().unwrap();
        gl.bind_vertex_array(Some(vao));

        let vbo = gl.create_buffer().unwrap();
        gl.bind_buffer(glow::ARRAY_BUFFER, Some(vbo));

        let bytes: &[u8] =
            std::slice::from_raw_parts(data.as_ptr() as *const u8, data.len() * size_of::<f32>());
        gl.buffer_data_u8_slice(glow::ARRAY_BUFFER, bytes, glow::STATIC_DRAW);

        let stride = 6 * size_of::<f32>() as i32;
        // Position: location 0
        gl.enable_vertex_attrib_array(0);
        gl.vertex_attrib_pointer_f32(0, 3, glow::FLOAT, false, stride, 0);
        // Normal: location 1
        gl.enable_vertex_attrib_array(1);
        gl.vertex_attrib_pointer_f32(1, 3, glow::FLOAT, false, stride, 3 * size_of::<f32>() as i32);

        gl.bind_vertex_array(None);
        gl.bind_buffer(glow::ARRAY_BUFFER, None);

        (vao, vbo)
    }
}

/// Number of tiles per half-axis for the checkerboard ground plane.
/// Total grid is (2*N)×(2*N) tiles, centred at the origin.
const CHECKER_HALF_TILES: i32 = 10;

/// Generate two meshes (light tiles, dark tiles) for a checkerboard ground plane.
///
/// Each tile is a 1×1 quad in XY at Z=0.  The grid spans
/// `[-N .. N] × [-N .. N]` where `N = CHECKER_HALF_TILES`.
/// Returns `(light_vertices, dark_vertices)` with 6 floats per vertex
/// (pos + normal).
fn generate_checkerboard_tiles() -> (Vec<f32>, Vec<f32>) {
    let n = CHECKER_HALF_TILES;
    let total = (2 * n) * (2 * n); // total tiles
    let cap = (total as usize / 2 + 1) * 6 * 6; // 6 vertices × 6 floats
    let mut light = Vec::with_capacity(cap);
    let mut dark = Vec::with_capacity(cap);
    let nrm = [0.0_f32, 0.0, 1.0];

    for iy in -n..n {
        for ix in -n..n {
            let x0 = ix as f32;
            let y0 = iy as f32;
            let x1 = x0 + 1.0;
            let y1 = y0 + 1.0;
            let is_light = (ix + iy) & 1 == 0;
            let buf = if is_light { &mut light } else { &mut dark };
            // Triangle 1
            buf.extend_from_slice(&[x0, y0, 0.0]); buf.extend_from_slice(&nrm);
            buf.extend_from_slice(&[x1, y0, 0.0]); buf.extend_from_slice(&nrm);
            buf.extend_from_slice(&[x1, y1, 0.0]); buf.extend_from_slice(&nrm);
            // Triangle 2
            buf.extend_from_slice(&[x0, y0, 0.0]); buf.extend_from_slice(&nrm);
            buf.extend_from_slice(&[x1, y1, 0.0]); buf.extend_from_slice(&nrm);
            buf.extend_from_slice(&[x0, y1, 0.0]); buf.extend_from_slice(&nrm);
        }
    }
    (light, dark)
}
