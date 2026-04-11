use glow::HasContext;
use nalgebra as na;
use std::collections::HashMap;

use crate::camera::OrbitCamera;
use crate::primitives;
use crate::robot::{GeomData, RobotModel};

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

struct GlMeshEntry {
    vao: glow::VertexArray,
    vbo: glow::Buffer,
    num_vertices: i32,
    color: [f32; 4],
    link_name: String,
    visual_origin: na::Isometry3<f32>,
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
    axes_num_vertices: i32,
    transforms: HashMap<String, na::Isometry3<f32>>,
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

        for link in &model.links {
            for visual in &link.visuals {
                let vertex_data = match &visual.geometry {
                    GeomData::Box { hx, hy, hz } => primitives::generate_box(*hx, *hy, *hz),
                    GeomData::Cylinder {
                        radius,
                        half_length,
                    } => primitives::generate_cylinder(*radius, *half_length, 16),
                    GeomData::Sphere { radius } => primitives::generate_sphere(*radius, 16, 8),
                    GeomData::Mesh { vertices } => vertices.clone(),
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
                });
            }
        }

        // Compute initial transforms
        self.transforms = model.compute_transforms();
    }

    /// Update link transforms (call when joint positions change).
    pub fn update_transforms(&mut self, transforms: HashMap<String, na::Isometry3<f32>>) {
        self.transforms = transforms;
    }

    /// Render the scene.
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

            // Draw robot meshes
            gl.uniform_1_i32(Some(&self.u_flat), 0);
            for entry in &self.mesh_entries {
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

                gl.uniform_4_f32(
                    Some(&self.u_color),
                    entry.color[0],
                    entry.color[1],
                    entry.color[2],
                    entry.color[3],
                );

                gl.bind_vertex_array(Some(entry.vao));
                gl.draw_arrays(glow::TRIANGLES, 0, entry.num_vertices);
            }

            // Restore state
            gl.disable(glow::SCISSOR_TEST);
            gl.disable(glow::DEPTH_TEST);
            gl.bind_vertex_array(None);
            gl.use_program(None);
        }
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
