//! Model-oriented Rhai scripting engine for articara.
//!
//! While [`super::ScriptEngine`] targets real-time control loops
//! (sensor→torque), this module exposes the full **model manipulation**
//! API — loading files, reading/writing joint positions, computing FK,
//! running IK, inspecting links, and exporting to various formats.
//!
//! # Quick start
//!
//! ```rhai
//! // Load a URDF and inspect it
//! load("robot.urdf");
//! print("Links: " + link_names().len());
//!
//! // Set joints and get FK
//! set_joint("shoulder", 0.5);
//! let p = link_pos("hand");
//! print("hand position: " + p);
//!
//! // Run IK
//! ik("hand", 0.3, 0.0, 0.5);
//! print("After IK: " + joint_pos("shoulder"));
//!
//! // Export
//! export_urdf("/tmp/out.urdf");
//! ```

use rhai::{Dynamic, Engine, AST, Scope, Array};
use std::cell::RefCell;
use std::rc::Rc;

use crate::robot::RobotModel;
use crate::rbd::model::{IkSolver, LoopClosure};

// ─────────────────────────────────────────────────────────────────────────
//  ModelScriptEngine
// ─────────────────────────────────────────────────────────────────────────

/// A Rhai scripting engine with full RobotModel manipulation bindings.
pub struct ModelScriptEngine {
    engine: Engine,
    ast: Option<AST>,
    scope: Scope<'static>,
    model: Rc<RefCell<Option<RobotModel>>>,
    /// Captured print output lines.
    output_lines: Rc<RefCell<Vec<String>>>,
    last_error: Option<String>,
}

impl ModelScriptEngine {
    /// Create a new model scripting engine.
    pub fn new() -> Self {
        let model: Rc<RefCell<Option<RobotModel>>> = Rc::new(RefCell::new(None));
        let output_lines: Rc<RefCell<Vec<String>>> = Rc::new(RefCell::new(Vec::new()));
        let engine = Self::build_engine(Rc::clone(&model), Rc::clone(&output_lines));
        Self {
            engine,
            ast: None,
            scope: Scope::new(),
            model,
            output_lines,
            last_error: None,
        }
    }

    /// Create with a pre-loaded model.
    pub fn with_model(robot: RobotModel) -> Self {
        let mut eng = Self::new();
        *eng.model.borrow_mut() = Some(robot);
        eng
    }

    /// Compile a script string.
    pub fn compile(&mut self, source: &str) -> Result<(), String> {
        match self.engine.compile(source) {
            Ok(ast) => {
                self.ast = Some(ast);
                self.last_error = None;
                Ok(())
            }
            Err(e) => {
                let msg = format!("Compile error: {e}");
                self.last_error = Some(msg.clone());
                Err(msg)
            }
        }
    }

    /// Run the compiled script.  Returns captured print output.
    pub fn run(&mut self) -> Result<Vec<String>, String> {
        let ast = self.ast.as_ref().ok_or("No script compiled")?;
        self.output_lines.borrow_mut().clear();

        match self.engine.eval_ast_with_scope::<Dynamic>(&mut self.scope, ast) {
            Ok(_) => {
                self.last_error = None;
                Ok(self.output_lines.borrow().clone())
            }
            Err(e) => {
                let msg = format!("Runtime error: {e}");
                self.last_error = Some(msg.clone());
                Err(msg)
            }
        }
    }

    /// Compile and run in one call.
    pub fn eval(&mut self, source: &str) -> Result<Vec<String>, String> {
        self.compile(source)?;
        self.run()
    }

    /// Returns the last error, if any.
    pub fn last_error(&self) -> Option<&str> {
        self.last_error.as_deref()
    }

    /// Access the currently loaded model (if any).
    pub fn model(&self) -> Option<std::cell::Ref<'_, RobotModel>> {
        let borrow = self.model.borrow();
        if borrow.is_some() {
            Some(std::cell::Ref::map(borrow, |o| o.as_ref().unwrap()))
        } else {
            None
        }
    }

    /// Set the model from outside.
    pub fn set_model(&mut self, robot: RobotModel) {
        *self.model.borrow_mut() = Some(robot);
    }

    /// Clear scope (but keep model).
    pub fn reset_scope(&mut self) {
        self.scope.clear();
    }

    // ── Engine builder ──────────────────────────────────────────────────

    fn build_engine(
        model: Rc<RefCell<Option<RobotModel>>>,
        output: Rc<RefCell<Vec<String>>>,
    ) -> Engine {
        let mut engine = Engine::new();

        // Safety limits
        engine.set_max_operations(500_000);
        engine.set_max_call_levels(32);
        engine.set_max_expr_depths(64, 64);

        // ── print → captured output ──
        let out = Rc::clone(&output);
        engine.on_print(move |s| {
            out.borrow_mut().push(s.to_string());
        });

        // ── Math ──
        engine.register_fn("abs", |x: f64| -> f64 { x.abs() });
        engine.register_fn("sqrt", |x: f64| -> f64 { x.sqrt() });
        engine.register_fn("sin", |x: f64| -> f64 { x.sin() });
        engine.register_fn("cos", |x: f64| -> f64 { x.cos() });
        engine.register_fn("atan2", |y: f64, x: f64| -> f64 { y.atan2(x) });
        engine.register_fn("min_f", |a: f64, b: f64| -> f64 { a.min(b) });
        engine.register_fn("max_f", |a: f64, b: f64| -> f64 { a.max(b) });
        engine.register_fn("clamp", |x: f64, lo: f64, hi: f64| -> f64 {
            x.clamp(lo, hi)
        });
        engine.register_fn("to_deg", |x: f64| -> f64 { x.to_degrees() });
        engine.register_fn("to_rad", |x: f64| -> f64 { x.to_radians() });
        engine.register_fn("PI", || -> f64 { std::f64::consts::PI });

        // ────────────────────────────────────────────────────────────────
        //  Model loading
        // ────────────────────────────────────────────────────────────────

        let m = Rc::clone(&model);
        engine.register_fn("load", move |path: &str| -> bool {
            let p = std::path::Path::new(path);
            match RobotModel::from_file(p) {
                Ok(mut robot) => {
                    robot.load_sidecar_config();
                    *m.borrow_mut() = Some(robot);
                    true
                }
                Err(e) => {
                    log::warn!("load({path}): {e}");
                    false
                }
            }
        });

        let m = Rc::clone(&model);
        engine.register_fn("model_name", move || -> String {
            m.borrow()
                .as_ref()
                .map(|r| r.name.clone())
                .unwrap_or_default()
        });

        let m = Rc::clone(&model);
        engine.register_fn("has_model", move || -> bool {
            m.borrow().is_some()
        });

        // ────────────────────────────────────────────────────────────────
        //  Link queries
        // ────────────────────────────────────────────────────────────────

        let m = Rc::clone(&model);
        engine.register_fn("link_names", move || -> Array {
            m.borrow()
                .as_ref()
                .map(|r| {
                    r.links
                        .iter()
                        .map(|l| Dynamic::from(l.name.clone()))
                        .collect()
                })
                .unwrap_or_default()
        });

        let m = Rc::clone(&model);
        engine.register_fn("num_links", move || -> i64 {
            m.borrow().as_ref().map(|r| r.links.len() as i64).unwrap_or(0)
        });

        let m = Rc::clone(&model);
        engine.register_fn("link_pos", move |name: &str| -> Array {
            let borrow = m.borrow();
            let Some(robot) = borrow.as_ref() else { return vec![].into() };
            let transforms = robot.compute_transforms();
            if let Some(tf) = transforms.get(name) {
                let t = tf.translation;
                vec![
                    Dynamic::from_float(t.x as f64),
                    Dynamic::from_float(t.y as f64),
                    Dynamic::from_float(t.z as f64),
                ]
            } else {
                vec![]
            }
        });

        let m = Rc::clone(&model);
        engine.register_fn("link_rpy", move |name: &str| -> Array {
            let borrow = m.borrow();
            let Some(robot) = borrow.as_ref() else { return vec![].into() };
            let transforms = robot.compute_transforms();
            if let Some(tf) = transforms.get(name) {
                let (r, p, y) = tf.rotation.euler_angles();
                vec![
                    Dynamic::from_float(r as f64),
                    Dynamic::from_float(p as f64),
                    Dynamic::from_float(y as f64),
                ]
            } else {
                vec![]
            }
        });

        // ────────────────────────────────────────────────────────────────
        //  Joint queries & manipulation
        // ────────────────────────────────────────────────────────────────

        let m = Rc::clone(&model);
        engine.register_fn("joint_names", move || -> Array {
            m.borrow()
                .as_ref()
                .map(|r| {
                    r.joints
                        .iter()
                        .map(|j| Dynamic::from(j.name.clone()))
                        .collect()
                })
                .unwrap_or_default()
        });

        let m = Rc::clone(&model);
        engine.register_fn("num_joints", move || -> i64 {
            m.borrow().as_ref().map(|r| r.joints.len() as i64).unwrap_or(0)
        });

        // get joint position by name
        let m = Rc::clone(&model);
        engine.register_fn("joint_pos", move |name: &str| -> f64 {
            let borrow = m.borrow();
            let Some(robot) = borrow.as_ref() else { return 0.0 };
            if let Some(&idx) = robot.joint_map.get(name) {
                robot.joint_positions[idx]
            } else {
                0.0
            }
        });

        // get joint position by index
        let m = Rc::clone(&model);
        engine.register_fn("joint_pos_idx", move |idx: i64| -> f64 {
            let borrow = m.borrow();
            let Some(robot) = borrow.as_ref() else { return 0.0 };
            let i = idx as usize;
            if i < robot.joint_positions.len() {
                robot.joint_positions[i]
            } else {
                0.0
            }
        });

        // get all joint positions as array
        let m = Rc::clone(&model);
        engine.register_fn("joint_positions", move || -> Array {
            m.borrow()
                .as_ref()
                .map(|r| {
                    r.joint_positions
                        .iter()
                        .map(|&v| Dynamic::from_float(v))
                        .collect()
                })
                .unwrap_or_default()
        });

        // set joint position by name
        let m = Rc::clone(&model);
        engine.register_fn("set_joint", move |name: &str, value: f64| {
            let mut borrow = m.borrow_mut();
            let Some(robot) = borrow.as_mut() else { return };
            if let Some(&idx) = robot.joint_map.get(name) {
                robot.joint_positions[idx] = value;
            }
        });

        // set joint position by index
        let m = Rc::clone(&model);
        engine.register_fn("set_joint_idx", move |idx: i64, value: f64| {
            let mut borrow = m.borrow_mut();
            let Some(robot) = borrow.as_mut() else { return };
            let i = idx as usize;
            if i < robot.joint_positions.len() {
                robot.joint_positions[i] = value;
            }
        });

        // set all joint positions from array
        let m = Rc::clone(&model);
        engine.register_fn("set_joints", move |values: Array| {
            let mut borrow = m.borrow_mut();
            let Some(robot) = borrow.as_mut() else { return };
            for (i, v) in values.iter().enumerate() {
                if i < robot.joint_positions.len() {
                    if let Some(f) = v.as_float().ok() {
                        robot.joint_positions[i] = f;
                    } else if let Some(n) = v.as_int().ok() {
                        robot.joint_positions[i] = n as f64;
                    }
                }
            }
        });

        // joint limits: [lower, upper]
        let m = Rc::clone(&model);
        engine.register_fn("joint_limits", move |name: &str| -> Array {
            let borrow = m.borrow();
            let Some(robot) = borrow.as_ref() else { return vec![].into() };
            if let Some(&idx) = robot.joint_map.get(name) {
                let j = &robot.joints[idx];
                vec![
                    Dynamic::from_float(j.lower),
                    Dynamic::from_float(j.upper),
                ]
            } else {
                vec![]
            }
        });

        // joint type string
        let m = Rc::clone(&model);
        engine.register_fn("joint_type", move |name: &str| -> String {
            let borrow = m.borrow();
            let Some(robot) = borrow.as_ref() else { return String::new() };
            if let Some(&idx) = robot.joint_map.get(name) {
                robot.joints[idx].joint_type.clone()
            } else {
                String::new()
            }
        });

        // ────────────────────────────────────────────────────────────────
        //  Forward Kinematics (convenience)
        // ────────────────────────────────────────────────────────────────

        // fk() → returns map of link_name → [x, y, z]
        let m = Rc::clone(&model);
        engine.register_fn("fk", move || -> rhai::Map {
            let borrow = m.borrow();
            let Some(robot) = borrow.as_ref() else { return rhai::Map::new() };
            let transforms = robot.compute_transforms();
            let mut map = rhai::Map::new();
            for (name, tf) in &transforms {
                let t = tf.translation;
                let arr: Array = vec![
                    Dynamic::from_float(t.x as f64),
                    Dynamic::from_float(t.y as f64),
                    Dynamic::from_float(t.z as f64),
                ];
                map.insert(name.clone().into(), Dynamic::from(arr));
            }
            map
        });

        // ────────────────────────────────────────────────────────────────
        //  Inverse Kinematics
        // ────────────────────────────────────────────────────────────────

        // ik(link_name, tx, ty, tz)  — 10 IK steps toward target, mutates joints
        let m = Rc::clone(&model);
        engine.register_fn("ik", move |link: &str, tx: f64, ty: f64, tz: f64| -> bool {
            let mut borrow = m.borrow_mut();
            let Some(robot) = borrow.as_mut() else { return false };
            let target = nalgebra::Point3::new(tx, ty, tz);

            if robot.joint_map.is_empty() {
                return false;
            }

            let chain = robot.chain_joints(link);
            if chain.is_empty() {
                return false;
            }

            for _ in 0..10 {
                let transforms = robot.compute_transforms();
                let ee_tf = match transforms.get(link) {
                    Some(tf) => tf,
                    None => return false,
                };
                let ee_pos = nalgebra::Point3::new(
                    ee_tf.translation.x as f64,
                    ee_tf.translation.y as f64,
                    ee_tf.translation.z as f64,
                );

                let err = (target - ee_pos).norm();
                if err < 1e-4 {
                    return true;
                }

                let deltas = robot.solve_ik_step(
                    &chain,
                    link,
                    None,         // root_link
                    &ee_pos,
                    &target,
                    0.01,         // damping
                    0.5,          // gain
                    0.1,          // max_step
                    None,         // ref_positions
                    IkSolver::Dls,
                    None,         // screen_axes
                    None,         // joint_weights
                );

                robot.apply_joint_deltas(&chain, &deltas);
            }
            true
        });

        // ik_steps(link_name, tx, ty, tz, steps) — configurable iteration count
        let m = Rc::clone(&model);
        engine.register_fn("ik_steps", move |link: &str, tx: f64, ty: f64, tz: f64, steps: i64| -> f64 {
            let mut borrow = m.borrow_mut();
            let Some(robot) = borrow.as_mut() else { return -1.0 };
            let target = nalgebra::Point3::new(tx, ty, tz);

            let chain = robot.chain_joints(link);
            if chain.is_empty() {
                return -1.0;
            }

            let mut err = f64::MAX;
            for _ in 0..steps.max(1) {
                let transforms = robot.compute_transforms();
                let ee_tf = match transforms.get(link) {
                    Some(tf) => tf,
                    None => return -1.0,
                };
                let ee_pos = nalgebra::Point3::new(
                    ee_tf.translation.x as f64,
                    ee_tf.translation.y as f64,
                    ee_tf.translation.z as f64,
                );

                err = (target - ee_pos).norm();
                if err < 1e-6 {
                    break;
                }

                let deltas = robot.solve_ik_step(
                    &chain,
                    link,
                    None,
                    &ee_pos,
                    &target,
                    0.01,
                    0.5,
                    0.1,
                    None,
                    IkSolver::Dls,
                    None,
                    None,
                );
                robot.apply_joint_deltas(&chain, &deltas);
            }
            err
        });

        // ────────────────────────────────────────────────────────────────
        //  Loop closures
        // ────────────────────────────────────────────────────────────────

        let m = Rc::clone(&model);
        engine.register_fn("add_loop_closure", move |name: &str, link_a: &str, ox_a: f64, oy_a: f64, oz_a: f64,
                                                       link_b: &str, ox_b: f64, oy_b: f64, oz_b: f64| {
            let mut borrow = m.borrow_mut();
            let Some(robot) = borrow.as_mut() else { return };
            robot.loop_closures.push(LoopClosure::position(
                name,
                link_a,
                nalgebra::Vector3::new(ox_a, oy_a, oz_a),
                link_b,
                nalgebra::Vector3::new(ox_b, oy_b, oz_b),
            ));
        });

        let m = Rc::clone(&model);
        engine.register_fn("loop_closure_error", move || -> f64 {
            let borrow = m.borrow();
            let Some(robot) = borrow.as_ref() else { return 0.0 };
            robot.loop_closure_error()
        });

        let m = Rc::clone(&model);
        engine.register_fn("num_loop_closures", move || -> i64 {
            let borrow = m.borrow();
            borrow.as_ref().map(|r| r.loop_closures.len() as i64).unwrap_or(0)
        });

        // ────────────────────────────────────────────────────────────────
        //  Export
        // ────────────────────────────────────────────────────────────────

        let m = Rc::clone(&model);
        engine.register_fn("export_urdf", move |path: &str| -> bool {
            let borrow = m.borrow();
            let Some(robot) = borrow.as_ref() else { return false };
            match robot.export_urdf_to_file(std::path::Path::new(path)) {
                Ok(()) => true,
                Err(e) => {
                    log::warn!("export_urdf({path}): {e}");
                    false
                }
            }
        });

        let m = Rc::clone(&model);
        engine.register_fn("export_sdf", move |path: &str| -> bool {
            let borrow = m.borrow();
            let Some(robot) = borrow.as_ref() else { return false };
            match crate::sdf::export_sdf_to_file(robot, std::path::Path::new(path)) {
                Ok(()) => true,
                Err(e) => {
                    log::warn!("export_sdf({path}): {e}");
                    false
                }
            }
        });

        let m = Rc::clone(&model);
        engine.register_fn("export_mjcf", move |path: &str| -> bool {
            let borrow = m.borrow();
            let Some(robot) = borrow.as_ref() else { return false };
            let xml = crate::mjcf::export_mjcf(robot);
            match std::fs::write(path, &xml) {
                Ok(()) => true,
                Err(e) => {
                    log::warn!("export_mjcf({path}): {e}");
                    false
                }
            }
        });

        // ────────────────────────────────────────────────────────────────
        //  Miscellaneous
        // ────────────────────────────────────────────────────────────────

        // dist(ax, ay, az, bx, by, bz) → euclidean distance
        engine.register_fn("dist", |ax: f64, ay: f64, az: f64, bx: f64, by: f64, bz: f64| -> f64 {
            ((ax - bx).powi(2) + (ay - by).powi(2) + (az - bz).powi(2)).sqrt()
        });

        // ────────────────────────────────────────────────────────────────
        //  Mesh reduction
        // ────────────────────────────────────────────────────────────────

        // reduce_mesh(link_name, visual_index, target_ratio)
        // Returns new triangle count, or -1 on error.
        let m = Rc::clone(&model);
        engine.register_fn("reduce_mesh", move |link: &str, vi: i64, ratio: f64| -> i64 {
            let mut borrow = m.borrow_mut();
            let Some(robot) = borrow.as_mut() else { return -1 };
            let Some(&li) = robot.link_map.get(link) else { return -1 };
            let vi = vi as usize;
            if vi >= robot.links[li].visuals.len() { return -1; }
            if let crate::robot::GeomData::Mesh { ref mut vertices, .. } = robot.links[li].visuals[vi].geometry {
                let mesh_data = misarta::mesh::MeshData::from_flat_vertices_f32(vertices);
                let reduced = mesh_data.decimate(ratio);
                *vertices = reduced.to_flat_vertices_f32();
                reduced.num_triangles() as i64
            } else {
                -1
            }
        });

        // reduce_collision_mesh(link_name, collision_index, target_ratio)
        let m = Rc::clone(&model);
        engine.register_fn("reduce_collision_mesh", move |link: &str, ci: i64, ratio: f64| -> i64 {
            let mut borrow = m.borrow_mut();
            let Some(robot) = borrow.as_mut() else { return -1 };
            let Some(&li) = robot.link_map.get(link) else { return -1 };
            let ci = ci as usize;
            if ci >= robot.links[li].collisions.len() { return -1; }
            if let crate::robot::GeomData::Mesh { ref mut vertices, .. } = robot.links[li].collisions[ci].geometry {
                let mesh_data = misarta::mesh::MeshData::from_flat_vertices_f32(vertices);
                let reduced = mesh_data.decimate(ratio);
                *vertices = reduced.to_flat_vertices_f32();
                reduced.num_triangles() as i64
            } else {
                -1
            }
        });

        // reduce_all_meshes(target_ratio) → total triangles removed
        let m = Rc::clone(&model);
        engine.register_fn("reduce_all_meshes", move |ratio: f64| -> i64 {
            let mut borrow = m.borrow_mut();
            let Some(robot) = borrow.as_mut() else { return -1 };
            let mut removed = 0i64;
            for link in &mut robot.links {
                for vis in &mut link.visuals {
                    if let crate::robot::GeomData::Mesh { ref mut vertices, .. } = vis.geometry {
                        let before = vertices.len() as i64 / 18;
                        let mesh_data = misarta::mesh::MeshData::from_flat_vertices_f32(vertices);
                        let reduced = mesh_data.decimate(ratio);
                        *vertices = reduced.to_flat_vertices_f32();
                        let after = reduced.num_triangles() as i64;
                        removed += before - after;
                    }
                }
                for col in &mut link.collisions {
                    if let crate::robot::GeomData::Mesh { ref mut vertices, .. } = col.geometry {
                        let before = vertices.len() as i64 / 18;
                        let mesh_data = misarta::mesh::MeshData::from_flat_vertices_f32(vertices);
                        let reduced = mesh_data.decimate(ratio);
                        *vertices = reduced.to_flat_vertices_f32();
                        let after = reduced.num_triangles() as i64;
                        removed += before - after;
                    }
                }
            }
            removed
        });

        engine
    }
}

impl ModelScriptEngine {
    /// Returns all function & keyword names available for tab completion.
    pub fn completion_candidates(&self) -> Vec<String> {
        // Registered Rhai functions
        let mut names: Vec<String> = Vec::new();

        // Model functions registered via register_fn
        let builtins = [
            "load", "model_name", "has_model",
            "link_names", "num_links", "link_pos", "link_rpy",
            "joint_names", "num_joints", "joint_pos", "joint_pos_idx",
            "joint_positions", "set_joint", "set_joint_idx", "set_joints",
            "joint_limits", "joint_type",
            "fk", "ik", "ik_steps",
            "add_loop_closure", "loop_closure_error", "num_loop_closures",
            "export_urdf", "export_sdf", "export_mjcf",
            "reduce_mesh", "reduce_collision_mesh", "reduce_all_meshes",
            "abs", "sqrt", "sin", "cos", "atan2",
            "min_f", "max_f", "clamp", "to_deg", "to_rad", "PI",
            "dist",
            "print",
            // Rhai keywords
            "let", "const", "if", "else", "while", "for", "in", "loop",
            "break", "continue", "return", "fn", "true", "false",
            // Built-in console commands
            "clear", "help",
        ];
        for name in builtins {
            names.push(name.to_string());
        }

        // Also add variable names from current scope
        for (name, _, _) in self.scope.iter() {
            let s = name.to_string();
            if !names.contains(&s) {
                names.push(s);
            }
        }

        names.sort();
        names.dedup();
        names
    }
}

impl Default for ModelScriptEngine {
    fn default() -> Self {
        Self::new()
    }
}

// ─────────────────────────────────────────────────────────────────────────
//  Tests
// ─────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::PathBuf;

    fn fixture_urdf() -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("tests/fixtures/urdf/test_robot.urdf")
    }

    fn fixture_sdf() -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("tests/fixtures/sdf/test_robot.sdf")
    }

    fn fixture_five_bar() -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("tests/fixtures/sdf/five_bar_parallel.sdf")
    }

    fn load_test_model() -> ModelScriptEngine {
        let robot = RobotModel::from_file(&fixture_urdf()).unwrap();
        ModelScriptEngine::with_model(robot)
    }

    #[test]
    fn test_load_from_script() {
        let mut eng = ModelScriptEngine::new();
        let path = fixture_urdf().display().to_string();
        let src = format!(r#"load("{path}")"#);
        eng.eval(&src).unwrap();
        assert!(eng.model().is_some());
    }

    #[test]
    fn test_model_name() {
        let mut eng = load_test_model();
        let out = eng.eval(r#"print(model_name())"#).unwrap();
        assert!(!out[0].is_empty());
    }

    #[test]
    fn test_link_names() {
        let mut eng = load_test_model();
        let out = eng.eval(r#"
            let names = link_names();
            print("count=" + names.len());
        "#).unwrap();
        assert!(out[0].starts_with("count="));
        let n: usize = out[0].strip_prefix("count=").unwrap().parse().unwrap();
        assert!(n >= 3); // test_robot has at least 3 links
    }

    #[test]
    fn test_joint_get_set() {
        let mut eng = load_test_model();
        let out = eng.eval(r#"
            let names = joint_names();
            let jname = names[0];
            set_joint(jname, 0.42);
            print(joint_pos(jname));
        "#).unwrap();
        let val: f64 = out[0].parse().unwrap();
        assert!((val - 0.42).abs() < 1e-9);
    }

    #[test]
    fn test_joint_positions_array() {
        let mut eng = load_test_model();
        let out = eng.eval(r#"
            let q = joint_positions();
            print(q.len());
        "#).unwrap();
        let n: usize = out[0].parse().unwrap();
        assert!(n >= 2);
    }

    #[test]
    fn test_link_pos() {
        let mut eng = load_test_model();
        let out = eng.eval(r#"
            let p = link_pos("base_link");
            print(p.len());
        "#).unwrap();
        assert_eq!(out[0], "3");
    }

    #[test]
    fn test_fk_returns_map() {
        let mut eng = load_test_model();
        let out = eng.eval(r#"
            let m = fk();
            print(m.len());
        "#).unwrap();
        let n: usize = out[0].parse().unwrap();
        assert!(n >= 3);
    }

    #[test]
    fn test_ik_reduces_error() {
        let mut eng = load_test_model();
        let out = eng.eval(r#"
            let names = link_names();
            let ee = names[names.len() - 1];  // last link as end-effector
            let p0 = link_pos(ee);
            // Target slightly offset from current position
            let tx = p0[0] + 0.01;
            let ty = p0[1];
            let tz = p0[2] + 0.01;
            let ok = ik(ee, tx, ty, tz);
            let p1 = link_pos(ee);
            let d = dist(p1[0], p1[1], p1[2], tx, ty, tz);
            print(d);
        "#).unwrap();
        let d: f64 = out[0].parse().unwrap();
        assert!(d < 0.05, "IK should reduce error, got d={d}");
    }

    #[test]
    fn test_joint_limits() {
        let mut eng = load_test_model();
        let out = eng.eval(r#"
            let names = joint_names();
            let lim = joint_limits(names[0]);
            print(lim.len());
        "#).unwrap();
        assert_eq!(out[0], "2");
    }

    #[test]
    fn test_joint_type() {
        let mut eng = load_test_model();
        let out = eng.eval(r#"
            let names = joint_names();
            print(joint_type(names[0]));
        "#).unwrap();
        assert!(!out[0].is_empty());
    }

    #[test]
    fn test_loop_closure() {
        let robot = crate::sdf::import_sdf(&fixture_five_bar()).unwrap();
        let mut eng = ModelScriptEngine::with_model(robot);
        let out = eng.eval(r#"
            add_loop_closure("test_loop", "end_effector", 0.0, 0.0, 0.0,
                             "distal_right", 0.0, 0.0, 0.2);
            print(num_loop_closures());
            let e = loop_closure_error();
            print(e);
        "#).unwrap();
        assert_eq!(out[0], "1");
        let err: f64 = out[1].parse().unwrap();
        assert!(err < 0.02, "Loop closure error at q=0 should be small, got {err}");
    }

    #[test]
    fn test_math_functions() {
        let mut eng = ModelScriptEngine::new();
        let out = eng.eval(r#"
            print(sin(0.0));
            print(abs(-3.14));
            print(to_deg(PI()));
        "#).unwrap();
        assert!((out[0].parse::<f64>().unwrap()).abs() < 1e-9);
        assert!((out[1].parse::<f64>().unwrap() - 3.14).abs() < 1e-9);
        assert!((out[2].parse::<f64>().unwrap() - 180.0).abs() < 1e-3);
    }

    #[test]
    fn test_no_model_graceful() {
        let mut eng = ModelScriptEngine::new();
        let out = eng.eval(r#"
            print(num_links());
            print(num_joints());
            print(model_name());
        "#).unwrap();
        assert_eq!(out[0], "0");
        assert_eq!(out[1], "0");
        assert_eq!(out[2], "");
    }

    #[test]
    fn test_infinite_loop_protection() {
        let mut eng = ModelScriptEngine::new();
        let result = eng.eval("loop { }");
        assert!(result.is_err());
    }
}
