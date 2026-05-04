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

#[cfg(feature = "mujoco")]
use crate::mujoco_sim::MujocoSim;

// ─────────────────────────────────────────────────────────────────────────
//  ModelScriptEngine
// ─────────────────────────────────────────────────────────────────────────

/// A Rhai scripting engine with full RobotModel manipulation bindings.
pub struct ModelScriptEngine {
    engine: Engine,
    ast: Option<AST>,
    scope: Scope<'static>,
    model: Rc<RefCell<Option<RobotModel>>>,
    /// MuJoCo sim handle borrowed from the host app for the duration of an
    /// `eval()` call. Set via [`Self::set_mujoco_sim`] before `run()` and
    /// taken back via [`Self::take_mujoco_sim`] afterwards. While present,
    /// scripts can drive playback through `mj_step`, `play_pose`, etc.
    #[cfg(feature = "mujoco")]
    mujoco_sim: Rc<RefCell<Option<MujocoSim>>>,
    /// Quadruped gait controller handle, borrowed from the host similarly
    /// to `mujoco_sim`. Allows scripts to call `gait_setup`, `gait_start`,
    /// `gait_set_velocity` etc. while the engine is evaluating.
    gait_controller: Rc<RefCell<Option<crate::gait::GaitController>>>,
    /// Captured print output lines.
    output_lines: Rc<RefCell<Vec<String>>>,
    last_error: Option<String>,
}

impl ModelScriptEngine {
    /// Create a new model scripting engine.
    pub fn new() -> Self {
        let model: Rc<RefCell<Option<RobotModel>>> = Rc::new(RefCell::new(None));
        let output_lines: Rc<RefCell<Vec<String>>> = Rc::new(RefCell::new(Vec::new()));
        #[cfg(feature = "mujoco")]
        let mujoco_sim: Rc<RefCell<Option<MujocoSim>>> = Rc::new(RefCell::new(None));
        let gait_controller: Rc<RefCell<Option<crate::gait::GaitController>>> =
            Rc::new(RefCell::new(None));
        let engine = Self::build_engine(
            Rc::clone(&model),
            Rc::clone(&output_lines),
            #[cfg(feature = "mujoco")]
            Rc::clone(&mujoco_sim),
            Rc::clone(&gait_controller),
        );
        Self {
            engine,
            ast: None,
            scope: Scope::new(),
            model,
            #[cfg(feature = "mujoco")]
            mujoco_sim,
            gait_controller,
            output_lines,
            last_error: None,
        }
    }

    /// Create with a pre-loaded model.
    #[allow(dead_code)]
    pub fn with_model(robot: RobotModel) -> Self {
        let eng = Self::new();
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
    #[allow(dead_code)]
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

    /// Hand the active MuJoCo sim to the script engine for the duration of
    /// the next `eval()`. The script can drive it via `mj_step`, `play_pose`,
    /// `apply_force`, etc. The host should call [`Self::take_mujoco_sim`]
    /// after the eval to put the (possibly mutated) sim back.
    #[cfg(feature = "mujoco")]
    pub fn set_mujoco_sim(&mut self, sim: Option<MujocoSim>) {
        *self.mujoco_sim.borrow_mut() = sim;
    }

    /// Take the MuJoCo sim back out of the engine. Returns whatever the
    /// script left in place (including `None` if it was stopped via
    /// `mj_stop`).
    #[cfg(feature = "mujoco")]
    pub fn take_mujoco_sim(&mut self) -> Option<MujocoSim> {
        self.mujoco_sim.borrow_mut().take()
    }

    /// Same handover semantics as [`Self::set_mujoco_sim`] but for the
    /// quadruped-gait controller. Lets scripts call `gait_setup`,
    /// `gait_start` etc. while the engine is running.
    pub fn set_gait_controller(
        &mut self,
        gc: Option<crate::gait::GaitController>,
    ) {
        *self.gait_controller.borrow_mut() = gc;
    }

    pub fn take_gait_controller(&mut self) -> Option<crate::gait::GaitController> {
        self.gait_controller.borrow_mut().take()
    }

    /// Clear scope (but keep model).
    #[allow(dead_code)]
    pub fn reset_scope(&mut self) {
        self.scope.clear();
    }

    // ── Engine builder ──────────────────────────────────────────────────

    fn build_engine(
        model: Rc<RefCell<Option<RobotModel>>>,
        output: Rc<RefCell<Vec<String>>>,
        #[cfg(feature = "mujoco")] mujoco_sim: Rc<RefCell<Option<MujocoSim>>>,
        gait_controller: Rc<RefCell<Option<crate::gait::GaitController>>>,
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
                    None,         // ee_offset_world
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
                    None,         // ee_offset_world
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

        // Helper closure to parse decimation method from Rhai (returns fn)
        fn parse_method(s: &str) -> misarta::decimate::DecimationMethod {
            misarta::decimate::DecimationMethod::from_str_loose(s)
        }

        // reduce_mesh(link_name, visual_index, target_ratio)  — QEM default
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

        // reduce_mesh(link_name, visual_index, target_ratio, method_string)
        let m = Rc::clone(&model);
        engine.register_fn("reduce_mesh", move |link: &str, vi: i64, ratio: f64, method: &str| -> i64 {
            let mut borrow = m.borrow_mut();
            let Some(robot) = borrow.as_mut() else { return -1 };
            let Some(&li) = robot.link_map.get(link) else { return -1 };
            let vi = vi as usize;
            if vi >= robot.links[li].visuals.len() { return -1; }
            if let crate::robot::GeomData::Mesh { ref mut vertices, .. } = robot.links[li].visuals[vi].geometry {
                let mesh_data = misarta::mesh::MeshData::from_flat_vertices_f32(vertices);
                let reduced = mesh_data.decimate_with(ratio, parse_method(method));
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

        // reduce_collision_mesh(link_name, collision_index, target_ratio, method_string)
        let m = Rc::clone(&model);
        engine.register_fn("reduce_collision_mesh", move |link: &str, ci: i64, ratio: f64, method: &str| -> i64 {
            let mut borrow = m.borrow_mut();
            let Some(robot) = borrow.as_mut() else { return -1 };
            let Some(&li) = robot.link_map.get(link) else { return -1 };
            let ci = ci as usize;
            if ci >= robot.links[li].collisions.len() { return -1; }
            if let crate::robot::GeomData::Mesh { ref mut vertices, .. } = robot.links[li].collisions[ci].geometry {
                let mesh_data = misarta::mesh::MeshData::from_flat_vertices_f32(vertices);
                let reduced = mesh_data.decimate_with(ratio, parse_method(method));
                *vertices = reduced.to_flat_vertices_f32();
                reduced.num_triangles() as i64
            } else {
                -1
            }
        });

        // reduce_all_meshes(target_ratio) → total triangles removed (QEM)
        let m = Rc::clone(&model);
        engine.register_fn("reduce_all_meshes", move |ratio: f64| -> i64 {
            let mut borrow = m.borrow_mut();
            let Some(robot) = borrow.as_mut() else { return -1 };
            reduce_all_meshes_impl(robot, ratio, misarta::decimate::DecimationMethod::Qem)
        });

        // reduce_all_meshes(target_ratio, method_string)
        let m = Rc::clone(&model);
        engine.register_fn("reduce_all_meshes", move |ratio: f64, method: &str| -> i64 {
            let mut borrow = m.borrow_mut();
            let Some(robot) = borrow.as_mut() else { return -1 };
            reduce_all_meshes_impl(robot, ratio, parse_method(method))
        });

        // ────────────────────────────────────────────────────────────────
        //  Mesh decomposition (V-HACD / Sphere Tree)
        // ────────────────────────────────────────────────────────────────

        // decompose_vhacd(link_name, collision_index) → number of hulls produced
        let m = Rc::clone(&model);
        engine.register_fn("decompose_vhacd", move |link: &str, ci: i64| -> i64 {
            let mut borrow = m.borrow_mut();
            let Some(robot) = borrow.as_mut() else { return -1 };
            decompose_collision_impl(robot, link, ci as usize, misarta::decompose::DecompositionMethod::Vhacd, None)
        });

        // decompose_vhacd(link_name, collision_index, max_hulls) → number of hulls
        let m = Rc::clone(&model);
        engine.register_fn("decompose_vhacd", move |link: &str, ci: i64, max_hulls: i64| -> i64 {
            let mut borrow = m.borrow_mut();
            let Some(robot) = borrow.as_mut() else { return -1 };
            decompose_collision_impl(robot, link, ci as usize, misarta::decompose::DecompositionMethod::Vhacd, Some(max_hulls as usize))
        });

        // decompose_spheres(link_name, collision_index) → number of spheres
        let m = Rc::clone(&model);
        engine.register_fn("decompose_spheres", move |link: &str, ci: i64| -> i64 {
            let mut borrow = m.borrow_mut();
            let Some(robot) = borrow.as_mut() else { return -1 };
            decompose_collision_impl(robot, link, ci as usize, misarta::decompose::DecompositionMethod::SphereTree, None)
        });

        // decompose_spheres(link_name, collision_index, max_spheres) → number of spheres
        let m = Rc::clone(&model);
        engine.register_fn("decompose_spheres", move |link: &str, ci: i64, max_count: i64| -> i64 {
            let mut borrow = m.borrow_mut();
            let Some(robot) = borrow.as_mut() else { return -1 };
            decompose_collision_impl(robot, link, ci as usize, misarta::decompose::DecompositionMethod::SphereTree, Some(max_count as usize))
        });

        // decompose_primitive(link_name, collision_index) → 1 on success (single primitive, no V-HACD)
        let m = Rc::clone(&model);
        engine.register_fn("decompose_primitive", move |link: &str, ci: i64| -> i64 {
            let mut borrow = m.borrow_mut();
            let Some(robot) = borrow.as_mut() else { return -1 };
            decompose_collision_impl(robot, link, ci as usize, misarta::decompose::DecompositionMethod::PrimitiveFitDirect, None)
        });

        // ────────────────────────────────────────────────────────────────
        //  MuJoCo sim control
        // ────────────────────────────────────────────────────────────────
        //
        // The host (script_console.rs) hands the live `MujocoSim` to the
        // engine via `set_mujoco_sim` before each eval and takes it back
        // afterwards, so all sim functions below act on the actual sim
        // shown in the viewport. `mj_active()` returns false outside that
        // window so scripts can check before issuing commands.

        #[cfg(feature = "mujoco")]
        {
            let s = Rc::clone(&mujoco_sim);
            engine.register_fn("mj_active", move || -> bool {
                s.borrow().is_some()
            });

            // mj_start() — construct a sim from the current model with
            // default options (auto-base lift, ground plane, no axis locks,
            // motor actuators on every joint). Returns true on success.
            let s = Rc::clone(&mujoco_sim);
            let m = Rc::clone(&model);
            engine.register_fn("mj_start", move || -> bool {
                let model_borrow = m.borrow();
                let Some(robot) = model_borrow.as_ref() else {
                    return false;
                };
                // Scripts default to "limits baked in" (matches the catalogue
                // hardware spec). To probe unconstrained motion from a script,
                // toggle `set_armature_all` / damping / Kp instead, or rebuild
                // via the UI Play button with the ⛔ Limits checkbox off.
                let opts = crate::mjcf::MjcfExportOptions {
                    base_pos: None,
                    ground_plane: Some(crate::mjcf::GroundPlaneCfg {
                        z: 0.0,
                        half_size: 2.0,
                        roll: 0.0,
                        pitch: 0.0,
                    }),
                    add_actuators: true,
                    base_locked_axes: [false; 6],
                    ..Default::default()
                };
                match MujocoSim::new(robot, opts) {
                    Ok(sim) => {
                        *s.borrow_mut() = Some(sim);
                        true
                    }
                    Err(e) => {
                        log::warn!("mj_start: {e}");
                        false
                    }
                }
            });

            // Step the sim by `n` physics frames, advancing the model along.
            // Returns the number of frames actually stepped, or 0 if there is
            // no active sim / no model. `mj_step` from scripts always runs
            // *without* actuator-limit clamping so test scripts can probe
            // the unconstrained response — use the UI toggle to enforce
            // limits on the interactive playback path.
            let s = Rc::clone(&mujoco_sim);
            let m = Rc::clone(&model);
            engine.register_fn("mj_step", move |n: i64| -> i64 {
                let mut sim_borrow = s.borrow_mut();
                let mut model_borrow = m.borrow_mut();
                let (Some(sim), Some(robot)) = (sim_borrow.as_mut(), model_borrow.as_mut()) else {
                    return 0;
                };
                let n = n.max(0) as u32;
                sim.step_n_frames(robot, n, false);
                n as i64
            });

            // Step backwards through the snapshot history.
            let s = Rc::clone(&mujoco_sim);
            let m = Rc::clone(&model);
            engine.register_fn("mj_step_back", move |n: i64| -> i64 {
                let mut sim_borrow = s.borrow_mut();
                let mut model_borrow = m.borrow_mut();
                let (Some(sim), Some(robot)) = (sim_borrow.as_mut(), model_borrow.as_mut()) else {
                    return 0;
                };
                let n = n.max(0) as u32;
                sim.step_back_frames(robot, n);
                n as i64
            });

            // Native physics timestep (s).
            let s = Rc::clone(&mujoco_sim);
            engine.register_fn("mj_timestep", move || -> f64 {
                s.borrow().as_ref().map(|x| x.timestep()).unwrap_or(0.0)
            });

            // History buffer length (number of frames available for backward stepping).
            let s = Rc::clone(&mujoco_sim);
            engine.register_fn("mj_history_len", move || -> i64 {
                s.borrow().as_ref().map(|x| x.history_len() as i64).unwrap_or(0)
            });

            // Number of (q, q̇, τ) samples in the time-series trace ring buffer.
            // Useful in tuning scripts that want to know how much data the
            // upcoming `save_peaks_csv` call will write.
            let s = Rc::clone(&mujoco_sim);
            engine.register_fn("mj_trace_len", move || -> i64 {
                s.borrow().as_ref().map(|x| x.trace_len() as i64).unwrap_or(0)
            });

            // ── Async timeline API ────────────────────────────────────────
            // These functions don't execute their op immediately — they push
            // it to the sim's async queue, which the host UI loop drains a
            // little each frame. This is what lets a script "schedule" a
            // jump and have the user actually see it animate in the viewport
            // rather than a synchronous mj_step batch that freezes the UI.

            let s = Rc::clone(&mujoco_sim);
            engine.register_fn("mj_async_step_seconds", move |seconds: f64| -> i64 {
                let mut sim_borrow = s.borrow_mut();
                let Some(sim) = sim_borrow.as_mut() else {
                    return -1;
                };
                let dt = sim.timestep().max(1e-6);
                let frames = (seconds / dt).max(0.0).round() as u32;
                if frames == 0 {
                    return 0;
                }
                sim.async_enqueue(crate::mujoco_sim::AsyncSimOp::StepFrames(frames));
                frames as i64
            });

            let s = Rc::clone(&mujoco_sim);
            engine.register_fn("mj_async_step_frames", move |n: i64| -> i64 {
                let mut sim_borrow = s.borrow_mut();
                let Some(sim) = sim_borrow.as_mut() else {
                    return -1;
                };
                let frames = n.max(0) as u32;
                if frames == 0 {
                    return 0;
                }
                sim.async_enqueue(crate::mujoco_sim::AsyncSimOp::StepFrames(frames));
                frames as i64
            });

            let s = Rc::clone(&mujoco_sim);
            let m = Rc::clone(&model);
            engine.register_fn(
                "mj_async_set_position_target",
                move |name: &str, target: f64| -> bool {
                    let mut sim_borrow = s.borrow_mut();
                    let model_borrow = m.borrow();
                    let (Some(sim), Some(robot)) =
                        (sim_borrow.as_mut(), model_borrow.as_ref())
                    else {
                        return false;
                    };
                    let Some(&idx) = robot.joint_map.get(name) else {
                        return false;
                    };
                    sim.async_enqueue(
                        crate::mujoco_sim::AsyncSimOp::SetPositionTarget(idx, target),
                    );
                    true
                },
            );

            let s = Rc::clone(&mujoco_sim);
            engine.register_fn("mj_async_print", move |msg: &str| -> bool {
                let mut sim_borrow = s.borrow_mut();
                let Some(sim) = sim_borrow.as_mut() else {
                    return false;
                };
                sim.async_enqueue(crate::mujoco_sim::AsyncSimOp::Print(msg.to_string()));
                true
            });

            let s = Rc::clone(&mujoco_sim);
            engine.register_fn("mj_async_save_csv", move |path: &str| -> bool {
                let mut sim_borrow = s.borrow_mut();
                let Some(sim) = sim_borrow.as_mut() else {
                    return false;
                };
                sim.async_enqueue(crate::mujoco_sim::AsyncSimOp::SaveCsv(
                    std::path::PathBuf::from(path),
                ));
                true
            });

            let s = Rc::clone(&mujoco_sim);
            engine.register_fn("mj_async_pending", move || -> i64 {
                s.borrow().as_ref().map(|x| x.async_pending() as i64).unwrap_or(-1)
            });

            let s = Rc::clone(&mujoco_sim);
            engine.register_fn("mj_async_clear", move || -> i64 {
                let mut sim_borrow = s.borrow_mut();
                let Some(sim) = sim_borrow.as_mut() else {
                    return -1;
                };
                let n = sim.async_pending() as i64;
                sim.async_clear();
                n
            });

            // Toggle gravity-compensation feedforward in the controller.
            // Returns the new value as i64 (1=on, 0=off, -1 if no sim).
            let s = Rc::clone(&mujoco_sim);
            engine.register_fn("mj_gravity_compensation", move |on: bool| -> i64 {
                let mut sim_borrow = s.borrow_mut();
                let Some(sim) = sim_borrow.as_mut() else {
                    return -1;
                };
                sim.set_gravity_compensation(on);
                if on { 1 } else { 0 }
            });

            // Resize the trace ring buffer cap. Existing samples beyond the
            // new cap are dropped from the front. Returns the value applied.
            let s = Rc::clone(&mujoco_sim);
            engine.register_fn("mj_set_trace_max", move |max: i64| -> i64 {
                let mut sim_borrow = s.borrow_mut();
                let Some(sim) = sim_borrow.as_mut() else {
                    return 0;
                };
                let m = max.max(1) as usize;
                sim.set_trace_max(m);
                m as i64
            });

            // Write the captured trace to a CSV file (same format the UI
            // "💾 Save CSV" button produces). Returns the number of rows
            // written, or -1 on error so scripts can detect failures.
            let s = Rc::clone(&mujoco_sim);
            let m = Rc::clone(&model);
            engine.register_fn("save_peaks_csv", move |path: &str| -> i64 {
                let sim_borrow = s.borrow();
                let model_borrow = m.borrow();
                let (Some(sim), Some(robot)) = (
                    sim_borrow.as_ref(),
                    model_borrow.as_ref(),
                ) else {
                    return -1;
                };
                let p = std::path::Path::new(path);
                match crate::mujoco_sim::save_peaks_csv(robot, sim, p) {
                    Ok(n) => n as i64,
                    Err(e) => {
                        log::warn!("save_peaks_csv: {e}");
                        -1
                    }
                }
            });

            // Stop the sim and restore the pre-sim pose. Returns true if a
            // sim was running.
            let s = Rc::clone(&mujoco_sim);
            let m = Rc::clone(&model);
            engine.register_fn("mj_stop", move || -> bool {
                let mut sim_borrow = s.borrow_mut();
                let Some(sim) = sim_borrow.as_ref() else {
                    return false;
                };
                if let Some(robot) = m.borrow_mut().as_mut() {
                    sim.restore(robot);
                }
                *sim_borrow = None;
                true
            });

            // Start a smooth pose transition by name (uses the pose's stored
            // duration / kind). Returns true on success.
            let s = Rc::clone(&mujoco_sim);
            let m = Rc::clone(&model);
            engine.register_fn("play_pose", move |name: &str| -> bool {
                let mut sim_borrow = s.borrow_mut();
                let model_borrow = m.borrow();
                let (Some(sim), Some(robot)) = (sim_borrow.as_mut(), model_borrow.as_ref()) else {
                    return false;
                };
                let Some(pose) = robot.poses.iter().find(|p| p.name == name) else {
                    return false;
                };
                let q = pose.to_vector(robot, &robot.joint_positions);
                sim.start_transition(q, pose.duration, pose.kind);
                true
            });

            // play_pose(name, duration) — explicit duration override.
            let s = Rc::clone(&mujoco_sim);
            let m = Rc::clone(&model);
            engine.register_fn("play_pose", move |name: &str, duration: f64| -> bool {
                let mut sim_borrow = s.borrow_mut();
                let model_borrow = m.borrow();
                let (Some(sim), Some(robot)) = (sim_borrow.as_mut(), model_borrow.as_ref()) else {
                    return false;
                };
                let Some(pose) = robot.poses.iter().find(|p| p.name == name) else {
                    return false;
                };
                let q = pose.to_vector(robot, &robot.joint_positions);
                sim.start_transition(q, duration, pose.kind);
                true
            });

            // Whether a transition is currently playing.
            let s = Rc::clone(&mujoco_sim);
            engine.register_fn("transition_in_progress", move || -> bool {
                s.borrow().as_ref().map(|x| x.transition_in_progress()).unwrap_or(false)
            });

            // Start a chained-pose sequence by name. Returns true on
            // success. The model is read to build the keyframe animation
            // (uses each step's pose's stored joint vector at the moment
            // play_sequence is called); the sim then drives position
            // targets from the resulting timeline tick-by-tick.
            let s = Rc::clone(&mujoco_sim);
            let m = Rc::clone(&model);
            engine.register_fn("play_sequence", move |name: &str| -> bool {
                let mut sim_borrow = s.borrow_mut();
                let model_borrow = m.borrow();
                let (Some(sim), Some(robot)) =
                    (sim_borrow.as_mut(), model_borrow.as_ref())
                else {
                    return false;
                };
                let Some(anim) = robot.build_sequence_animation(name) else {
                    return false;
                };
                sim.start_sequence(anim, name.to_string());
                true
            });

            // Whether a sequence is currently playing.
            let s = Rc::clone(&mujoco_sim);
            engine.register_fn("sequence_in_progress", move || -> bool {
                s.borrow()
                    .as_ref()
                    .map(|x| x.sequence_in_progress())
                    .unwrap_or(false)
            });

            // Normalised sequence progress in [0, 1], or -1 if no sequence.
            let s = Rc::clone(&mujoco_sim);
            engine.register_fn("sequence_progress", move || -> f64 {
                s.borrow()
                    .as_ref()
                    .and_then(|x| x.sequence_progress())
                    .map(|p| p as f64)
                    .unwrap_or(-1.0)
            });

            // Normalised transition progress (0..1), or -1 if idle.
            let s = Rc::clone(&mujoco_sim);
            engine.register_fn("transition_progress", move || -> f64 {
                s.borrow()
                    .as_ref()
                    .and_then(|x| x.transition_progress())
                    .map(|p| p as f64)
                    .unwrap_or(-1.0)
            });

            // apply_force(link, fx, fy, fz, tx, ty, tz, duration_s)
            let s = Rc::clone(&mujoco_sim);
            engine.register_fn(
                "apply_force",
                move |link: &str,
                      fx: f64,
                      fy: f64,
                      fz: f64,
                      tx: f64,
                      ty: f64,
                      tz: f64,
                      dur: f64|
                      -> bool {
                    let mut sim_borrow = s.borrow_mut();
                    let Some(sim) = sim_borrow.as_mut() else {
                        return false;
                    };
                    sim.apply_external_force(link, [fx, fy, fz], [tx, ty, tz], dur);
                    true
                },
            );

            let s = Rc::clone(&mujoco_sim);
            engine.register_fn("cancel_force", move |link: &str| -> bool {
                let mut sim_borrow = s.borrow_mut();
                let Some(sim) = sim_borrow.as_mut() else {
                    return false;
                };
                sim.cancel_external_force(link)
            });

            // ── Joint peaks (since last reset / last play / last pulse) ──

            // Reset all peaks to zero.
            let s = Rc::clone(&mujoco_sim);
            engine.register_fn("reset_peaks", move || -> bool {
                let mut sim_borrow = s.borrow_mut();
                let Some(sim) = sim_borrow.as_mut() else {
                    return false;
                };
                sim.reset_peaks();
                true
            });

            // peak_torque(joint_name) → max |τ| observed since last reset
            // (N·m for revolute, N for prismatic). Returns 0 if not found.
            let s = Rc::clone(&mujoco_sim);
            let m = Rc::clone(&model);
            engine.register_fn("peak_torque", move |name: &str| -> f64 {
                let sim_borrow = s.borrow();
                let model_borrow = m.borrow();
                let (Some(sim), Some(robot)) = (sim_borrow.as_ref(), model_borrow.as_ref()) else {
                    return 0.0;
                };
                let Some(idx) = sim.joint_index(robot, name) else {
                    return 0.0;
                };
                sim.peaks().get(idx).map(|p| p.tau_abs).unwrap_or(0.0)
            });

            // peak_velocity(joint_name) → max |q̇| observed since last reset
            // (rad/s for revolute, m/s for prismatic). Returns 0 if not found.
            let s = Rc::clone(&mujoco_sim);
            let m = Rc::clone(&model);
            engine.register_fn("peak_velocity", move |name: &str| -> f64 {
                let sim_borrow = s.borrow();
                let model_borrow = m.borrow();
                let (Some(sim), Some(robot)) = (sim_borrow.as_ref(), model_borrow.as_ref()) else {
                    return 0.0;
                };
                let Some(idx) = sim.joint_index(robot, name) else {
                    return 0.0;
                };
                sim.peaks().get(idx).map(|p| p.qvel_abs).unwrap_or(0.0)
            });

            // peaks() → Map of {joint_name: [tau_abs, qvel_abs]} for all
            // movable joints. Convenient one-shot read for status print.
            let s = Rc::clone(&mujoco_sim);
            let m = Rc::clone(&model);
            engine.register_fn("peaks", move || -> rhai::Map {
                let mut map = rhai::Map::new();
                let sim_borrow = s.borrow();
                let model_borrow = m.borrow();
                let (Some(sim), Some(robot)) = (sim_borrow.as_ref(), model_borrow.as_ref()) else {
                    return map;
                };
                let peaks = sim.peaks();
                for (i, j) in robot.joints.iter().enumerate() {
                    if j.joint_type == "fixed" {
                        continue;
                    }
                    let p = peaks.get(i).cloned().unwrap_or_default();
                    let arr: Array = vec![
                        Dynamic::from_float(p.tau_abs),
                        Dynamic::from_float(p.qvel_abs),
                    ];
                    map.insert(j.name.clone().into(), Dynamic::from(arr));
                }
                map
            });

            // ── Per-joint actuator gain / mode / target setters ──

            let m = Rc::clone(&model);
            engine.register_fn("set_kp", move |name: &str, kp: f64| -> bool {
                let mut model_borrow = m.borrow_mut();
                let Some(robot) = model_borrow.as_mut() else {
                    return false;
                };
                let Some(&idx) = robot.joint_map.get(name) else {
                    return false;
                };
                robot.joints[idx].actuator_kp = kp;
                true
            });

            let m = Rc::clone(&model);
            engine.register_fn("set_kv", move |name: &str, kv: f64| -> bool {
                let mut model_borrow = m.borrow_mut();
                let Some(robot) = model_borrow.as_mut() else {
                    return false;
                };
                let Some(&idx) = robot.joint_map.get(name) else {
                    return false;
                };
                robot.joints[idx].actuator_kv = kv;
                true
            });

            // Per-joint actuator mode by name. Accepts "Position", "Velocity",
            // "Torque", or "ComputedTorque" (case-insensitive). Returns true
            // on success.
            let m = Rc::clone(&model);
            engine.register_fn(
                "set_actuator_mode",
                move |name: &str, mode: &str| -> bool {
                    let parsed = match mode.to_ascii_lowercase().as_str() {
                        "position" => crate::rbd::model::ActuatorMode::Position,
                        "velocity" => crate::rbd::model::ActuatorMode::Velocity,
                        "torque" => crate::rbd::model::ActuatorMode::Torque,
                        "computedtorque" | "computed_torque" | "computed-torque"
                            | "ct" => crate::rbd::model::ActuatorMode::ComputedTorque,
                        _ => return false,
                    };
                    let mut model_borrow = m.borrow_mut();
                    let Some(robot) = model_borrow.as_mut() else {
                        return false;
                    };
                    let Some(&idx) = robot.joint_map.get(name) else {
                        return false;
                    };
                    robot.joints[idx].actuator_mode = parsed;
                    true
                },
            );

            // Bulk-set actuator mode on every non-fixed joint. Same string
            // syntax as `set_actuator_mode`. Returns the number of joints
            // touched, or -1 on a typo.
            let m = Rc::clone(&model);
            engine.register_fn("set_actuator_mode_all", move |mode: &str| -> i64 {
                let parsed = match mode.to_ascii_lowercase().as_str() {
                    "position" => crate::rbd::model::ActuatorMode::Position,
                    "velocity" => crate::rbd::model::ActuatorMode::Velocity,
                    "torque" => crate::rbd::model::ActuatorMode::Torque,
                    "computedtorque" | "computed_torque" | "computed-torque"
                        | "ct" => crate::rbd::model::ActuatorMode::ComputedTorque,
                    _ => return -1,
                };
                let mut model_borrow = m.borrow_mut();
                let Some(robot) = model_borrow.as_mut() else {
                    return 0;
                };
                let mut n = 0i64;
                for j in robot.joints.iter_mut() {
                    if j.joint_type != "fixed" {
                        j.actuator_mode = parsed;
                        n += 1;
                    }
                }
                n
            });

            // Per-joint armature (rotor inertia, kg·m²). Mapped to MuJoCo
            // `<joint armature>` at the next sim start.
            let m = Rc::clone(&model);
            engine.register_fn("set_armature", move |name: &str, value: f64| -> bool {
                let mut model_borrow = m.borrow_mut();
                let Some(robot) = model_borrow.as_mut() else {
                    return false;
                };
                let Some(&idx) = robot.joint_map.get(name) else {
                    return false;
                };
                robot.joints[idx].armature = value;
                true
            });

            // Per-joint passive viscous damping (N·m·s/rad). Mapped to MuJoCo
            // `<joint damping>` at the next sim start.
            let m = Rc::clone(&model);
            engine.register_fn(
                "set_joint_damping",
                move |name: &str, value: f64| -> bool {
                    let mut model_borrow = m.borrow_mut();
                    let Some(robot) = model_borrow.as_mut() else {
                        return false;
                    };
                    let Some(&idx) = robot.joint_map.get(name) else {
                        return false;
                    };
                    robot.joints[idx].joint_damping = value;
                    true
                },
            );

            // Bulk setters: apply the same value to every non-fixed joint.
            // Returns the number of joints touched. Designed for tuning sweeps
            // where the user wants "set every leg motor to 0.1 damping" in
            // one call rather than naming each joint.
            let m = Rc::clone(&model);
            engine.register_fn("set_kp_all", move |kp: f64| -> i64 {
                let mut model_borrow = m.borrow_mut();
                let Some(robot) = model_borrow.as_mut() else {
                    return 0;
                };
                let mut n = 0i64;
                for j in robot.joints.iter_mut() {
                    if j.joint_type != "fixed" {
                        j.actuator_kp = kp;
                        n += 1;
                    }
                }
                n
            });

            let m = Rc::clone(&model);
            engine.register_fn("set_kv_all", move |kv: f64| -> i64 {
                let mut model_borrow = m.borrow_mut();
                let Some(robot) = model_borrow.as_mut() else {
                    return 0;
                };
                let mut n = 0i64;
                for j in robot.joints.iter_mut() {
                    if j.joint_type != "fixed" {
                        j.actuator_kv = kv;
                        n += 1;
                    }
                }
                n
            });

            let m = Rc::clone(&model);
            engine.register_fn("set_armature_all", move |value: f64| -> i64 {
                let mut model_borrow = m.borrow_mut();
                let Some(robot) = model_borrow.as_mut() else {
                    return 0;
                };
                let mut n = 0i64;
                for j in robot.joints.iter_mut() {
                    if j.joint_type != "fixed" {
                        j.armature = value;
                        n += 1;
                    }
                }
                n
            });

            let m = Rc::clone(&model);
            engine.register_fn("set_joint_damping_all", move |value: f64| -> i64 {
                let mut model_borrow = m.borrow_mut();
                let Some(robot) = model_borrow.as_mut() else {
                    return 0;
                };
                let mut n = 0i64;
                for j in robot.joints.iter_mut() {
                    if j.joint_type != "fixed" {
                        j.joint_damping = value;
                        n += 1;
                    }
                }
                n
            });

            let s = Rc::clone(&mujoco_sim);
            let m = Rc::clone(&model);
            engine.register_fn(
                "set_position_target",
                move |name: &str, target: f64| -> bool {
                    let mut sim_borrow = s.borrow_mut();
                    let model_borrow = m.borrow();
                    let (Some(sim), Some(robot)) =
                        (sim_borrow.as_mut(), model_borrow.as_ref())
                    else {
                        return false;
                    };
                    let Some(&idx) = robot.joint_map.get(name) else {
                        return false;
                    };
                    sim.set_position_target(idx, target);
                    true
                },
            );

            let s = Rc::clone(&mujoco_sim);
            let m = Rc::clone(&model);
            engine.register_fn(
                "set_velocity_target",
                move |name: &str, target: f64| -> bool {
                    let mut sim_borrow = s.borrow_mut();
                    let model_borrow = m.borrow();
                    let (Some(sim), Some(robot)) =
                        (sim_borrow.as_mut(), model_borrow.as_ref())
                    else {
                        return false;
                    };
                    let Some(&idx) = robot.joint_map.get(name) else {
                        return false;
                    };
                    sim.set_velocity_target(idx, target);
                    true
                },
            );

            let s = Rc::clone(&mujoco_sim);
            let m = Rc::clone(&model);
            engine.register_fn(
                "set_torque_target",
                move |name: &str, target: f64| -> bool {
                    let mut sim_borrow = s.borrow_mut();
                    let model_borrow = m.borrow();
                    let (Some(sim), Some(robot)) =
                        (sim_borrow.as_mut(), model_borrow.as_ref())
                    else {
                        return false;
                    };
                    let Some(&idx) = robot.joint_map.get(name) else {
                        return false;
                    };
                    sim.set_torque_target(idx, target);
                    true
                },
            );
        }

        // ── Quadruped gait API ──────────────────────────────────────────
        // The gait controller is independent of the MuJoCo feature gate
        // (it just produces joint targets); the actual sim hook lives in
        // ArticaraApp's step loop where the targets are forwarded to
        // mujoco_sim::set_position_target.
        {
            let g = Rc::clone(&gait_controller);
            let m = Rc::clone(&model);
            engine.register_fn("gait_setup", move || -> bool {
                let model_borrow = m.borrow();
                let Some(robot) = model_borrow.as_ref() else {
                    return false;
                };
                let foot_links = crate::gait::DEFAULT_FOOT_LINKS;
                let kin = match crate::gait::auto_detect_kinematics_config(
                    robot, &foot_links,
                ) {
                    Ok(k) => k,
                    Err(errs) => {
                        for (leg, msg) in errs {
                            log::warn!("gait_setup: {}: {msg}", leg.label());
                        }
                        return false;
                    }
                };
                let cfg = quadruped_gait::GaitConfig::trot();
                match crate::gait::GaitController::build(robot, kin, cfg, quadruped_gait::GaitMode::Champ) {
                    Ok(ctrl) => {
                        *g.borrow_mut() = Some(ctrl);
                        true
                    }
                    Err(e) => {
                        log::warn!("gait_setup build: {e}");
                        false
                    }
                }
            });

            let g = Rc::clone(&gait_controller);
            let m = Rc::clone(&model);
            engine.register_fn(
                "gait_setup_with_feet",
                move |fl: &str, fr: &str, rl: &str, rr: &str| -> bool {
                    let model_borrow = m.borrow();
                    let Some(robot) = model_borrow.as_ref() else {
                        return false;
                    };
                    let foot_links = [
                        (quadruped_gait::LegId::FL, fl),
                        (quadruped_gait::LegId::FR, fr),
                        (quadruped_gait::LegId::RL, rl),
                        (quadruped_gait::LegId::RR, rr),
                    ];
                    let kin = match crate::gait::auto_detect_kinematics_config(
                        robot, &foot_links,
                    ) {
                        Ok(k) => k,
                        Err(errs) => {
                            for (leg, msg) in errs {
                                log::warn!(
                                    "gait_setup_with_feet: {}: {msg}",
                                    leg.label(),
                                );
                            }
                            return false;
                        }
                    };
                    let cfg = quadruped_gait::GaitConfig::trot();
                    match crate::gait::GaitController::build(robot, kin, cfg, quadruped_gait::GaitMode::Champ) {
                        Ok(ctrl) => {
                            *g.borrow_mut() = Some(ctrl);
                            true
                        }
                        Err(e) => {
                            log::warn!("gait_setup_with_feet build: {e}");
                            false
                        }
                    }
                },
            );

            let g = Rc::clone(&gait_controller);
            engine.register_fn(
                "gait_set_velocity",
                move |vx: f64, vy: f64, wz: f64| -> bool {
                    let mut gb = g.borrow_mut();
                    let Some(ctrl) = gb.as_mut() else {
                        return false;
                    };
                    ctrl.set_velocity_cmd(quadruped_gait::VelocityCmd {
                        vx,
                        vy,
                        wz,
                    });
                    true
                },
            );

            let g = Rc::clone(&gait_controller);
            engine.register_fn("gait_start", move || -> bool {
                let mut gb = g.borrow_mut();
                let Some(ctrl) = gb.as_mut() else {
                    return false;
                };
                ctrl.enable();
                true
            });

            let g = Rc::clone(&gait_controller);
            engine.register_fn("gait_stop", move || -> bool {
                let mut gb = g.borrow_mut();
                let Some(ctrl) = gb.as_mut() else {
                    return false;
                };
                ctrl.disable();
                true
            });

            let g = Rc::clone(&gait_controller);
            engine.register_fn("gait_running", move || -> bool {
                g.borrow().as_ref().map(|c| c.is_enabled()).unwrap_or(false)
            });

            let g = Rc::clone(&gait_controller);
            engine.register_fn("gait_active", move || -> bool {
                g.borrow().is_some()
            });

            // Gait config tweaks. Each takes a single parameter and
            // returns true on success / false if no controller exists.
            let g = Rc::clone(&gait_controller);
            engine.register_fn("gait_set_cycle_period", move |s: f64| -> bool {
                let mut gb = g.borrow_mut();
                let Some(ctrl) = gb.as_mut() else { return false; };
                let mut cfg = ctrl.config().clone();
                cfg.cycle_period_s = s.max(0.05);
                ctrl.set_config(cfg);
                true
            });
            let g = Rc::clone(&gait_controller);
            engine.register_fn("gait_set_swing_height", move |m: f64| -> bool {
                let mut gb = g.borrow_mut();
                let Some(ctrl) = gb.as_mut() else { return false; };
                let mut cfg = ctrl.config().clone();
                cfg.swing_height_m = m.max(0.0);
                ctrl.set_config(cfg);
                true
            });
            let g = Rc::clone(&gait_controller);
            engine.register_fn("gait_set_duty", move |d: f64| -> bool {
                let mut gb = g.borrow_mut();
                let Some(ctrl) = gb.as_mut() else { return false; };
                let mut cfg = ctrl.config().clone();
                cfg.duty_factor = d.clamp(0.05, 0.95);
                ctrl.set_config(cfg);
                true
            });
            // Knee pattern shorthand: `<<` / `<>` / `><` / `>>`. Returns
            // true on success, false on bad string or no controller.
            let g = Rc::clone(&gait_controller);
            engine.register_fn("gait_set_knee_pattern", move |s: &str| -> bool {
                let Some(p) = quadruped_gait::KneePattern::from_label(s) else {
                    log::warn!(
                        "gait_set_knee_pattern: unknown pattern {s:?} \
                         (expected <<, <>, ><, or >>)"
                    );
                    return false;
                };
                let mut gb = g.borrow_mut();
                let Some(ctrl) = gb.as_mut() else { return false; };
                ctrl.set_knee_pattern(p);
                true
            });

            // Read back as the same shorthand string. Returns "" if no
            // controller — Rhai scripts can `if pattern != ""`.
            let g = Rc::clone(&gait_controller);
            engine.register_fn("gait_knee_pattern", move || -> String {
                let gb = g.borrow();
                gb.as_ref()
                    .map(|c| c.knee_pattern().label().to_string())
                    .unwrap_or_default()
            });

            let g = Rc::clone(&gait_controller);
            engine.register_fn("gait_set_max_step", move |m: f64| -> bool {
                let mut gb = g.borrow_mut();
                let Some(ctrl) = gb.as_mut() else { return false; };
                let mut cfg = ctrl.config().clone();
                cfg.max_step_length_m = m.max(0.0);
                ctrl.set_config(cfg);
                true
            });
        }

        engine
    }
}

/// Shared implementation for reduce_all_meshes (avoids code duplication).
fn reduce_all_meshes_impl(
    robot: &mut crate::robot::RobotModel,
    ratio: f64,
    method: misarta::decimate::DecimationMethod,
) -> i64 {
    let mut removed = 0i64;
    for link in &mut robot.links {
        for vis in &mut link.visuals {
            if let crate::robot::GeomData::Mesh { ref mut vertices, .. } = vis.geometry {
                let before = vertices.len() as i64 / 18;
                let mesh_data = misarta::mesh::MeshData::from_flat_vertices_f32(vertices);
                let reduced = mesh_data.decimate_with(ratio, method);
                *vertices = reduced.to_flat_vertices_f32();
                let after = reduced.num_triangles() as i64;
                removed += before - after;
            }
        }
        for col in &mut link.collisions {
            if let crate::robot::GeomData::Mesh { ref mut vertices, .. } = col.geometry {
                let before = vertices.len() as i64 / 18;
                let mesh_data = misarta::mesh::MeshData::from_flat_vertices_f32(vertices);
                let reduced = mesh_data.decimate_with(ratio, method);
                *vertices = reduced.to_flat_vertices_f32();
                let after = reduced.num_triangles() as i64;
                removed += before - after;
            }
        }
    }
    removed
}

/// Shared implementation for decompose_vhacd / decompose_spheres.
///
/// Replaces collision `ci` of the named link with multiple shapes.
/// Returns the number of shapes produced, or -1 on error.
fn decompose_collision_impl(
    robot: &mut crate::robot::RobotModel,
    link: &str,
    ci: usize,
    method: misarta::decompose::DecompositionMethod,
    max_count: Option<usize>,
) -> i64 {
    let Some(&li) = robot.link_map.get(link) else { return -1 };
    if ci >= robot.links[li].collisions.len() { return -1; }

    let col = &robot.links[li].collisions[ci];
    let origin = col.origin;

    let vertices = match &col.geometry {
        crate::robot::GeomData::Mesh { vertices, .. } => vertices.clone(),
        _ => return -1,
    };

    let mesh_data = misarta::mesh::MeshData::from_flat_vertices_f32(&vertices);

    let new_collisions: Vec<crate::robot::CollisionData> = match method {
        misarta::decompose::DecompositionMethod::Vhacd => {
            let params = misarta::decompose::VhacdParams {
                max_hulls: max_count.unwrap_or(16) as u32,
                ..Default::default()
            };
            let hulls = misarta::decompose::vhacd(&mesh_data, &params);
            hulls.iter().map(|h| {
                crate::robot::CollisionData {
                    origin,
                    geometry: crate::robot::GeomData::Mesh {
                        vertices: h.to_flat_vertices_f32(),
                        filename: None,
                        scale: None,
                    },
                }
            }).collect()
        }
        misarta::decompose::DecompositionMethod::SphereTree => {
            let params = misarta::decompose::SphereTreeParams {
                max_spheres: max_count.unwrap_or(16),
                ..Default::default()
            };
            let spheres = misarta::decompose::sphere_tree(&mesh_data, &params);
            spheres.iter().map(|s| {
                use nalgebra as na;
                let t = na::Translation3::new(s.center.x as f32, s.center.y as f32, s.center.z as f32);
                let sphere_origin = origin * na::Isometry3::from_parts(t, na::UnitQuaternion::identity());
                crate::robot::CollisionData {
                    origin: sphere_origin,
                    geometry: crate::robot::GeomData::Sphere { radius: s.radius as f32 },
                }
            }).collect()
        }
        misarta::decompose::DecompositionMethod::PrimitiveFit => {
            use nalgebra as na;
            let params = misarta::decompose::VhacdParams::default();
            let prims = misarta::decompose::primitive_fit(&mesh_data, &params);
            prims.iter().map(|p| {
                let t = na::Translation3::new(
                    p.center.x as f32,
                    p.center.y as f32,
                    p.center.z as f32,
                );
                let r = na::UnitQuaternion::new_normalize(na::Quaternion::new(
                    p.rotation.w as f32,
                    p.rotation.i as f32,
                    p.rotation.j as f32,
                    p.rotation.k as f32,
                ));
                let prim_origin = origin * na::Isometry3::from_parts(t, r);
                let geometry = match p.kind {
                    misarta::decompose::PrimitiveKind::Box { hx, hy, hz } => {
                        crate::robot::GeomData::Box {
                            hx: hx as f32,
                            hy: hy as f32,
                            hz: hz as f32,
                        }
                    }
                    misarta::decompose::PrimitiveKind::Cylinder { radius, half_length } => {
                        crate::robot::GeomData::Cylinder {
                            radius: radius as f32,
                            half_length: half_length as f32,
                        }
                    }
                    misarta::decompose::PrimitiveKind::Sphere { radius } => {
                        crate::robot::GeomData::Sphere { radius: radius as f32 }
                    }
                };
                crate::robot::CollisionData {
                    origin: prim_origin,
                    geometry,
                }
            }).collect()
        }
        misarta::decompose::DecompositionMethod::PrimitiveFitDirect => {
            use nalgebra as na;
            let p = misarta::decompose::primitive_fit_direct(&mesh_data);
            let t = na::Translation3::new(
                p.center.x as f32,
                p.center.y as f32,
                p.center.z as f32,
            );
            let r = na::UnitQuaternion::new_normalize(na::Quaternion::new(
                p.rotation.w as f32,
                p.rotation.i as f32,
                p.rotation.j as f32,
                p.rotation.k as f32,
            ));
            let prim_origin = origin * na::Isometry3::from_parts(t, r);
            let geometry = match p.kind {
                misarta::decompose::PrimitiveKind::Box { hx, hy, hz } => {
                    crate::robot::GeomData::Box {
                        hx: hx as f32,
                        hy: hy as f32,
                        hz: hz as f32,
                    }
                }
                misarta::decompose::PrimitiveKind::Cylinder { radius, half_length } => {
                    crate::robot::GeomData::Cylinder {
                        radius: radius as f32,
                        half_length: half_length as f32,
                    }
                }
                misarta::decompose::PrimitiveKind::Sphere { radius } => {
                    crate::robot::GeomData::Sphere { radius: radius as f32 }
                }
            };
            vec![crate::robot::CollisionData {
                origin: prim_origin,
                geometry,
            }]
        }
    };

    if new_collisions.is_empty() {
        return 0;
    }

    let n = new_collisions.len() as i64;
    robot.links[li].collisions.remove(ci);
    for (i, c) in new_collisions.into_iter().enumerate() {
        robot.links[li].collisions.insert(ci + i, c);
    }
    n
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
            "decompose_vhacd", "decompose_spheres", "decompose_primitive",
            "mj_active", "mj_start", "mj_stop", "mj_step", "mj_step_back",
            "mj_timestep", "mj_history_len", "mj_trace_len", "mj_set_trace_max",
            "mj_gravity_compensation", "save_peaks_csv",
            "gait_setup", "gait_setup_with_feet", "gait_set_velocity",
            "gait_start", "gait_stop", "gait_running", "gait_active",
            "gait_set_cycle_period", "gait_set_swing_height",
            "gait_set_duty", "gait_set_max_step",
            "gait_set_knee_pattern", "gait_knee_pattern",
            "mj_async_step_seconds", "mj_async_step_frames",
            "mj_async_set_position_target", "mj_async_print",
            "mj_async_save_csv", "mj_async_pending", "mj_async_clear",
            "play_pose", "transition_in_progress", "transition_progress",
            "play_sequence", "sequence_in_progress", "sequence_progress",
            "apply_force", "cancel_force",
            "reset_peaks", "peak_torque", "peak_velocity", "peaks",
            "set_actuator_mode", "set_actuator_mode_all",
            "set_kp", "set_kv", "set_armature", "set_joint_damping",
            "set_kp_all", "set_kv_all", "set_armature_all", "set_joint_damping_all",
            "set_position_target", "set_velocity_target", "set_torque_target",
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

    /// Compile-only check for the bundled example scripts so that future
    /// edits to the API don't silently rot the demos. Each script must
    /// parse with the production engine; we don't *run* them since that
    /// would need a live MuJoCo sim.
    #[test]
    fn example_scripts_parse() {
        let scripts = [
            "scripts/example_jump.rhai",
            "scripts/example_jump_async.rhai",
            "scripts/verify_jump_tuning.rhai",
            "scripts/walk_demo.rhai",
        ];
        for rel in scripts {
            let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join(rel);
            let src = std::fs::read_to_string(&path)
                .expect(&format!("read {}", path.display()));
            let mut eng = ModelScriptEngine::new();
            eng.compile(&src)
                .unwrap_or_else(|e| panic!("{} failed to compile: {e}", rel));
        }
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
