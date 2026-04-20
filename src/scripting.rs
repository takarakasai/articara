//! Rhai-based scripting engine for interactive robot control.
//!
//! Provides a sandboxed scripting environment where control engineers can
//! write Python-like logic (via Rhai syntax for now) that is evaluated every
//! simulation cycle.  The engine exposes read-only sensor data and write
//! accessors for torque overrides, enabling rapid prototyping of reflexes
//! and controllers without recompilation.
//!
//! # Architecture
//!
//! ```text
//! ┌────────────────────────────┐
//! │   REPL / GUI editor        │
//! │   (Rhai script text)       │
//! └───────────┬────────────────┘
//!             │  compile (off-loop)
//!             ▼
//! ┌────────────────────────────┐
//! │  Compiled AST (Arc<AST>)   │──► atomic swap
//! └───────────┬────────────────┘
//!             │  eval (in-loop, every cycle)
//!             ▼
//! ┌────────────────────────────┐
//! │  Rust control loop         │
//! │  read sensors → run AST →  │
//! │  apply torque overrides    │
//! └────────────────────────────┘
//! ```
//!
//! # Safety
//!
//! - `max_operations` caps execution to prevent infinite loops.
//! - `max_call_levels` prevents stack overflow.
//! - No file I/O, no network — Rhai is sandboxed by design.

use rhai::{Dynamic, Engine, AST, Scope, Map};
use std::collections::HashMap;
use std::sync::{Arc, RwLock};

// ─────────────────────────────────────────────────────────────────────────
//  Shared control context — exchanged between control loop and script
// ─────────────────────────────────────────────────────────────────────────

/// Data snapshot available to the running script each cycle.
///
/// Populated by the control loop *before* script evaluation, consumed
/// read-only by the script.
#[derive(Clone, Debug, Default)]
pub struct ScriptInputs {
    /// Current joint positions (rad or m), keyed by joint index.
    pub joint_positions: HashMap<usize, f64>,
    /// Current joint velocities (rad/s or m/s), keyed by joint index.
    pub joint_velocities: HashMap<usize, f64>,
    /// Named scalar sensor readings (e.g. "foot_force_left" → 42.0).
    pub sensors: HashMap<String, f64>,
    /// Simulation time in seconds.
    pub time: f64,
    /// Time-step of the current cycle (s).
    pub dt: f64,
}

/// Outputs written by the script, consumed by the control loop *after*
/// evaluation.
#[derive(Clone, Debug, Default)]
pub struct ScriptOutputs {
    /// Torque overrides: joint_index → desired torque.
    /// Only joints present here will have their torque overridden.
    pub torque_overrides: HashMap<usize, f64>,
    /// Generic key-value outputs for logging / GUI display.
    pub debug_values: HashMap<String, f64>,
}

// ─────────────────────────────────────────────────────────────────────────
//  Script engine
// ─────────────────────────────────────────────────────────────────────────

/// The scripting engine that compiles and evaluates user scripts.
pub struct ScriptEngine {
    engine: Engine,
    /// The currently active compiled AST (hot-swappable).
    ast: Option<Arc<AST>>,
    /// Persistent scope across evaluations (variables survive between cycles).
    scope: Scope<'static>,
    /// Shared outputs written by the script.
    outputs: Arc<RwLock<ScriptOutputs>>,
    /// Last compilation / evaluation error message (for UI display).
    last_error: Option<String>,
}

impl ScriptEngine {
    /// Create a new scripting engine with control-domain built-in functions.
    pub fn new() -> Self {
        let outputs = Arc::new(RwLock::new(ScriptOutputs::default()));
        let engine = Self::build_engine(Arc::clone(&outputs));
        Self {
            engine,
            ast: None,
            scope: Scope::new(),
            outputs,
            last_error: None,
        }
    }

    /// Build the Rhai engine with safety limits and control-domain functions.
    fn build_engine(outputs: Arc<RwLock<ScriptOutputs>>) -> Engine {
        let mut engine = Engine::new();

        // ── Safety limits ──
        engine.set_max_operations(10_000); // prevent infinite loops
        engine.set_max_call_levels(16);    // prevent deep recursion
        engine.set_max_expr_depths(32, 32);

        // ── Utility math functions ──
        engine.register_fn("abs", |x: f64| -> f64 { x.abs() });
        engine.register_fn("sqrt", |x: f64| -> f64 { x.sqrt() });
        engine.register_fn("sin", |x: f64| -> f64 { x.sin() });
        engine.register_fn("cos", |x: f64| -> f64 { x.cos() });
        engine.register_fn("atan2", |y: f64, x: f64| -> f64 { y.atan2(x) });
        engine.register_fn("min", |a: f64, b: f64| -> f64 { a.min(b) });
        engine.register_fn("max", |a: f64, b: f64| -> f64 { a.max(b) });
        engine.register_fn("clamp", |x: f64, lo: f64, hi: f64| -> f64 {
            x.clamp(lo, hi)
        });
        engine.register_fn("sign", |x: f64| -> f64 {
            if x > 0.0 { 1.0 } else if x < 0.0 { -1.0 } else { 0.0 }
        });

        // ── Constants ──
        engine.register_fn("PI", || -> f64 { std::f64::consts::PI });

        // ── Output: set_torque(joint_index, torque_value) ──
        let out = Arc::clone(&outputs);
        engine.register_fn("set_torque", move |joint_idx: i64, torque: f64| {
            if let Ok(mut o) = out.write() {
                o.torque_overrides.insert(joint_idx as usize, torque);
            }
        });

        // ── Output: debug(name, value) — publish a named scalar for the UI ──
        let out = Arc::clone(&outputs);
        engine.register_fn("debug_val", move |name: &str, value: f64| {
            if let Ok(mut o) = out.write() {
                o.debug_values.insert(name.to_string(), value);
            }
        });

        // ── Output: print override (log::info) ──
        engine.on_print(|s| {
            log::info!("[script] {}", s);
        });

        engine
    }

    /// Compile a script string into an AST.
    ///
    /// On success the new AST is stored and will be used from the next
    /// `eval()` call.  On failure the previous AST remains active and the
    /// error is stored in `last_error`.
    pub fn compile(&mut self, source: &str) -> Result<(), String> {
        match self.engine.compile(source) {
            Ok(ast) => {
                self.ast = Some(Arc::new(ast));
                self.last_error = None;
                log::info!("Script compiled successfully ({} bytes)", source.len());
                Ok(())
            }
            Err(e) => {
                let msg = format!("Compile error: {e}");
                log::warn!("{msg}");
                self.last_error = Some(msg.clone());
                Err(msg)
            }
        }
    }

    /// Evaluate the compiled script with the given inputs.
    ///
    /// Call this once per control cycle.  The function:
    /// 1. Clears previous outputs.
    /// 2. Pushes sensor/state data into the Rhai scope as variables.
    /// 3. Evaluates the AST.
    /// 4. Returns the script's outputs (torque overrides, debug values).
    pub fn eval(&mut self, inputs: &ScriptInputs) -> ScriptOutputs {
        // Clear previous outputs
        if let Ok(mut o) = self.outputs.write() {
            *o = ScriptOutputs::default();
        }

        let ast = match &self.ast {
            Some(a) => Arc::clone(a),
            None => return ScriptOutputs::default(),
        };

        // Push inputs into scope
        self.scope.set_or_push("t", inputs.time);
        self.scope.set_or_push("dt", inputs.dt);

        // Joint positions as a Map: index(string) → value
        let mut q_map = Map::new();
        for (&idx, &val) in &inputs.joint_positions {
            q_map.insert(idx.to_string().into(), Dynamic::from_float(val));
        }
        self.scope.set_or_push("q", q_map);

        // Joint velocities as a Map
        let mut qd_map = Map::new();
        for (&idx, &val) in &inputs.joint_velocities {
            qd_map.insert(idx.to_string().into(), Dynamic::from_float(val));
        }
        self.scope.set_or_push("qd", qd_map);

        // Sensors as a Map: name → value
        let mut sensor_map = Map::new();
        for (name, &val) in &inputs.sensors {
            sensor_map.insert(name.clone().into(), Dynamic::from_float(val));
        }
        self.scope.set_or_push("sensor", sensor_map);

        // Evaluate
        match self.engine.eval_ast_with_scope::<Dynamic>(&mut self.scope, &ast) {
            Ok(_) => {
                self.last_error = None;
            }
            Err(e) => {
                let msg = format!("Runtime error: {e}");
                // Only log once per distinct error to avoid spam
                if self.last_error.as_deref() != Some(&msg) {
                    log::warn!("[script] {msg}");
                }
                self.last_error = Some(msg);
            }
        }

        // Return outputs
        self.outputs.read().map(|o| o.clone()).unwrap_or_default()
    }

    /// Clear the active script and reset the scope.
    pub fn clear(&mut self) {
        self.ast = None;
        self.scope.clear();
        self.last_error = None;
        if let Ok(mut o) = self.outputs.write() {
            *o = ScriptOutputs::default();
        }
        log::info!("Script cleared");
    }

    /// Returns the last error message, if any.
    pub fn last_error(&self) -> Option<&str> {
        self.last_error.as_deref()
    }

    /// Returns `true` if a compiled script is loaded.
    pub fn has_script(&self) -> bool {
        self.ast.is_some()
    }

    /// Reset the persistent scope (clear all user variables) without
    /// removing the compiled AST.
    pub fn reset_scope(&mut self) {
        self.scope.clear();
    }
}

impl Default for ScriptEngine {
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

    #[test]
    fn test_compile_and_eval_basic() {
        let mut engine = ScriptEngine::new();
        engine.compile(r#"
            let k = 0.5;
            let threshold = 30.0;
            let f = sensor["foot_force"];
            if f > threshold {
                let tau = -k * (f - threshold);
                set_torque(2, tau);
            }
        "#).unwrap();

        let mut inputs = ScriptInputs::default();
        inputs.sensors.insert("foot_force".to_string(), 50.0);
        inputs.dt = 0.001;

        let outputs = engine.eval(&inputs);
        // f=50 > threshold=30, tau = -0.5*(50-30) = -10.0
        assert!((outputs.torque_overrides[&2] - (-10.0)).abs() < 1e-9);
    }

    #[test]
    fn test_compile_error() {
        let mut engine = ScriptEngine::new();
        let result = engine.compile("this is not valid {{{");
        assert!(result.is_err());
        assert!(engine.last_error().is_some());
    }

    #[test]
    fn test_no_script_returns_empty() {
        let mut engine = ScriptEngine::new();
        let outputs = engine.eval(&ScriptInputs::default());
        assert!(outputs.torque_overrides.is_empty());
    }

    #[test]
    fn test_infinite_loop_protection() {
        let mut engine = ScriptEngine::new();
        engine.compile("loop { }").unwrap();
        let outputs = engine.eval(&ScriptInputs::default());
        // Should not hang — max_operations kicks in
        assert!(engine.last_error().is_some());
        assert!(outputs.torque_overrides.is_empty());
    }

    #[test]
    fn test_math_functions() {
        let mut engine = ScriptEngine::new();
        engine.compile(r#"
            debug_val("sin_val", sin(0.0));
            debug_val("abs_val", abs(-5.0));
            debug_val("clamp_val", clamp(100.0, -1.0, 1.0));
        "#).unwrap();

        let outputs = engine.eval(&ScriptInputs::default());
        assert!((outputs.debug_values["sin_val"]).abs() < 1e-9);
        assert!((outputs.debug_values["abs_val"] - 5.0).abs() < 1e-9);
        assert!((outputs.debug_values["clamp_val"] - 1.0).abs() < 1e-9);
    }

    #[test]
    fn test_persistent_scope() {
        let mut engine = ScriptEngine::new();

        // First eval: set a variable via scope directly
        engine.scope.set_or_push("counter", 0 as i64);

        // Compile a script that increments the counter
        engine.compile(r#"
            counter += 1;
            debug_val("counter", counter.to_float());
        "#).unwrap();

        // First eval: counter goes from 0 → 1
        let outputs = engine.eval(&ScriptInputs::default());
        assert!((outputs.debug_values["counter"] - 1.0).abs() < 1e-9);

        // Second eval: counter goes from 1 → 2
        let outputs = engine.eval(&ScriptInputs::default());
        assert!((outputs.debug_values["counter"] - 2.0).abs() < 1e-9);
    }

    #[test]
    fn test_joint_pos_vel_access() {
        let mut engine = ScriptEngine::new();
        engine.compile(r#"
            let pos = q["3"];
            let vel = qd["3"];
            debug_val("pos", pos);
            debug_val("vel", vel);
        "#).unwrap();

        let mut inputs = ScriptInputs::default();
        inputs.joint_positions.insert(3, 1.57);
        inputs.joint_velocities.insert(3, -0.5);

        let outputs = engine.eval(&inputs);
        assert!((outputs.debug_values["pos"] - 1.57).abs() < 1e-9);
        assert!((outputs.debug_values["vel"] - (-0.5)).abs() < 1e-9);
    }

    #[test]
    fn test_debug_val_output() {
        let mut engine = ScriptEngine::new();
        engine.compile(r#"
            debug_val("my_metric", 42.0);
        "#).unwrap();

        let outputs = engine.eval(&ScriptInputs::default());
        assert!((outputs.debug_values["my_metric"] - 42.0).abs() < 1e-9);
    }

    #[test]
    fn test_clear() {
        let mut engine = ScriptEngine::new();
        engine.compile("set_torque(0, 99.0);").unwrap();
        assert!(engine.has_script());
        engine.clear();
        assert!(!engine.has_script());
        let outputs = engine.eval(&ScriptInputs::default());
        assert!(outputs.torque_overrides.is_empty());
    }
}
