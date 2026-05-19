//! MuJoCo physics simulation integration.

use std::collections::VecDeque;
use std::path::PathBuf;
use std::sync::Arc;

use mujoco::prelude::{MjData, MjModel, load_all_plugin_libraries};
use nalgebra as na;

use crate::mjcf::MjcfExportOptions;
use crate::rbd::model::ActuatorMode;
use crate::robot::RobotModel;

pub use misarta::trajectory::InterpolationKind;

/// One MuJoCo physics tick worth of state, captured *before* `data.step()` was
/// called. Replaying it via [`MujocoSim::step_back_frames`] restores the sim to
/// that pre-step state.
#[derive(Clone)]
struct FrameSnapshot {
    qpos: Vec<f64>,
    qvel: Vec<f64>,
    time: f64,
}

/// Active MuJoCo simulation instance.
pub struct MujocoSim {
    model: Arc<MjModel>,
    data: MjData<Arc<MjModel>>,
    time_accumulator: f64,
    /// Robot pose captured at sim start, restored on Stop.
    saved_base_transform: na::Isometry3<f64>,
    saved_joint_positions: Vec<f64>,
    /// Ring buffer of pre-step snapshots, used for backward frame stepping.
    history: VecDeque<FrameSnapshot>,
    /// Maximum number of snapshots to retain (older entries are discarded).
    history_max: usize,
    /// Per-joint position target (used by Position-mode controller). Indexed
    /// by joint index in the [`RobotModel`]. Initialised to the robot's pose
    /// at sim start so the robot holds that pose against gravity.
    position_targets: Vec<f64>,
    /// Per-joint trajectory velocity feedforward q̇* used by the Position-mode
    /// controller. Updated each step from the active transition / sequence's
    /// derivative; held at zero when no trajectory is running so the
    /// controller naturally damps to rest. Without this, the Kv term would
    /// fight any deliberate motion (`Kv·(0 − q̇)` brakes against the target),
    /// causing high-amplitude torque oscillation during fast moves like jumps.
    position_target_velocities: Vec<f64>,
    /// Per-joint trajectory acceleration feedforward q̈* used by the
    /// `ComputedTorque` mode. Updated each step from the trajectory's
    /// second derivative; zero when no trajectory is running. Multiplied by
    /// the inertia matrix `M(q)` to give the inverse-dynamics torque
    /// component that drives motion at the commanded rate.
    position_target_accelerations: Vec<f64>,
    /// Per-joint velocity target (used by Velocity-mode controller).
    velocity_targets: Vec<f64>,
    /// Per-joint direct torque command (used by Torque-mode controller).
    torque_targets: Vec<f64>,
    /// Per-joint feedforward torque added on top of the PD output for
    /// Position and ComputedTorque modes (used by the WBC layer to inject
    /// `τ = -J^T · f_GRF` from the SRBD MPC). Held at zero unless the
    /// caller updates it each tick — auto-zeroed by `clear_torque_feedforward`
    /// when the gait controller is disabled, so a stale value can't keep
    /// driving the leg after the user stops walking.
    position_target_torque_ff: Vec<f64>,
    /// Exponentially-weighted moving average of the realised physics-
    /// step rate (MuJoCo sub-steps per wall-clock second). Updated each
    /// [`Self::step`] / [`Self::step_n_frames`] call from the elapsed
    /// time and number of inner sub-steps. Read by the viewport
    /// overlay to display the realtime achievement ratio = `EMA / (1 / timestep)`,
    /// which sits in [0, 1] when keeping up at speed=1 and drops below
    /// 1 when the controller / WBC can't sustain the 500 Hz target.
    realized_step_rate_hz_ema: f64,
    /// EMA smoothing factor — higher = more responsive to bursts,
    /// lower = smoother. 0.1 ≈ 10-frame time constant.
    realized_step_rate_alpha: f64,
    /// Full per-joint torque override produced by the Hierarchical WBC.
    /// When `Some`, [`Self::apply_controller`] writes these torques
    /// **directly** to each motor's `ctrl` slot, bypassing per-joint
    /// `ActuatorMode` logic (Position-PD, gravity FF, etc.). Length
    /// matches `robot.joints.len()`; entries for fixed joints are
    /// ignored. Cleared via [`Self::clear_wbc_torques`].
    wbc_torque_override: Option<Vec<f64>>,
    /// Active pose-to-pose transition (drives `position_targets` per step).
    /// `None` when the controller should hold the current target.
    transition: Option<ActiveTransition>,
    /// Active multi-step sequence playback (chained pose transitions).
    /// Mutually exclusive with `transition` — starting one cancels the other.
    sequence: Option<ActiveSequence>,
    /// Active external force/torque pulses applied to specific bodies.
    /// Each entry is decremented per physics tick; expired pulses are removed
    /// and the body's `xfrc_applied` slot is cleared.
    force_pulses: Vec<ExternalForcePulse>,
    /// Running peak |τ| and |q̇| per joint (indexed by RobotModel joint idx).
    /// Reset on construction and at each `start_transition` /
    /// `apply_external_force` so each Play / pulse gets a fresh observation
    /// window. Use [`Self::reset_peaks`] to clear manually.
    peaks: Vec<JointPeak>,
    /// Latest commanded torques per joint, populated by `apply_controller`
    /// each tick and read by `record_trace` to fill the τ column of the
    /// time-series plot. Indexed by RobotModel joint index; non-controlled
    /// (fixed) joints stay at 0.
    last_tau: Vec<f64>,
    /// Recent (q, q̇, τ) samples per joint for the timeline plot in the UI.
    /// Bounded ring buffer; the newest entry is at the back. Reset together
    /// with `peaks` so the chart shows only the response to the latest
    /// command. See [`Self::trace`] for the access API.
    trace: VecDeque<TraceFrame>,
    /// Cap on trace length (`~10s` of history at the default 2 ms timestep).
    trace_max: usize,
    /// When `true`, [`Self::apply_controller`] adds a feedforward gravity-
    /// compensation torque (computed via misarta's RNEA) on top of the PD
    /// command in Position and Velocity modes. Off by default to preserve the
    /// existing controller behaviour for users who haven't opted in. The
    /// equivalent UI knob lives in dynamics_panel "Sim toggles".
    gravity_compensation: bool,
    /// Pending operations queued by Rhai's `mj_async_*` family. Consumed
    /// gradually by the UI animation loop so a script's intended timeline
    /// plays out frame-by-frame in the viewport rather than being collapsed
    /// into one synchronous batch. See [`AsyncSimOp`].
    async_queue: VecDeque<AsyncSimOp>,
}

/// One op sitting on the [`MujocoSim`] async queue. Step ops carry their own
/// remaining-frame counter so the host can chip away at them across UI ticks
/// without losing track. All other ops are point-in-time and execute exactly
/// once when popped.
#[derive(Clone, Debug)]
pub enum AsyncSimOp {
    /// Advance the sim by this many physics frames, paced by the host's
    /// wall-clock × speed slider. Decrements as frames are consumed.
    StepFrames(u32),
    /// Equivalent of `set_position_target(joint_idx, q)` deferred to the
    /// timeline point at which it's popped.
    SetPositionTarget(usize, f64),
    /// Append a system line to the script console output.
    Print(String),
    /// Write the current trace as CSV at the timeline point it's popped.
    SaveCsv(std::path::PathBuf),
    /// Set the active gait controller's `(vx, vy, wz)` velocity command.
    /// Lets a script schedule cmd changes mid-timeline (e.g. forward
    /// for 5 s, then lateral for 5 s, then yaw for 5 s — the
    /// `walk_3axis_demo.rhai` benchmark reproducer).
    /// All three components are body-frame; semantically identical to
    /// the synchronous `gait_set_velocity(vx, vy, wz)` script call.
    SetGaitVelocity(f64, f64, f64),
}

/// One sample in the time-series ring buffer.
///
/// `q`, `qvel`, and `tau` are aligned with the [`RobotModel`] joint order,
/// padded to the same length. `time` is MuJoCo's simulation clock at the
/// instant the sample was captured (i.e. *after* the physics step).
#[derive(Clone, Debug)]
pub struct TraceFrame {
    pub time: f64,
    pub q: Vec<f64>,
    pub qvel: Vec<f64>,
    pub tau: Vec<f64>,
    /// World-frame position `[x, y, z]` of the model's root link, captured
    /// straight from MuJoCo's `xpos`. `None` when the root link is absent
    /// from the compiled MJCF (e.g. fixed-base manipulators without a free
    /// joint). Diagnostic-only — exported via [`save_peaks_csv`] so GUI
    /// traces can be compared against headless-test body trajectories.
    pub base_pos: Option<[f64; 3]>,
    /// World-frame orientation quaternion `[w, x, y, z]` of the root link.
    /// Same provenance and same `None` condition as [`Self::base_pos`].
    pub base_quat: Option<[f64; 4]>,
}

/// Running peak observations for a single joint, accumulated tick-by-tick.
///
/// `tau` carries the applied generalised effort (N·m for revolute, N for
/// prismatic). `qvel` carries the absolute velocity (rad/s or m/s). All
/// values are non-negative magnitudes — the sample with the largest absolute
/// magnitude is kept along with its signed origin in `tau_signed` /
/// `qvel_signed` so the UI can show direction.
#[derive(Clone, Debug, Default)]
pub struct JointPeak {
    pub tau_abs: f64,
    pub tau_signed: f64,
    pub qvel_abs: f64,
    pub qvel_signed: f64,
}

/// Smooth pose transition currently being played out.
struct ActiveTransition {
    traj: misarta::trajectory::PoseTransition<f64>,
    /// Sim-time elapsed since the transition started.
    elapsed: f64,
}

/// Multi-step sequence playback. Each tick we evaluate the underlying
/// [`misarta::trajectory::KeyframeAnimation`] at the current `elapsed`
/// time and copy the result into `position_targets`. The sequence is
/// dropped once `is_done(elapsed)` returns true.
struct ActiveSequence {
    anim: misarta::trajectory::KeyframeAnimation<f64>,
    elapsed: f64,
    /// Optional human-readable name carried for status messages.
    name: String,
}

/// One contact between geoms reported by MuJoCo, in world frame.
///
/// `pos` is the contact point. `force_mag` is the magnitude of the linear
/// contact force (N) and `force_world` is the linear part rotated into world
/// coordinates so the UI can draw it directly. `body1` / `body2` are the
/// names of the bodies involved (resolved via `geom_bodyid` + body
/// `id_to_name`); they're useful for distinguishing self-collisions
/// (body1 == one of the robot's links and body2 == another) from external
/// contacts like ground.
#[derive(Clone, Debug)]
pub struct ContactInfo {
    pub pos: [f64; 3],
    /// Magnitude of the linear contact force (N).
    pub force_mag: f64,
    /// Linear contact force expressed in world coordinates (N).
    pub force_world: [f64; 3],
    /// Name of the first contact body (lowercase `body1` in MJCF). Empty
    /// string if the body has no name (e.g. world body 0).
    pub body1: String,
    /// Name of the second contact body.
    pub body2: String,
}

impl ContactInfo {
    /// `true` when neither body is the world body (== both are robot links).
    /// Lets the renderer distinguish self-collision from ground/world
    /// contacts so the user can spot unintended interpenetrations.
    pub fn is_self_collision(&self) -> bool {
        !self.body1.is_empty() && !self.body2.is_empty()
    }
}

/// One IMU sensor's readings at a sim tick. `accel` units: m/s² (proper
/// acceleration; static = +g along the up axis when the IMU is level).
/// `gyro` units: rad/s.
#[derive(Clone, Debug)]
pub struct ImuReading {
    pub name: String,
    pub link: String,
    pub accel: [f64; 3],
    pub gyro: [f64; 3],
    /// MuJoCo simulation time at which the reading was captured.
    pub sim_time: f64,
}

/// Time-bounded external wrench applied to a single body.
///
/// MuJoCo's `xfrc_applied[6]` (force [N] + torque [N·m] in the world frame)
/// is written each tick while the pulse is active and zeroed when it
/// expires, so the disturbance has a clean rectangular envelope.
#[derive(Clone, Debug)]
pub struct ExternalForcePulse {
    pub link_name: String,
    /// Force in world frame (N).
    pub force: [f64; 3],
    /// Torque in world frame (N·m).
    pub torque: [f64; 3],
    /// Total duration of the pulse (s).
    pub duration: f64,
    /// Sim-time elapsed since the pulse started.
    pub elapsed: f64,
}

/// Finds the `bin/mujoco_plugin` directory inside the MuJoCo installation
/// **that matches the currently-linked runtime version**.
///
/// Loading plugins from a different MuJoCo version than the linked
/// runtime is fatal: MuJoCo 3.8.0 has the OBJ / STL decoders built in,
/// so loading the 3.6.0 `libobj_decoder.so` / `libstl_decoder.so`
/// plugins triggers `mj_loadResource: ERROR: resource decoder
/// 'model/obj' is already registered`, which `mju_error()` turns into
/// process termination. So we have to be picky about the directory we
/// hand to `load_all_plugin_libraries`.
///
/// Search order:
/// 1. `MUJOCO_DYNAMIC_LINK_DIR` (the env var `mujoco-rs` uses for
///    linking) → its parent's `bin/mujoco_plugin`. Most reliable —
///    plugin dir is guaranteed to match the linked runtime.
/// 2. `MUJOCO_DOWNLOAD_DIR` (or `$HOME/.mujoco`) + `mujoco-{version}/
///    bin/mujoco_plugin`, where `{version}` comes from the live
///    [`crate::mujoco_version`] cache. Lets us pick the right install
///    when only the runtime path is set.
/// 3. Fallback: first `mujoco-*` directory found under the base. Kept
///    for backward compatibility with existing single-version installs;
///    logs a warning when the chosen dir might not match the runtime.
fn find_plugin_dir() -> Option<PathBuf> {
    // ── 1. Derive directly from MUJOCO_DYNAMIC_LINK_DIR ──────────────
    // mujoco-rs reads this at build time to pick the libmujoco to link
    // against, so its parent directory is the install root we want
    // plugins from. This path is identical to the linked runtime by
    // construction.
    if let Ok(lib_dir) = std::env::var("MUJOCO_DYNAMIC_LINK_DIR") {
        let lib_path = PathBuf::from(&lib_dir);
        if let Some(install_root) = lib_path.parent() {
            let plugin_dir = install_root.join("bin").join("mujoco_plugin");
            if plugin_dir.exists() {
                return Some(plugin_dir);
            }
        }
    }

    // ── 2. Match against the runtime version reported by mj_version ──
    let base = std::env::var("MUJOCO_DOWNLOAD_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|_| {
            std::env::var("HOME")
                .map(|h| PathBuf::from(h).join(".mujoco"))
                .unwrap_or_default()
        });

    if let Some(crate::mujoco_version::CheckResult::Compatible(v)) =
        crate::mujoco_version::cached()
    {
        let exact = base
            .join(format!("mujoco-{v}"))
            .join("bin")
            .join("mujoco_plugin");
        if exact.exists() {
            return Some(exact);
        }
    }

    // ── 3. Last-resort fallback: first mujoco-* dir under base ──────
    // Warn so the user can spot version mismatches when multiple
    // installs are present.
    let chosen = std::fs::read_dir(&base).ok()?.find_map(|entry| {
        let entry = entry.ok()?;
        let name = entry.file_name();
        if name.to_string_lossy().starts_with("mujoco-") {
            let plugin_dir = entry.path().join("bin").join("mujoco_plugin");
            plugin_dir.exists().then_some(plugin_dir)
        } else {
            None
        }
    });
    if let Some(ref dir) = chosen {
        log::warn!(
            "find_plugin_dir: falling back to first mujoco-* directory ({}). \
             Set MUJOCO_DYNAMIC_LINK_DIR to disambiguate when multiple \
             MuJoCo versions are installed under {} — loading plugins from \
             a different version than the linked runtime can crash MuJoCo.",
            dir.display(),
            base.display(),
        );
    }
    chosen
}

impl MujocoSim {
    /// Create a new MuJoCo simulation instance from the current RobotModel
    /// using the supplied MJCF export options.
    ///
    /// `opts.add_actuators` is forced to `true` — every interactive sim needs
    /// per-joint actuators so the user-set initial pose can be held against
    /// gravity. The actuator type for each joint is selected by its
    /// `actuator_mode` field (Position / Velocity / Torque).
    pub fn new(robot: &RobotModel, mut opts: MjcfExportOptions) -> Result<Self, String> {
        // Pre-flight: surface a MuJoCo runtime / FFI version mismatch as
        // a clean error rather than letting `MjModel::from_xml_string`
        // panic deep inside `mujoco-rs::util::assert_mujoco_version`.
        if let Some(crate::mujoco_version::CheckResult::Mismatch { .. }) =
            crate::mujoco_version::cached()
        {
            return Err(crate::mujoco_version::cached().unwrap().diagnostic());
        }

        opts.add_actuators = true;
        // Snapshot the limit-baking flag before `opts` is moved into the
        // MJCF exporter below — we need it again when seeding qpos.
        let bake_joint_position_limits = opts.bake_joint_position_limits;

        // Load MuJoCo plugins (STL decoder, OBJ decoder, etc.) before loading any model.
        if let Some(dir) = find_plugin_dir() {
            load_all_plugin_libraries(&dir, None)
                .map_err(|e| format!("Failed to load MuJoCo plugins from {dir:?}: {e}"))?;
        }

        let xml = crate::mjcf::export_mjcf_with_options(robot, opts);

        let model = Arc::new(
            MjModel::from_xml_string(&xml)
                .map_err(|e| format!("Failed to load MuJoCo model: {e}"))?,
        );
        let mut data = MjData::new(Arc::clone(&model));

        // Seed MuJoCo's qpos with the user's current joint angles so the sim
        // starts in the same pose the editor is showing. The MJCF only carries
        // structure; per-joint qpos defaults to 0 unless we write them here.
        //
        // **Clamp to the joint's [lower, upper] range whenever the joint
        // declares one** (lower < upper). The clamp is unconditional — not
        // gated on `bake_joint_position_limits` — because the failure mode
        // it guards against bites regardless of whether MuJoCo enforces the
        // limit:
        //
        //   * limits baked    → MuJoCo's hard-constraint solver shoves the
        //                       joint back into range with a huge spike at
        //                       t=0 (origin of the "robot explodes" symptom).
        //   * limits NOT baked → the PD actuator drives toward the stored
        //                       (out-of-range) target while gravity + self-
        //                       collision impulses act on a kinematically
        //                       invalid pose — the first sim step can fling
        //                       a joint by π radians.
        //
        // A real motor can't physically reach an angle outside its mechanical
        // range, so silently starting the sim there is never the right
        // behaviour; clamp and log a warn! so the user knows to fix the
        // source model's home_pose.
        let mut seeded_positions = robot.joint_positions.clone();
        let _ = bake_joint_position_limits; // kept for symmetry with the older path
        for (ji, joint) in robot.joints.iter().enumerate() {
            if joint.joint_type == "fixed" {
                continue;
            }
            if joint.lower < joint.upper {
                let q = seeded_positions[ji].clamp(joint.lower, joint.upper);
                if (q - seeded_positions[ji]).abs() > 1e-9 {
                    log::warn!(
                        "joint {:?} initial position {:.4} clamped to [{:.4}, {:.4}] → {:.4} \
                         (home pose violated joint range — fix the source model's home_pose)",
                        joint.name,
                        seeded_positions[ji],
                        joint.lower,
                        joint.upper,
                        q,
                    );
                }
                seeded_positions[ji] = q;
            }
            if let Some(joint_info) = data.joint(&joint.name) {
                let mut view = joint_info.view_mut(&mut data);
                if !view.qpos.is_empty() {
                    view.qpos[0] = seeded_positions[ji];
                }
            }
        }

        // Refresh xpos/xquat/qfrc_bias etc. from the seeded qpos so the very
        // first sync_back can render the correct initial pose.
        data.forward();

        // Initial control targets: hold the start pose in Position mode, no
        // velocity/torque command in the other modes. Use the *clamped*
        // positions so the Position-PD controller doesn't fight the joint
        // range limiter — see the clamp block above for the failure mode.
        let position_targets = seeded_positions.clone();
        let position_target_velocities = vec![0.0; robot.joints.len()];
        let position_target_accelerations = vec![0.0; robot.joints.len()];
        let velocity_targets = vec![0.0; robot.joints.len()];
        let torque_targets = vec![0.0; robot.joints.len()];
        let position_target_torque_ff = vec![0.0; robot.joints.len()];

        Ok(Self {
            model,
            data,
            time_accumulator: 0.0,
            saved_base_transform: robot.base_transform,
            saved_joint_positions: robot.joint_positions.clone(),
            history: VecDeque::new(),
            // ~10s of history at the default 2 ms timestep — bounded so the
            // ring buffer can't grow without bound during long sessions.
            history_max: 5000,
            position_targets,
            position_target_velocities,
            position_target_accelerations,
            velocity_targets,
            torque_targets,
            position_target_torque_ff,
            wbc_torque_override: None,
            realized_step_rate_hz_ema: 0.0,
            realized_step_rate_alpha: 0.1,
            transition: None,
            sequence: None,
            force_pulses: Vec::new(),
            peaks: vec![JointPeak::default(); robot.joints.len()],
            last_tau: vec![0.0; robot.joints.len()],
            trace: VecDeque::new(),
            trace_max: 5000,
            gravity_compensation: false,
            async_queue: VecDeque::new(),
        })
    }

    /// Append an op to the async queue. The host UI loop drains it gradually,
    /// so call this from script bindings instead of executing the op directly
    /// when you want it to take effect on a future timeline point.
    pub fn async_enqueue(&mut self, op: AsyncSimOp) {
        self.async_queue.push_back(op);
    }

    /// Number of ops still queued.
    pub fn async_pending(&self) -> usize {
        self.async_queue.len()
    }

    /// Drop the entire async queue. Useful from scripts that want a clean
    /// slate before re-queuing a new timeline.
    pub fn async_clear(&mut self) {
        self.async_queue.clear();
    }

    /// Read-only peek at the next op without consuming it.
    pub fn async_peek(&self) -> Option<&AsyncSimOp> {
        self.async_queue.front()
    }

    /// Pop the next op for execution.
    pub fn async_pop(&mut self) -> Option<AsyncSimOp> {
        self.async_queue.pop_front()
    }

    /// Decrement the head Step op's remaining frame count by `n`. Pops the op
    /// off the queue when it hits zero. Returns `true` if the head op was
    /// fully consumed (caller may want to drain the next non-step ops on the
    /// same UI tick).
    pub fn async_consume_step_frames(&mut self, n: u32) -> bool {
        let Some(front) = self.async_queue.front_mut() else {
            return false;
        };
        if let AsyncSimOp::StepFrames(remaining) = front {
            if *remaining <= n {
                self.async_queue.pop_front();
                true
            } else {
                *remaining -= n;
                false
            }
        } else {
            false
        }
    }

    /// Toggle gravity-compensation feedforward in [`Self::apply_controller`].
    /// When enabled, each Position / Velocity-mode joint gets `τ_grav` added
    /// before any clipping. Read by every tick of [`Self::step`] /
    /// [`Self::step_n_frames`]; takes effect on the next physics tick.
    pub fn set_gravity_compensation(&mut self, on: bool) {
        self.gravity_compensation = on;
    }

    /// Whether gravity-compensation feedforward is currently enabled.
    pub fn gravity_compensation(&self) -> bool {
        self.gravity_compensation
    }

    /// Read-only view of the time-series ring buffer (oldest → newest).
    pub fn trace(&self) -> impl Iterator<Item = &TraceFrame> {
        self.trace.iter()
    }

    /// Total samples currently stored in the trace.
    pub fn trace_len(&self) -> usize {
        self.trace.len()
    }

    /// Resize the cap on the time-series ring buffer. Existing samples beyond
    /// the new cap are dropped from the front (oldest first), matching the
    /// per-step eviction behaviour in [`Self::record_trace`]. Setting this to
    /// 0 effectively disables tracing — guard against that at the call site.
    pub fn set_trace_max(&mut self, max: usize) {
        let max = max.max(1);
        self.trace_max = max;
        while self.trace.len() > max {
            self.trace.pop_front();
        }
    }

    /// Read-only access to the per-joint peak observations.
    pub fn peaks(&self) -> &[JointPeak] {
        &self.peaks
    }

    /// Clear the per-joint peak observations and the time-series trace.
    /// Called automatically at the start of every pose transition and
    /// external-force pulse so the plot resets to the new command's response.
    pub fn reset_peaks(&mut self) {
        for p in self.peaks.iter_mut() {
            *p = JointPeak::default();
        }
        self.trace.clear();
    }

    /// Apply a world-frame force / torque to `link_name` for `duration`
    /// seconds. Replaces any pulse currently targeting the same link so the
    /// caller can update force on the fly without stacking entries.
    pub fn apply_external_force(
        &mut self,
        link_name: &str,
        force: [f64; 3],
        torque: [f64; 3],
        duration: f64,
    ) {
        // Drop any existing pulse on the same body, then push the new one.
        self.force_pulses.retain(|p| p.link_name != link_name);
        self.force_pulses.push(ExternalForcePulse {
            link_name: link_name.to_string(),
            force,
            torque,
            duration: duration.max(0.0),
            elapsed: 0.0,
        });
        // Fresh peaks window for this pulse so the user can read the
        // disturbance response without stale history.
        self.reset_peaks();
    }

    /// Cancel any external force currently applied to `link_name`.
    /// Returns true if a pulse was found and removed.
    pub fn cancel_external_force(&mut self, link_name: &str) -> bool {
        let before = self.force_pulses.len();
        self.force_pulses.retain(|p| p.link_name != link_name);
        // Zero out xfrc_applied on the body so MuJoCo stops applying force.
        if let Some(body) = self.data.body(link_name) {
            let mut view = body.view_mut(&mut self.data);
            for v in view.xfrc_applied.iter_mut() {
                *v = 0.0;
            }
        }
        self.force_pulses.len() != before
    }

    /// Iterate the active external-force pulses (for UI status display).
    pub fn external_force_pulses(&self) -> &[ExternalForcePulse] {
        &self.force_pulses
    }

    /// Begin a smooth transition from the current Position-mode targets to
    /// `goal_targets` over `duration` seconds using the chosen interpolation
    /// curve. Subsequent calls cancel any in-progress transition.
    ///
    /// `goal_targets` must be the same length as `position_targets`. Joints
    /// not listed (or whose value matches the current target) simply hold.
    pub fn start_transition(
        &mut self,
        goal_targets: Vec<f64>,
        duration: f64,
        kind: InterpolationKind,
    ) {
        let q_start = self.position_targets.clone();
        let q_end = if goal_targets.len() == q_start.len() {
            goal_targets
        } else {
            let mut padded = q_start.clone();
            for (i, v) in goal_targets.iter().enumerate() {
                if i < padded.len() {
                    padded[i] = *v;
                }
            }
            padded
        };
        self.transition = Some(ActiveTransition {
            traj: misarta::trajectory::PoseTransition::new(
                q_start,
                q_end,
                duration.max(1e-3),
                kind,
            ),
            elapsed: 0.0,
        });
        // Each Play starts a new measurement window for the peaks panel /
        // scripting API; previous-pose peaks would otherwise occlude the
        // current command's response.
        self.reset_peaks();
    }

    /// Begin chained-pose sequence playback. The animation already encodes
    /// each waypoint's absolute time; the controller cancels any single
    /// transition currently active and resets the peaks window so the
    /// recorded τ / q̇ peaks reflect just this sequence.
    pub fn start_sequence(
        &mut self,
        anim: misarta::trajectory::KeyframeAnimation<f64>,
        name: impl Into<String>,
    ) {
        self.transition = None;
        self.sequence = Some(ActiveSequence {
            anim,
            elapsed: 0.0,
            name: name.into(),
        });
        self.reset_peaks();
    }

    /// Whether a chained sequence is currently being played.
    pub fn sequence_in_progress(&self) -> bool {
        self.sequence.is_some()
    }

    /// Normalised progress (0..1) of the current sequence; `None` if idle.
    pub fn sequence_progress(&self) -> Option<f32> {
        self.sequence.as_ref().map(|s| {
            let dur = s.anim.duration().max(1e-9);
            ((s.elapsed / dur).clamp(0.0, 1.0)) as f32
        })
    }

    /// Name of the currently-playing sequence, if any.
    pub fn current_sequence_name(&self) -> Option<&str> {
        self.sequence.as_ref().map(|s| s.name.as_str())
    }

    /// Whether a pose transition is currently playing.
    pub fn transition_in_progress(&self) -> bool {
        self.transition.is_some()
    }

    /// Normalised progress (0.0 → 1.0) of the current transition; `None`
    /// when no transition is active.
    pub fn transition_progress(&self) -> Option<f32> {
        self.transition.as_ref().map(|t| {
            let dur = t.traj.duration.max(1e-9);
            ((t.elapsed / dur).clamp(0.0, 1.0)) as f32
        })
    }

    /// Set the position target (rad / m) for a joint by index.
    pub fn set_position_target(&mut self, joint_idx: usize, target: f64) {
        if let Some(slot) = self.position_targets.get_mut(joint_idx) {
            *slot = target;
        }
    }

    /// Set the velocity target (rad/s / m/s) for a joint by index.
    pub fn set_velocity_target(&mut self, joint_idx: usize, target: f64) {
        if let Some(slot) = self.velocity_targets.get_mut(joint_idx) {
            *slot = target;
        }
    }

    /// Set the direct torque command for a joint by index.
    pub fn set_torque_target(&mut self, joint_idx: usize, target: f64) {
        if let Some(slot) = self.torque_targets.get_mut(joint_idx) {
            *slot = target;
        }
    }

    /// Set the feedforward torque added on top of the Position / ComputedTorque
    /// PD output for one joint. Used by the WBC layer to inject
    /// `-J^T · f_GRF` from the SRBD MPC. Caller must update this every
    /// tick a fresh GRF is available; pair with [`Self::clear_torque_feedforward`]
    /// when the gait controller is disabled.
    pub fn set_torque_feedforward(&mut self, joint_idx: usize, tau: f64) {
        if let Some(slot) = self.position_target_torque_ff.get_mut(joint_idx) {
            *slot = tau;
        }
    }

    /// Zero every joint's torque feedforward in one call. The host calls
    /// this when the gait controller is turned off so a stale GRF-derived
    /// τ_ff doesn't keep accelerating the leg after walking stops.
    pub fn clear_torque_feedforward(&mut self) {
        for slot in self.position_target_torque_ff.iter_mut() {
            *slot = 0.0;
        }
    }

    /// Replace per-joint actuator-mode logic with a direct torque vector
    /// produced by the Hierarchical WBC. The vector is indexed by
    /// `RobotModel::joints` order; entries for fixed joints are
    /// ignored. Stays in effect for every subsequent `step` until
    /// [`Self::clear_wbc_torques`] is called.
    ///
    /// Use case: the gait + WBC pipeline solves the full
    /// `(q̈, f_GRF, τ)` triple under hard physical constraints
    /// (floating-base EoM, friction cone, torque limits) and writes
    /// the resulting `τ` here. This bypasses Position / Velocity /
    /// ComputedTorque paths, since the WBC's torque already
    /// incorporates the equivalent of those PD residuals — adding
    /// them on top would double-count.
    pub fn set_wbc_torques(&mut self, taus: &[f64]) {
        if taus.len() != self.position_targets.len() {
            log::warn!(
                "set_wbc_torques: length mismatch ({} given, {} expected)",
                taus.len(),
                self.position_targets.len()
            );
            return;
        }
        self.wbc_torque_override = Some(taus.to_vec());
    }

    /// Disable WBC-direct torque mode. Subsequent ticks fall back to
    /// per-joint `ActuatorMode` logic (Position PD + τ_ff etc.). Call
    /// when the WBC is turned off in the UI so a stale `τ_wbc` from a
    /// previous run can't keep driving the legs.
    pub fn clear_wbc_torques(&mut self) {
        self.wbc_torque_override = None;
    }

    /// True when the WBC override is currently active. Exposed for the
    /// UI panel so it can show the current control mode.
    pub fn wbc_active(&self) -> bool {
        self.wbc_torque_override.is_some()
    }

    /// Compute and write each motor's `ctrl` (= applied torque) for the
    /// upcoming physics tick, based on the per-joint mode + gains in `robot`
    /// and the controller targets stored on `self`.
    ///
    /// When `enforce_limits` is true the commanded torque is clamped to the
    /// joint's `effort` (τmax / Fmax) and a damping term is folded in once
    /// the velocity exceeds `joint.velocity` (ωmax / vmax). The damping
    /// follows `τ ← τ - kv·(qd - qd_max·sign(qd))` for the over-speed region
    /// so the motor smoothly bleeds energy instead of clipping abruptly. The
    /// flag mirrors the dynamics-panel `⛔ Limits` checkbox.
    ///
    /// When `self.gravity_compensation` is true, a feedforward gravity term
    /// (RNEA evaluated at the current pose, q̇=0) is added to the PD output
    /// for Position and Velocity modes. Torque-mode joints are left alone
    /// since their command is supposed to be the user's full torque request.
    fn apply_controller(&mut self, robot: &mut RobotModel, enforce_limits: bool) {
        // ── WBC direct-torque override path ─────────────────────────
        // When the host has handed us a per-joint τ vector from the
        // Hierarchical WBC, write each motor's `ctrl` directly without
        // running per-joint Position / Velocity / ComputedTorque PD
        // logic. The WBC's torque already incorporates the equivalent
        // of those PD residuals (the QP minimises tracking error of
        // base accel + swing accel under hard constraints), so adding
        // them on top would double-count.
        if let Some(taus) = self.wbc_torque_override.clone() {
            for (ji, joint) in robot.joints.iter().enumerate() {
                if joint.joint_type == "fixed" {
                    continue;
                }
                let mut tau = taus.get(ji).copied().unwrap_or(0.0);
                if enforce_limits && joint.effort > 0.0 {
                    tau = tau.clamp(-joint.effort, joint.effort);
                }
                let actuator_name = format!("motor_{}", joint.name);
                if let Some(act_info) = self.data.actuator(&actuator_name) {
                    let mut view = act_info.view_mut(&mut self.data);
                    if !view.ctrl.is_empty() {
                        view.ctrl[0] = tau;
                    }
                }
                // Update the running peaks + last_tau the same way the
                // per-joint path does so the trace plots line up
                // regardless of which control mode is active.
                if let Some(peak) = self.peaks.get_mut(ji) {
                    let tau_abs = tau.abs();
                    if tau_abs > peak.tau_abs {
                        peak.tau_abs = tau_abs;
                        peak.tau_signed = tau;
                    }
                    let qd = match self.data.joint(&joint.name) {
                        Some(info) => {
                            let view = info.view(&self.data);
                            if view.qvel.is_empty() {
                                0.0
                            } else {
                                view.qvel[0]
                            }
                        }
                        None => 0.0,
                    };
                    let qd_abs = qd.abs();
                    if qd_abs > peak.qvel_abs {
                        peak.qvel_abs = qd_abs;
                        peak.qvel_signed = qd;
                    }
                }
                if let Some(slot) = self.last_tau.get_mut(ji) {
                    *slot = tau;
                }
            }
            return;
        }

        // Pre-compute feedforward vectors once per tick. Two independent
        // streams may need them:
        //   gravity_comp  → τ_grav = compute_gravity(q)            (q̇=0, q̈=0)
        //   ComputedTorque → τ_invdyn = rnea(q, q̇, q̈*)             (full inverse dynamics)
        // We compute them separately because they're not the same vector:
        // Position+grav_comp joints want pure gravity, while ComputedTorque
        // joints additionally want the M·q̈* and Coriolis terms specific to
        // the commanded acceleration.
        let any_computed_torque = robot
            .joints
            .iter()
            .any(|j| j.actuator_mode == ActuatorMode::ComputedTorque);
        let need_state_sync = self.gravity_compensation || any_computed_torque;
        if need_state_sync {
            // Sync MuJoCo's pre-step state into `robot` so build_q reflects
            // the current configuration (joint angles + floating base).
            // Without this the feedforward vectors would be stale by one tick.
            self.sync_back(robot);
        }

        let gravity_torques: Option<Vec<f64>> = if self.gravity_compensation {
            let adapter = robot.mc();
            let q = robot.build_q();
            let g_full = misarta::rnea::compute_gravity(&adapter.model, &q);
            Some(project_nv_to_joints(&g_full, &adapter, robot.joints.len()))
        } else {
            None
        };

        let computed_torque_ff: Option<Vec<f64>> = if any_computed_torque {
            let adapter = robot.mc();
            let q = robot.build_q();
            // Build v (q̇) from MuJoCo and a (q̈*) from the trajectory
            // feedforward — but only populate `a` for joints actually in
            // ComputedTorque mode, so a Position-mode joint's q̈* isn't
            // accidentally fed into the inverse-dynamics computation for
            // a different joint's row of M(q).
            let nv = adapter.model.nv;
            let mut v = vec![0.0_f64; nv];
            let mut a = vec![0.0_f64; nv];
            for ji in 0..robot.joints.len() {
                let Some(mi) = adapter.a2m.get(ji).and_then(|&m| m) else {
                    continue;
                };
                if adapter.model.joints[mi].joint_type.nv() != 1 {
                    continue;
                }
                let vi = adapter.model.v_idx[mi];
                // q̇ from MuJoCo state
                if let Some(info) = self.data.joint(&robot.joints[ji].name) {
                    let view = info.view(&self.data);
                    if !view.qvel.is_empty() {
                        v[vi] = view.qvel[0];
                    }
                }
                if robot.joints[ji].actuator_mode == ActuatorMode::ComputedTorque {
                    a[vi] = self
                        .position_target_accelerations
                        .get(ji)
                        .copied()
                        .unwrap_or(0.0);
                }
            }
            let tau_full = misarta::rnea::rnea(&adapter.model, &q, &v, &a);
            Some(project_nv_to_joints(&tau_full, &adapter, robot.joints.len()))
        } else {
            None
        };

        for (ji, joint) in robot.joints.iter().enumerate() {
            if joint.joint_type == "fixed" {
                continue;
            }

            // Read current MuJoCo state for this joint.
            let (q, qd) = match self.data.joint(&joint.name) {
                Some(info) => {
                    let view = info.view(&self.data);
                    if view.qpos.is_empty() || view.qvel.is_empty() {
                        continue;
                    }
                    (view.qpos[0], view.qvel[0])
                }
                None => continue,
            };

            let mut tau = match joint.actuator_mode {
                ActuatorMode::Position => {
                    let q_target = self.position_targets.get(ji).copied().unwrap_or(q);
                    // Trajectory-velocity feedforward: damping must reference
                    // the trajectory's q̇*, not zero. Otherwise the Kv term
                    // would brake against the commanded motion every tick a
                    // sequence is being played, which manifests as large
                    // torque oscillation during fast moves like jumps.
                    let qd_target = self
                        .position_target_velocities
                        .get(ji)
                        .copied()
                        .unwrap_or(0.0);
                    let pd = joint.actuator_kp * (q_target - q)
                        + joint.actuator_kv * (qd_target - qd);
                    // Gravity feedforward: the PD now only has to correct
                    // tracking error, not also fight the joint's static
                    // load. This drops the static error from `τ_grav / Kp`
                    // (typically a few degrees) to essentially zero.
                    let grav = gravity_torques
                        .as_ref()
                        .and_then(|g| g.get(ji))
                        .copied()
                        .unwrap_or(0.0);
                    // WBC torque feedforward: when the gait controller's
                    // SRBD MPC has solved for stance-leg ground reaction
                    // forces, it pushes `-J^T · f_GRF` here so the joint
                    // produces those forces without the PD having to
                    // chase them via tracking error. Zero unless the host
                    // wired the gait→sim feedforward each tick.
                    let tau_ff = self
                        .position_target_torque_ff
                        .get(ji)
                        .copied()
                        .unwrap_or(0.0);
                    pd + grav + tau_ff
                }
                ActuatorMode::Velocity => {
                    let mut qd_target =
                        self.velocity_targets.get(ji).copied().unwrap_or(0.0);
                    if enforce_limits && joint.velocity > 0.0 {
                        // Clamp the velocity reference to the joint's rated
                        // q̇max so the controller doesn't ask for an
                        // unreachable speed.
                        qd_target = qd_target.clamp(-joint.velocity, joint.velocity);
                    }
                    let pd = joint.actuator_kv * (qd_target - qd);
                    let grav = gravity_torques
                        .as_ref()
                        .and_then(|g| g.get(ji))
                        .copied()
                        .unwrap_or(0.0);
                    pd + grav
                }
                ActuatorMode::Torque => {
                    // Pure torque mode is the user's explicit request — don't
                    // add gravity comp here, otherwise scripts that command
                    // `τ_target = 0` for a "freeze" would unexpectedly hold
                    // the joint up against gravity.
                    self.torque_targets.get(ji).copied().unwrap_or(0.0)
                }
                ActuatorMode::ComputedTorque => {
                    // τ = M(q)·q̈* + h(q, q̇) + Kp·(q*−q) + Kv·(q̇*−q̇).
                    // The first two terms come from `computed_torque_ff` (one
                    // rnea call covers the whole robot); the PD residual
                    // corrects for modelling error and tracks pose deviations.
                    let q_target =
                        self.position_targets.get(ji).copied().unwrap_or(q);
                    let qd_target = self
                        .position_target_velocities
                        .get(ji)
                        .copied()
                        .unwrap_or(0.0);
                    let pd = joint.actuator_kp * (q_target - q)
                        + joint.actuator_kv * (qd_target - qd);
                    let ff = computed_torque_ff
                        .as_ref()
                        .and_then(|t| t.get(ji))
                        .copied()
                        .unwrap_or(0.0);
                    // WBC torque feedforward (same role as in Position
                    // mode) — additive so the inverse-dynamics term and
                    // the GRF-derived term coexist.
                    let tau_ff = self
                        .position_target_torque_ff
                        .get(ji)
                        .copied()
                        .unwrap_or(0.0);
                    pd + ff + tau_ff
                }
            };

            if enforce_limits {
                // Velocity-saturation back-off: when |q̇| has already exceeded
                // q̇max, add a braking torque proportional to the overspeed so
                // the joint can't keep accelerating past the rated velocity
                // even if the user commands more. Without this, a torque-mode
                // command would simply blow through the speed limit.
                if joint.velocity > 0.0 {
                    let qd_lim = joint.velocity;
                    let overspeed = if qd > qd_lim {
                        qd - qd_lim
                    } else if qd < -qd_lim {
                        qd + qd_lim
                    } else {
                        0.0
                    };
                    if overspeed != 0.0 {
                        tau -= joint.actuator_kv.max(1.0) * overspeed;
                    }
                }
                // Hard torque clip — Final-line-of-defence so the motor never
                // commands more than the rated τmax / Fmax.
                if joint.effort > 0.0 {
                    tau = tau.clamp(-joint.effort, joint.effort);
                }
            }

            // Write to the motor actuator's ctrl slot.
            let actuator_name = format!("motor_{}", joint.name);
            if let Some(act_info) = self.data.actuator(&actuator_name) {
                let mut view = act_info.view_mut(&mut self.data);
                if !view.ctrl.is_empty() {
                    view.ctrl[0] = tau;
                }
            }

            // Update the running peaks for this joint. We store the *commanded*
            // torque (=motor ctrl) since for a default-gear motor that is the
            // applied generalised force; UI labels it as N·m or N depending on
            // the joint type. q̇ is read straight from MuJoCo state for the
            // step we just observed.
            if let Some(peak) = self.peaks.get_mut(ji) {
                let tau_abs = tau.abs();
                if tau_abs > peak.tau_abs {
                    peak.tau_abs = tau_abs;
                    peak.tau_signed = tau;
                }
                let qd_abs = qd.abs();
                if qd_abs > peak.qvel_abs {
                    peak.qvel_abs = qd_abs;
                    peak.qvel_signed = qd;
                }
            }
            if let Some(slot) = self.last_tau.get_mut(ji) {
                *slot = tau;
            }
        }
    }

    /// Capture a sample of (q, q̇, τ) per joint after the most recent
    /// physics step and append to the trace ring. Called from the step loop
    /// once per tick so the resulting timeline matches the plot's expected
    /// dt = `self.timestep()` cadence.
    fn record_trace(&mut self, robot: &RobotModel) {
        if self.trace.len() >= self.trace_max {
            self.trace.pop_front();
        }
        let n = robot.joints.len();
        let mut q = vec![0.0; n];
        let mut qvel = vec![0.0; n];
        for (ji, joint) in robot.joints.iter().enumerate() {
            if joint.joint_type == "fixed" {
                continue;
            }
            if let Some(info) = self.data.joint(&joint.name) {
                let view = info.view(&self.data);
                if !view.qpos.is_empty() {
                    q[ji] = view.qpos[0];
                }
                if !view.qvel.is_empty() {
                    qvel[ji] = view.qvel[0];
                }
            }
        }
        let (base_pos, base_quat) = match self
            .model
            .name_to_id(mujoco::prelude::MjtObj::mjOBJ_BODY, &robot.root_link)
        {
            Some(id) => {
                let p = &self.data.xpos()[id];
                let q = &self.data.xquat()[id];
                (Some([p[0], p[1], p[2]]), Some([q[0], q[1], q[2], q[3]]))
            }
            None => (None, None),
        };
        self.trace.push_back(TraceFrame {
            time: self.data.ffi().time,
            q,
            qvel,
            tau: self.last_tau.clone(),
            base_pos,
            base_quat,
        });
    }

    /// Restore the robot's pre-sim pose (called when the user stops the sim).
    pub fn restore(&self, robot: &mut RobotModel) {
        robot.base_transform = self.saved_base_transform;
        robot.joint_positions = self.saved_joint_positions.clone();
    }

    /// Resolve a joint name to its [`RobotModel`] joint index, or `None` if
    /// the joint does not exist. Used by scripting / UI to look up entries
    /// in [`Self::peaks`] by name.
    pub fn joint_index(&self, robot: &RobotModel, name: &str) -> Option<usize> {
        robot.joint_map.get(name).copied()
    }

    /// Snapshot the active contacts reported by MuJoCo this tick.
    ///
    /// Each `ContactInfo` carries the world-frame contact point, surface
    /// normal, force magnitude, full linear force vector, and the names of
    /// the two bodies the contact involves. Returns an empty Vec when no
    /// contacts are active or when the sim has not run a step yet (since
    /// `contact_force` is only meaningful after `step`).
    pub fn contacts(&self) -> Vec<ContactInfo> {
        // mujoco-rs deprecated `contacts()` in favour of `contact()` but the
        // 3.0.1 release we depend on still ships the old name; suppress the
        // warning rather than chase a single API rename.
        #[allow(deprecated)]
        let raw = self.data.contacts();
        if raw.is_empty() {
            return Vec::new();
        }
        // Mapping from geom_id → body name. Looking up via the model is
        // cheap (constant slice + string pool) but we still keep a small
        // local cache in case the same geoms appear in many contacts.
        let model_ffi = self.model.ffi();
        let geom_bodyid = self.model.geom_bodyid();
        let geom_to_body_name = |geom_id: i32| -> String {
            if geom_id < 0 {
                return String::new();
            }
            let g = geom_id as usize;
            if g >= geom_bodyid.len() {
                return String::new();
            }
            let body_id = geom_bodyid[g] as usize;
            if body_id == 0 {
                // World body — leave empty so callers can detect ground contacts.
                return String::new();
            }
            self.model
                .id_to_name(mujoco::prelude::MjtObj::mjOBJ_BODY, body_id)
                .map(|s| s.to_string())
                .unwrap_or_default()
        };
        let _ = model_ffi;

        let mut out = Vec::with_capacity(raw.len());
        for (i, c) in raw.iter().enumerate() {
            // MuJoCo's contact frame is row-major: rows = (normal, t1, t2)
            // expressed in world coordinates. Local force is (Fn, Ft1, Ft2, …);
            // mapping back to world is fᵂ = Rᵀ · fᶜ where R has those rows.
            let f_local = self.data.contact_force(i);
            let n = [c.frame[0], c.frame[1], c.frame[2]];
            let t1 = [c.frame[3], c.frame[4], c.frame[5]];
            let t2 = [c.frame[6], c.frame[7], c.frame[8]];
            // f_world = Fn·n + Ft1·t1 + Ft2·t2
            let fw = [
                f_local[0] * n[0] + f_local[1] * t1[0] + f_local[2] * t2[0],
                f_local[0] * n[1] + f_local[1] * t1[1] + f_local[2] * t2[1],
                f_local[0] * n[2] + f_local[1] * t1[2] + f_local[2] * t2[2],
            ];
            let mag = (fw[0] * fw[0] + fw[1] * fw[1] + fw[2] * fw[2]).sqrt();
            out.push(ContactInfo {
                pos: [c.pos[0], c.pos[1], c.pos[2]],
                force_mag: mag,
                force_world: fw,
                body1: geom_to_body_name(c.geom1),
                body2: geom_to_body_name(c.geom2),
            });
        }
        out
    }

    /// Sum the **vertical** (world-z) ground-contact force on each of
    /// the four foot links, in `[FL, FR, RL, RR]` order matching
    /// `foot_links`.
    ///
    /// Returns `[0; 4]` when no contacts are active. Contact forces
    /// where neither body is the world (= self-collisions) are
    /// ignored — only ground contacts contribute.
    ///
    /// Used by [`quadruped_gait::phase::ContactDrivenPhase`] to detect
    /// **early touchdown** (foot loaded while the nominal schedule
    /// still says swing) and **late liftoff** (foot still loaded
    /// after the nominal stance window ended). Both are common when
    /// trotting over uneven ground or when the MPC's phase clock
    /// drifts relative to the actual physics, and they're the typical
    /// failure mode of pure open-loop gait scheduling.
    pub fn contact_force_per_foot(&self, foot_links: &[&str; 4]) -> [f64; 4] {
        let mut out = [0.0_f64; 4];
        for c in self.contacts() {
            // Pick the non-world side as the "robot body" — ContactInfo
            // leaves the world body's name empty, so exactly one side is
            // empty for ground contacts. Self-collisions (both non-empty)
            // skip via the early `continue`.
            let foot_name: &str = match (c.body1.is_empty(), c.body2.is_empty()) {
                (true, false) => c.body2.as_str(),
                (false, true) => c.body1.as_str(),
                _ => continue,
            };
            for (slot, &link) in foot_links.iter().enumerate() {
                if foot_name == link {
                    out[slot] += c.force_world[2];
                    break;
                }
            }
        }
        out
    }

    /// MuJoCo's native physics timestep (s).
    pub fn timestep(&self) -> f64 {
        self.model.ffi().opt.timestep as f64
    }

    /// MuJoCo's `qfrc_bias` = `C(q, q̇)·q̇ + g(q)`. With `q̇ = 0`
    /// this collapses to the pure gravity-comp generalised force at
    /// the current `q`. Used by the misarta vs MuJoCo dynamics
    /// consistency test to verify that `misarta::compute_gravity`
    /// returns the same value the real simulator uses.
    pub fn qfrc_bias(&self) -> Vec<f64> {
        self.data.qfrc_bias().to_vec()
    }

    /// Realtime achievement ratio of the physics integration:
    /// `realised_step_rate_hz / target_step_rate_hz` where the target
    /// is `1 / timestep()`. 1.0 means we're keeping up at the
    /// requested speed-slider rate; below 1 means the controller +
    /// WBC + MuJoCo loop can't sustain the target tick rate.
    ///
    /// Returns 0.0 before any step has run (no data yet). The value
    /// **can** exceed 1.0 when the user has speed > 1 and the loop
    /// keeps up; the viewport clamps for display purposes.
    pub fn realtime_ratio(&self) -> f64 {
        let target_hz = 1.0 / self.timestep();
        if target_hz > 0.0 {
            (self.realized_step_rate_hz_ema / target_hz).max(0.0)
        } else {
            0.0
        }
    }

    /// Update the realised-step-rate EMA from one step-loop's
    /// `(n_substeps, wall_elapsed)` pair. Called internally by
    /// [`Self::step`] and [`Self::step_n_frames`].
    fn update_step_rate_ema(&mut self, n_steps: u32, wall_elapsed_s: f64) {
        if n_steps == 0 || wall_elapsed_s <= 1e-9 {
            return;
        }
        let rate_hz = n_steps as f64 / wall_elapsed_s;
        let a = self.realized_step_rate_alpha;
        self.realized_step_rate_hz_ema =
            a * rate_hz + (1.0 - a) * self.realized_step_rate_hz_ema;
    }

    /// Observed world-frame linear velocity of `body_link` (m/s).
    /// Returns `None` when the name isn't present in the compiled MJCF.
    /// MuJoCo's `cvel` row layout is `[ω_x, ω_y, ω_z, v_x, v_y, v_z]`.
    pub fn body_world_linear_velocity(&self, body_link: &str) -> Option<[f64; 3]> {
        let id = self
            .model
            .name_to_id(mujoco::prelude::MjtObj::mjOBJ_BODY, body_link)?;
        let cvel = self.data.cvel();
        let row = &cvel[id];
        Some([row[3], row[4], row[5]])
    }

    /// Observed world-frame angular velocity of `body_link` (rad/s).
    /// Returns `None` when the name isn't present in the compiled MJCF.
    pub fn body_world_angular_velocity(&self, body_link: &str) -> Option<[f64; 3]> {
        let id = self
            .model
            .name_to_id(mujoco::prelude::MjtObj::mjOBJ_BODY, body_link)?;
        let cvel = self.data.cvel();
        let row = &cvel[id];
        Some([row[0], row[1], row[2]])
    }

    /// World-frame position of `body_link` (m). Reads MuJoCo's `xpos` —
    /// ground-truth position straight out of the integrator. Used as the
    /// `GroundTruth` pose source and as a fallback when the IMU-fusion
    /// path can't recover position (Madgwick is attitude-only).
    pub fn body_world_position(&self, body_link: &str) -> Option<[f64; 3]> {
        let id = self
            .model
            .name_to_id(mujoco::prelude::MjtObj::mjOBJ_BODY, body_link)?;
        let xpos = self.data.xpos();
        let row = &xpos[id];
        Some([row[0], row[1], row[2]])
    }

    /// Read `(q, q̇)` for a single joint by name. Used by the leg-
    /// odometry estimator to pull encoder data from MuJoCo without
    /// going through the full `RobotModel` sync. Returns `None` for
    /// joints absent from the compiled MJCF or with empty
    /// position/velocity views.
    pub fn joint_q_qd(&self, joint_name: &str) -> Option<(f64, f64)> {
        let info = self.data.joint(joint_name)?;
        let view = info.view(&self.data);
        if view.qpos.is_empty() || view.qvel.is_empty() {
            return None;
        }
        Some((view.qpos[0], view.qvel[0]))
    }

    /// World-frame orientation of `body_link` as a unit quaternion.
    /// MuJoCo's `xquat` stores `[w, x, y, z]` (Hamilton); we hand it
    /// to `nalgebra::Quaternion::new(w, i, j, k)` directly. Used by
    /// the WBC pipeline to sync the misarta floating-base `q[3..7]`
    /// (so gravity-comp and Coriolis terms reflect the actual body
    /// tilt, not a synthetic identity orientation).
    pub fn body_world_orientation(
        &self,
        body_link: &str,
    ) -> Option<nalgebra::UnitQuaternion<f64>> {
        let id = self
            .model
            .name_to_id(mujoco::prelude::MjtObj::mjOBJ_BODY, body_link)?;
        let xquat = self.data.xquat();
        let q = &xquat[id];
        Some(nalgebra::UnitQuaternion::from_quaternion(
            nalgebra::Quaternion::new(q[0], q[1], q[2], q[3]),
        ))
    }

    /// World-frame yaw of `body_link` (rad). Extracts the z-axis Euler
    /// angle from MuJoCo's `xquat = [w, x, y, z]` (Hamilton convention).
    pub fn body_world_yaw(&self, body_link: &str) -> Option<f64> {
        let id = self
            .model
            .name_to_id(mujoco::prelude::MjtObj::mjOBJ_BODY, body_link)?;
        let xquat = self.data.xquat();
        let q = &xquat[id];
        // yaw = atan2(2(w·z + x·y), 1 − 2(y² + z²)) (ZYX Euler).
        let yaw = (2.0 * (q[0] * q[3] + q[1] * q[2]))
            .atan2(1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]));
        Some(yaw)
    }

    /// Read every IMU sensor declared on the loaded `RobotModel` from
    /// MuJoCo's sensor array. Each IMU is expected to have an
    /// `<accelerometer>` and a `<gyro>` channel in the compiled MJCF
    /// named `<imu_name>_accel` / `<imu_name>_gyro`. Sensors whose
    /// channels can't be located are silently skipped (logged once at
    /// trace level) so the caller doesn't have to special-case
    /// partially-instrumented models. Both readings are in the **sensor
    /// frame** (the `<site>` MuJoCo emits internally) — for an IMU
    /// mounted with identity origin on a link, this matches the link's
    /// local frame.
    pub fn imu_readings(&self, robot: &RobotModel) -> Vec<ImuReading> {
        let mut out = Vec::new();
        let sensordata = self.data.sensordata();
        let sensor_adr = self.model.sensor_adr();
        let sensor_dim = self.model.sensor_dim();

        for sensor in &robot.sensors {
            let crate::rbd::model::SensorKind::Imu { .. } = &sensor.kind else {
                continue;
            };
            let accel_name = format!("{}_accel", sensor.name);
            let gyro_name = format!("{}_gyro", sensor.name);
            let Some(accel_id) = self
                .model
                .name_to_id(mujoco::prelude::MjtObj::mjOBJ_SENSOR, &accel_name)
            else {
                log::trace!("imu_readings: sensor '{accel_name}' not in compiled MJCF");
                continue;
            };
            let Some(gyro_id) = self
                .model
                .name_to_id(mujoco::prelude::MjtObj::mjOBJ_SENSOR, &gyro_name)
            else {
                log::trace!("imu_readings: sensor '{gyro_name}' not in compiled MJCF");
                continue;
            };
            let accel_offset = sensor_adr[accel_id] as usize;
            let gyro_offset = sensor_adr[gyro_id] as usize;
            debug_assert_eq!(sensor_dim[accel_id], 3);
            debug_assert_eq!(sensor_dim[gyro_id], 3);
            let accel = [
                sensordata[accel_offset],
                sensordata[accel_offset + 1],
                sensordata[accel_offset + 2],
            ];
            let gyro = [
                sensordata[gyro_offset],
                sensordata[gyro_offset + 1],
                sensordata[gyro_offset + 2],
            ];
            out.push(ImuReading {
                name: sensor.name.clone(),
                link: sensor.link.clone(),
                accel,
                gyro,
                sim_time: self.data.ffi().time,
            });
        }
        out
    }

    /// Write the per-body `xfrc_applied` slots from the active force pulses
    /// for the upcoming physics tick, then advance their timers and drop
    /// expired entries (zeroing their slot first so MuJoCo stops applying).
    fn advance_force_pulses(&mut self, mj_dt: f64) {
        // Two-pass: first apply forces (immutable read of pulses, mutable
        // write to data), then update timers (mutable write to pulses).
        for pulse in &self.force_pulses {
            if let Some(body) = self.data.body(&pulse.link_name) {
                let mut view = body.view_mut(&mut self.data);
                if view.xfrc_applied.len() >= 6 {
                    view.xfrc_applied[0] = pulse.force[0];
                    view.xfrc_applied[1] = pulse.force[1];
                    view.xfrc_applied[2] = pulse.force[2];
                    view.xfrc_applied[3] = pulse.torque[0];
                    view.xfrc_applied[4] = pulse.torque[1];
                    view.xfrc_applied[5] = pulse.torque[2];
                }
            }
        }
        // Advance timers and zero out expired pulses' slots.
        let mut expired: Vec<String> = Vec::new();
        for pulse in self.force_pulses.iter_mut() {
            pulse.elapsed += mj_dt;
            if pulse.elapsed >= pulse.duration {
                expired.push(pulse.link_name.clone());
            }
        }
        if !expired.is_empty() {
            for name in &expired {
                if let Some(body) = self.data.body(name) {
                    let mut view = body.view_mut(&mut self.data);
                    for v in view.xfrc_applied.iter_mut() {
                        *v = 0.0;
                    }
                }
            }
            self.force_pulses
                .retain(|p| !expired.contains(&p.link_name));
        }
    }

    /// Advance any active pose transition by `mj_dt` seconds and update
    /// `position_targets` to the interpolated joint vector. When the transition
    /// completes, it is dropped and `position_targets` snaps to the goal.
    fn advance_transition(&mut self, mj_dt: f64) {
        let Some(t) = self.transition.as_mut() else {
            return;
        };
        t.elapsed += mj_dt;
        let q = t.traj.evaluate(t.elapsed);
        let qd = t.traj.evaluate_velocity(t.elapsed);
        let qdd = t.traj.evaluate_acceleration(t.elapsed);
        let n = self.position_targets.len().min(q.len());
        for i in 0..n {
            self.position_targets[i] = q[i];
        }
        let nv = self.position_target_velocities.len().min(qd.len());
        for i in 0..nv {
            self.position_target_velocities[i] = qd[i];
        }
        let na = self.position_target_accelerations.len().min(qdd.len());
        for i in 0..na {
            self.position_target_accelerations[i] = qdd[i];
        }
        if t.traj.is_done(t.elapsed) {
            // Snap exactly to the goal so any rounding error doesn't linger.
            for i in 0..self.position_targets.len().min(t.traj.q_end.len()) {
                self.position_targets[i] = t.traj.q_end[i];
            }
            // After the transition completes the controller should hold pose,
            // so the velocity / acceleration feedforward must collapse back
            // to zero — leaving stale q̇* / q̈* would have the PD continuously
            // commanding motion (and computed-torque would feed M·q̈* of zero
            // anyway, but be explicit).
            for v in self.position_target_velocities.iter_mut() {
                *v = 0.0;
            }
            for a in self.position_target_accelerations.iter_mut() {
                *a = 0.0;
            }
            self.transition = None;
        }
    }

    /// Advance the active multi-step sequence by `mj_dt` seconds and copy
    /// the keyframe animation's interpolated joint vector into
    /// `position_targets`. When the sequence completes, it is dropped (the
    /// final keyframe's q-vector remains in `position_targets` as the new
    /// hold pose).
    fn advance_sequence(&mut self, mj_dt: f64) {
        let Some(s) = self.sequence.as_mut() else {
            return;
        };
        s.elapsed += mj_dt;
        let q = s.anim.evaluate(s.elapsed);
        let qd = s.anim.evaluate_velocity(s.elapsed);
        let qdd = s.anim.evaluate_acceleration(s.elapsed);
        let n = self.position_targets.len().min(q.len());
        for i in 0..n {
            self.position_targets[i] = q[i];
        }
        let nv = self.position_target_velocities.len().min(qd.len());
        for i in 0..nv {
            self.position_target_velocities[i] = qd[i];
        }
        let na = self.position_target_accelerations.len().min(qdd.len());
        for i in 0..na {
            self.position_target_accelerations[i] = qdd[i];
        }
        if s.anim.is_done(s.elapsed) {
            // Same rationale as advance_transition: drop the feedforward to
            // zero once the playback ends, otherwise the controller would
            // keep nudging joints in the direction of the last segment.
            for v in self.position_target_velocities.iter_mut() {
                *v = 0.0;
            }
            for a in self.position_target_accelerations.iter_mut() {
                *a = 0.0;
            }
            self.sequence = None;
        }
    }

    /// Step the simulation by `dt` seconds and sync the state back to `RobotModel`.
    ///
    /// `enforce_limits` is forwarded straight to [`Self::apply_controller`]
    /// — when true the commanded torques and velocity references are clamped
    /// to each joint's `effort` and `velocity` ratings. Wire it from the UI
    /// `⛔ Limits` checkbox.
    pub fn step(&mut self, robot: &mut RobotModel, dt: f64, enforce_limits: bool) {
        self.time_accumulator += dt;

        let mj_dt = self.timestep();
        let wall_start = std::time::Instant::now();
        let mut n_steps: u32 = 0;
        while self.time_accumulator >= mj_dt {
            self.advance_sequence(mj_dt);
            self.advance_transition(mj_dt);
            self.advance_force_pulses(mj_dt);
            self.apply_controller(robot, enforce_limits);
            self.snapshot();
            self.data.step();
            self.record_trace(robot);
            self.time_accumulator -= mj_dt;
            n_steps += 1;
        }
        self.update_step_rate_ema(n_steps, wall_start.elapsed().as_secs_f64());

        self.sync_back(robot);
    }

    /// Advance the simulation by exactly `n` physics frames (each = `timestep()`
    /// seconds) and sync the state back. Each frame is pre-snapshotted so it
    /// can be reversed via [`Self::step_back_frames`].
    pub fn step_n_frames(&mut self, robot: &mut RobotModel, n: u32, enforce_limits: bool) {
        let mj_dt = self.timestep();
        let wall_start = std::time::Instant::now();
        for _ in 0..n {
            self.advance_sequence(mj_dt);
            self.advance_transition(mj_dt);
            self.advance_force_pulses(mj_dt);
            self.apply_controller(robot, enforce_limits);
            self.snapshot();
            self.data.step();
            self.record_trace(robot);
        }
        self.update_step_rate_ema(n, wall_start.elapsed().as_secs_f64());
        // Drop any partial-frame accumulator so explicit frame stepping is exact.
        self.time_accumulator = 0.0;
        self.sync_back(robot);
    }

    /// Restore the simulation to its state `n` frames ago (or as far back as
    /// the history allows). Calls `mj_forward` to refresh derived quantities
    /// before syncing back to `robot`.
    pub fn step_back_frames(&mut self, robot: &mut RobotModel, n: u32) {
        let mut popped = 0;
        while popped < n {
            let Some(snap) = self.history.pop_back() else {
                break;
            };
            self.data.qpos_mut().copy_from_slice(&snap.qpos);
            self.data.qvel_mut().copy_from_slice(&snap.qvel);
            // SAFETY: Writing the scalar `time` field on the FFI struct is a
            // simple memory store — no MuJoCo invariants depend on the value
            // beyond the next call to `data.forward()` below.
            unsafe { self.data.ffi_mut().time = snap.time; }
            popped += 1;
        }
        if popped > 0 {
            // Refresh xpos/xquat/qfrc_bias etc. from the restored qpos/qvel.
            self.data.forward();
            self.time_accumulator = 0.0;
        }
        self.sync_back(robot);
    }

    /// Number of recorded frames currently available for [`Self::step_back_frames`].
    pub fn history_len(&self) -> usize {
        self.history.len()
    }

    /// Push a pre-step snapshot to the history ring, dropping the oldest if
    /// the buffer is full.
    fn snapshot(&mut self) {
        if self.history.len() >= self.history_max {
            self.history.pop_front();
        }
        self.history.push_back(FrameSnapshot {
            qpos: self.data.qpos().to_vec(),
            qvel: self.data.qvel().to_vec(),
            time: self.data.ffi().time,
        });
    }

    /// Mirror MuJoCo's body and joint state back into `robot`.
    fn sync_back(&self, robot: &mut RobotModel) {
        // Floating-base world pose from root body xpos / xquat.
        if let Some(body_info) = self.data.body(&robot.root_link) {
            let view = body_info.view(&self.data);
            let translation = na::Translation3::new(view.xpos[0], view.xpos[1], view.xpos[2]);
            // MuJoCo stores quaternions in (w, x, y, z) order, matching
            // nalgebra's `Quaternion::new(w, i, j, k)` constructor.
            let quat = na::Quaternion::new(view.xquat[0], view.xquat[1], view.xquat[2], view.xquat[3]);
            let rotation = na::UnitQuaternion::from_quaternion(quat);
            robot.base_transform = na::Isometry3::from_parts(translation, rotation);
        }

        for (ji, joint) in robot.joints.iter().enumerate() {
            if joint.joint_type == "fixed" {
                continue;
            }
            if let Some(joint_info) = self.data.joint(&joint.name) {
                let view = joint_info.view(&self.data);
                if !view.qpos.is_empty() {
                    robot.joint_positions[ji] = view.qpos[0] as f64;
                }
            }
        }
    }
}

/// Write the captured trace to a CSV file. Each row is one recorded frame;
/// columns are `time_s` followed by `q[name],qvel[name],tau[name]` triplets
/// for every non-fixed joint, in model order. Returns the number of data
/// rows written on success.
///
/// Lives on `mujoco_sim` (not in the `app` UI module) so the scripting layer
/// — which is published from `lib.rs` and can't see `app::*` — can call it
/// to capture traces from automated tuning scripts.
pub fn save_peaks_csv(
    model: &RobotModel,
    sim: &MujocoSim,
    path: &std::path::Path,
) -> Result<usize, String> {
    use std::io::Write;

    if let Some(parent) = path.parent() {
        if !parent.as_os_str().is_empty() && !parent.exists() {
            std::fs::create_dir_all(parent).map_err(|e| format!("{e}"))?;
        }
    }

    let movable: Vec<(usize, &str)> = model
        .joints
        .iter()
        .enumerate()
        .filter(|(_, j)| j.joint_type != "fixed")
        .map(|(i, j)| (i, j.name.as_str()))
        .collect();

    let mut f = std::fs::File::create(path).map_err(|e| format!("{e}"))?;

    let mut header = String::from(
        "time_s,base_px,base_py,base_pz,base_qw,base_qx,base_qy,base_qz,base_yaw",
    );
    for (_, name) in &movable {
        header.push(',');
        header.push_str(&csv_field(&format!("q[{name}]")));
        header.push(',');
        header.push_str(&csv_field(&format!("qvel[{name}]")));
        header.push(',');
        header.push_str(&csv_field(&format!("tau[{name}]")));
    }
    writeln!(f, "{header}").map_err(|e| format!("{e}"))?;

    let mut count = 0usize;
    let t0 = sim.trace().next().map(|fr| fr.time).unwrap_or(0.0);
    for frame in sim.trace() {
        let mut row = format!("{:.6}", frame.time - t0);
        let p = frame.base_pos.unwrap_or([f64::NAN; 3]);
        let q = frame.base_quat.unwrap_or([f64::NAN; 4]);
        let yaw = match frame.base_quat {
            Some([qw, qx, qy, qz]) => {
                (2.0 * (qw * qz + qx * qy))
                    .atan2(1.0 - 2.0 * (qy * qy + qz * qz))
            }
            None => f64::NAN,
        };
        row.push_str(&format!(
            ",{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6}",
            p[0], p[1], p[2], q[0], q[1], q[2], q[3], yaw
        ));
        for (idx, _) in &movable {
            let q = frame.q.get(*idx).copied().unwrap_or(0.0);
            let v = frame.qvel.get(*idx).copied().unwrap_or(0.0);
            let t = frame.tau.get(*idx).copied().unwrap_or(0.0);
            row.push_str(&format!(",{q:.6},{v:.6},{t:.6}"));
        }
        writeln!(f, "{row}").map_err(|e| format!("{e}"))?;
        count += 1;
    }
    Ok(count)
}

/// Quote a CSV field if it contains commas, quotes, or newlines (RFC 4180).
fn csv_field(s: &str) -> String {
    if s.contains(',') || s.contains('"') || s.contains('\n') || s.contains('\r') {
        let escaped = s.replace('"', "\"\"");
        format!("\"{escaped}\"")
    } else {
        s.to_string()
    }
}

/// Project a misarta nv-dimensional vector (one entry per generalised
/// velocity coordinate) onto a per-articara-joint vector. Joints that don't
/// resolve to a misarta joint, or whose misarta joint has nv != 1 (ball,
/// free), get 0. Used by the controller to consume gravity / inverse-
/// dynamics feedforward without re-implementing the index dance everywhere.
fn project_nv_to_joints(
    nv_vec: &na::DVector<f64>,
    adapter: &crate::rbd::model::MisartaCache,
    n_joints: usize,
) -> Vec<f64> {
    let mut out = vec![0.0_f64; n_joints];
    for ji in 0..n_joints {
        if let Some(mi) = adapter.a2m.get(ji).and_then(|&m| m) {
            if adapter.model.joints[mi].joint_type.nv() == 1 {
                out[ji] = nv_vec[adapter.model.v_idx[mi]];
            }
        }
    }
    out
}
