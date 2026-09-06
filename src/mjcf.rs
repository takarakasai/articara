//! MJCF (MuJoCo XML) import and export — articara boundary layer.
//!
//! Parsing and emission live in `misarta_formats::mjcf` (A4, see
//! `doc/refactor_20260702.md` §4.7); this layer converts
//! [`crate::robot::RobotModel`] ⇄ [`misarta::native::MisaFile`] at the
//! boundary and applies the two policies that need editor context:
//!
//! - **Mesh path style** ([`crate::mesh_paths::MeshPathStyle`]): the
//!   emitted `file=` strings depend on the model's on-disk layout
//!   (`package://` resolution, absolute for in-process MuJoCo, relative
//!   + copy for shipping). `Geom::Mesh.file` is rewritten here before
//!   handing the `MisaFile` to the exporter, which emits it verbatim.
//! - **Base auto-lift**: placing the floating base so the lowest visual
//!   sits just above the ground plane needs the *loaded* mesh vertices
//!   (`RobotModel::compute_min_z`), which the format layer never has.

use std::path::Path;

use crate::robot::*;

pub use misarta_formats::mjcf::GroundPlaneCfg;

// ========== Import ==========

/// Parse an MJCF file and return a RobotModel.
///
/// The structural parse happens in `misarta_formats::mjcf::import`
/// (returning a `MisaFile`); meshes are then loaded through the standard
/// `.misa` path with the MJCF's own directory as the asset base.
pub fn import_mjcf(path: &Path) -> Result<RobotModel, String> {
    let out = misarta_formats::mjcf::import(path)?;
    for w in &out.warnings {
        log::warn!("MJCF import {path:?}: {w}");
    }
    RobotModel::from_misa_file(&out.file, path)
}

// ========== Export ==========

/// Options controlling how a [`RobotModel`] is exported to MJCF XML.
///
/// All fields are optional / defaulted, so [`MjcfExportOptions::default()`]
/// reproduces the legacy behaviour of [`export_mjcf`] (auto-lifted base, no
/// ground plane, no actuators, all per-joint hardware limits baked in).
#[derive(Clone, Debug)]
pub struct MjcfExportOptions {
    /// Override for the floating-base world position. `None` = auto-lift so
    /// the lowest link sits just above z = 0.
    pub base_pos: Option<[f64; 3]>,
    /// Embed a collidable ground plane geom at the given configuration.
    pub ground_plane: Option<GroundPlaneCfg>,
    /// When true, emit `<motor>` actuators (named `motor_<joint>`) for each
    /// non-fixed joint so torques can be applied via `data.ctrl`.
    pub add_actuators: bool,
    /// Per-axis locks on the floating-base, ordered `[TX, TY, TZ, RX, RY, RZ]`.
    /// `true` = axis locked (no DoF), `false` = axis free.
    ///
    /// - All `false` → emit `<freejoint/>` (full 6-DoF base, the default)
    /// - All `true`  → emit no joint (base welded to the world at `base_pos`)
    /// - Mixed       → emit individual `<joint type="slide"/>` / `<hinge>`
    ///                 elements only for the unlocked axes
    pub base_locked_axes: [bool; 6],
    /// When true, the `<motor>` actuators carry `forcelimited="true"
    /// forcerange="-effort effort"`, making MuJoCo clamp `data.ctrl` to the
    /// joint's catalogue effort. When false the motors are unrestricted at
    /// the MuJoCo level — useful for "what if the motor were stronger" sweeps.
    /// Defaults to true so a one-off `export_mjcf()` produces a faithful
    /// hardware spec for re-loading in other tools.
    pub bake_actuator_limits: bool,
    /// When true, joints carry their `range="lower upper"` so MuJoCo enforces
    /// the URDF position limits. False omits the range so the joint can swing
    /// past mechanical stops — matching the semantics of "limits off" for
    /// users probing the dynamic envelope.
    pub bake_joint_position_limits: bool,
    /// How `<mesh file="...">` paths are emitted. Default
    /// [`MeshPathStyle::Absolute`] suits in-process loading via
    /// [`mujoco-rs::MjModel::from_xml_string`] (no on-disk anchor →
    /// MuJoCo would otherwise fail to resolve relative paths). When
    /// exporting to a file the user can ship, switch to
    /// [`MeshPathStyle::RelativeToDir`] and call
    /// [`crate::mesh_paths::copy_meshes_to`] afterwards.
    ///
    /// [`MeshPathStyle::Absolute`]: crate::mesh_paths::MeshPathStyle::Absolute
    /// [`MeshPathStyle::RelativeToDir`]: crate::mesh_paths::MeshPathStyle::RelativeToDir
    pub mesh_path_style: crate::mesh_paths::MeshPathStyle,
    /// Override MuJoCo's physics timestep (s). `None` keeps MuJoCo's own
    /// default (2 ms).
    ///
    /// Worth reaching for on light robots, because
    /// [`crate::mujoco_sim::MujocoSim`]'s per-joint PD is an **explicit**
    /// velocity feedback: it is stable only while
    /// `actuator_kv < 2·I/dt`, where `I` is the joint's own inertia
    /// (link inertia + `armature`). A distal joint with `I ~ 1e-4 kg·m²`
    /// caps `kv` below 1 at the default 2 ms step — under any `kv` a
    /// position hold actually wants, so the joint buzzes instead of
    /// holding. Halving `dt` doubles the usable `kv`.
    pub timestep: Option<f64>,
    /// Default contact friction for every emitted geom, ordered
    /// `[sliding, torsional, rolling]`. Emitted into MJCF's
    /// `<default><geom friction="..."/></default>` so ground plane,
    /// foot collisions, and every link collider inherit the same value.
    /// MuJoCo combines contact pairs by per-axis `max`, so foot-on-ground
    /// at this μ from both sides gives a contact μ equal to `sliding`.
    /// Default `[0.7, 0.005, 0.0001]` — μ_slide=0.7 sits in the middle of
    /// the realistic rubber-on-lab-floor range (0.4–1.0) and matches
    /// MPC `friction_mu` defaults.
    pub default_friction: [f64; 3],
    /// Replace the emitted `<motor>` actuators with MuJoCo's own
    /// `<velocity kv="…">` servos, and switch the integrator to
    /// `implicitfast` so their damping is integrated implicitly.
    ///
    /// This matters because articara's own Position/Velocity modes compute
    /// their PD in Rust and push the result through a `motor`, which makes it
    /// an EXPLICIT feedback term bounded by `kv < 2·I/dt` — about 20 for a
    /// 1 ms step and a 0.01 kg·m² rotor. That is a limitation of doing the
    /// servo outside the integrator, not of MuJoCo: a native velocity
    /// actuator under an implicit integrator has no such ceiling, which is
    /// how velocity-controlled robots are normally simulated.
    pub native_velocity_servo: Option<f64>,
    /// Integrator name for `<option integrator="…">`. `None` leaves MuJoCo's
    /// default (semi-implicit Euler).
    pub integrator: Option<&'static str>,
    /// `<option impratio="…">`: ratio of frictional-to-normal constraint
    /// impedance. MuJoCo's default (1) makes tangential contacts as soft as
    /// normal ones, so a loaded foot creeps along the ground under
    /// tangential load — a stance foot on namiashi's trot skated forward at
    /// 0.1–0.25 m/s. MuJoCo's own guidance for "no slip" is to raise this
    /// (10 or more) together with `cone = "elliptic"`. `None` keeps the
    /// default so existing results are reproducible.
    pub impratio: Option<f64>,
    /// `<option cone="…">`: `"pyramidal"` (MuJoCo default) or `"elliptic"`.
    /// `impratio` only has its documented effect with the elliptic cone.
    pub cone: Option<&'static str>,
}

impl Default for MjcfExportOptions {
    fn default() -> Self {
        Self {
            base_pos: None,
            ground_plane: None,
            add_actuators: false,
            base_locked_axes: [false; 6],
            bake_actuator_limits: true,
            bake_joint_position_limits: true,
            mesh_path_style: crate::mesh_paths::MeshPathStyle::default(),
            default_friction: [0.7, 0.005, 0.0001],
            native_velocity_servo: None,
            integrator: None,
            timestep: None,
            impratio: None,
            cone: None,
        }
    }
}

/// Export a RobotModel to MJCF XML string with default options.
pub fn export_mjcf(model: &RobotModel) -> String {
    export_mjcf_with_options(model, MjcfExportOptions::default())
}

/// Export a RobotModel to a `.xml` file on disk, copying referenced
/// meshes to `<output_dir>/meshes/` and emitting `meshes/<basename>`
/// relative paths. The result is self-contained and portable —
/// `tar`-ing the output directory and shipping it Just Works on the
/// receiving end.
///
/// For in-process loading via `MjModel::from_xml_string` use
/// [`export_mjcf`] / [`export_mjcf_with_options`] directly with the
/// default `Absolute` mesh-path style.
pub fn export_mjcf_to_file(
    model: &RobotModel,
    output_path: &std::path::Path,
) -> Result<(), String> {
    let output_dir = output_path
        .parent()
        .ok_or_else(|| format!("export_mjcf_to_file: invalid path {:?}", output_path))?
        .to_path_buf();
    let mut opts = MjcfExportOptions::default();
    opts.mesh_path_style =
        crate::mesh_paths::MeshPathStyle::RelativeToDir(output_dir.clone());
    let xml = export_mjcf_with_options(model, opts);
    std::fs::write(output_path, xml)
        .map_err(|e| format!("write {:?}: {e}", output_path))?;
    let copied = crate::mesh_paths::copy_meshes_to(model, &output_dir)?;
    log::info!(
        "Exported MJCF to {:?}, copied {} mesh file(s)",
        output_path,
        copied,
    );
    Ok(())
}

/// Full-configurability MJCF export.
///
/// Builds a `MisaFile` from the model, applies the mesh-path policy and
/// the auto-lift, then delegates the XML emission to
/// `misarta_formats::mjcf::export`.
pub fn export_mjcf_with_options(
    model: &RobotModel,
    opts: MjcfExportOptions,
) -> String {
    let mut file = match model.to_misa() {
        Ok(f) => f,
        Err(e) => {
            log::error!("MJCF export: cannot build MisaFile: {e}");
            return String::new();
        }
    };
    crate::mesh_paths::rewrite_mesh_refs(&mut file, model, &opts.mesh_path_style);

    // Either honour the user-supplied base position or auto-lift the root so
    // the lowest visual geometry sits ~5 mm above the active ground plane.
    // `compute_initial_z` walked only joint-origin Z and ignored geom shapes
    // (sphere radius, capsule half-length, mesh extent) — for any robot with
    // collision spheres / capsules on its feet that produced a t=0 contact
    // penetration which MuJoCo's contact solver answered with a violent
    // bounce. `RobotModel::compute_min_z` samples the actual visual primitives.
    let base_pos = opts.base_pos.unwrap_or_else(|| {
        const CLEARANCE_M: f64 = 0.005;
        // model_min_z is in world coords with the current base_transform
        // applied; we want body-relative min_z so subtract the base's
        // current Z out before re-applying the new root_z below.
        let base_z = model.base_transform.translation.z as f64;
        let local_min_z = model
            .compute_min_z()
            .map(|z| z as f64 - base_z)
            .unwrap_or_else(|| compute_initial_z_legacy(model) * -1.0 + 0.01);
        let ground_z = opts.ground_plane.as_ref().map(|g| g.z).unwrap_or(0.0);
        // Solve  root_z + local_min_z = ground_z + clearance  for root_z.
        let root_z = ground_z + CLEARANCE_M - local_min_z;
        [0.0, 0.0, root_z]
    });

    let fopts = misarta_formats::mjcf::MjcfExportOptions {
        base_pos,
        ground_plane: opts.ground_plane,
        add_actuators: opts.add_actuators,
        base_locked_axes: opts.base_locked_axes,
        bake_actuator_limits: opts.bake_actuator_limits,
        bake_joint_position_limits: opts.bake_joint_position_limits,
        default_friction: opts.default_friction,
    };
    let xml = misarta_formats::mjcf::export(&file, &fopts);

    // `misarta_formats::mjcf::export` emits no `<option>` element, so
    // splice one in rather than fork the exporter. MuJoCo accepts
    // `<option>` anywhere among `<mujoco>`'s children.
    let integrator = opts
        .integrator
        .or(opts.native_velocity_servo.map(|_| "implicitfast"));
    let xml = match (opts.timestep, integrator, opts.impratio, opts.cone) {
        (None, None, None, None) => xml,
        (dt, ig, ir, cone) => {
            let mut attrs = String::new();
            if let Some(dt) = dt {
                attrs.push_str(&format!(" timestep=\"{dt}\""));
            }
            if let Some(ig) = ig {
                attrs.push_str(&format!(" integrator=\"{ig}\""));
            }
            if let Some(ir) = ir {
                attrs.push_str(&format!(" impratio=\"{ir}\""));
            }
            if let Some(cone) = cone {
                attrs.push_str(&format!(" cone=\"{cone}\""));
            }
            match xml.find('\n') {
                Some(nl) => format!("{}\n  <option{attrs}/>{}", &xml[..nl], &xml[nl..]),
                None => xml,
            }
        }
    };

    // Swap `<motor …/>` for `<velocity kv="…" …/>`, keeping name, joint and
    // force limits so the rest of the pipeline (which looks actuators up by
    // `motor_<joint>`) does not notice.
    match opts.native_velocity_servo {
        None => xml,
        Some(kv) => xml
            .lines()
            .map(|l| {
                let t = l.trim_start();
                if !t.starts_with("<motor ") {
                    return l.to_string();
                }
                let indent = &l[..l.len() - t.len()];
                format!("{indent}<velocity kv=\"{kv}\"{}", &t["<motor".len()..])
            })
            .collect::<Vec<_>>()
            .join("\n"),
    }
}

/// Computes the minimum cumulative z translation in the kinematic chain.
/// Legacy joint-origin-only fallback. Used only when `compute_min_z` returns
/// `None` (e.g. a model with no visual geometry at all). For models with
/// visuals, the auto-lift path uses `RobotModel::compute_min_z` directly
/// because it accounts for primitive shapes (sphere radius etc.).
fn compute_initial_z_legacy(model: &RobotModel) -> f64 {
    fn min_z_recursive(model: &RobotModel, link: &str, z: f64) -> f64 {
        let mut min = z;
        if let Some(children) = model.children_joints.get(link) {
            for &ji in children {
                let dz = model.joints[ji].origin.translation.z as f64;
                min = min.min(min_z_recursive(model, &model.joints[ji].child_link, z + dz));
            }
        }
        min
    }
    min_z_recursive(model, &model.root_link, 0.0)
}
