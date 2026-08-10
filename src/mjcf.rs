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
    /// Override only the base's x/y, keeping the automatic z lift. Ignored
    /// when `base_pos` is set. For placing a robot somewhere other than the
    /// origin on a field whose surface is still at z=0 -- a start platform,
    /// say -- without having to reproduce the lift calculation to get z.
    pub base_xy: Option<(f64, f64)>,
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
    /// Raw `<geom .../>` (or any other worldbody-legal) XML, spliced in just
    /// before the closing `</worldbody>` tag.
    ///
    /// There is no structured terrain builder in this exporter -- the ground
    /// is a single infinite plane, optionally tilted, via
    /// [`GroundPlaneCfg`]. This is the escape hatch for anything that plane
    /// cannot express (stepping-stone islands, a low-friction patch, a curb),
    /// without inventing a terrain API before there is a second caller that
    /// needs one. `None` (the default) changes nothing.
    pub extra_worldbody_xml: Option<String>,
    /// Raw `<asset>`-legal XML (`<hfield>`, `<texture>`, `<material>`, …),
    /// spliced in as its own `<asset>` block. MJCF merges repeated top-level
    /// sections, so this coexists with the mesh assets the exporter emits.
    ///
    /// The companion to [`Self::extra_worldbody_xml`]: a `<geom
    /// type="hfield" hfield="…"/>` in the worldbody needs the matching
    /// `<hfield>` declared here, and there is no other way to reach the
    /// asset section without forking `misarta_formats::mjcf::export`.
    /// Declaring an hfield with `nrow`/`ncol` and no `file` leaves MuJoCo to
    /// allocate the grid uninitialised, for the host to fill via
    /// [`crate::mujoco_sim::MujocoSim::set_hfield_data`] -- no image file on
    /// disk, and the terrain can change between runs without touching it.
    pub extra_asset_xml: Option<String>,
    /// Raw `<visual>`-legal XML (`<headlight>`, `<rgba>`, `<global>`, …),
    /// spliced as its own `<visual>` block.
    ///
    /// MuJoCo's default scene is lit by a dim headlight and nothing else,
    /// which is fine for a debug view of one robot and leaves a whole
    /// competition field reading as a black rectangle. Also the only way to
    /// reach `<global offwidth>`, without which offscreen renders are capped
    /// at 640x480.
    pub extra_visual_xml: Option<String>,
}

impl Default for MjcfExportOptions {
    fn default() -> Self {
        Self {
            base_pos: None,
            base_xy: None,
            ground_plane: None,
            add_actuators: false,
            base_locked_axes: [false; 6],
            bake_actuator_limits: true,
            bake_joint_position_limits: true,
            mesh_path_style: crate::mesh_paths::MeshPathStyle::default(),
            default_friction: [0.7, 0.005, 0.0001],
            native_velocity_servo: None,
            integrator: None,
            extra_worldbody_xml: None,
            extra_asset_xml: None,
            extra_visual_xml: None,
            timestep: None,
        }
    }
}

/// A straight staircase: `n_steps` steps of `rise_m` height and `run_m` tread
/// depth, `approach_m` of flat floor before the first riser, `top_platform_m`
/// of flat floor after the last one. Built from solid overlapping boxes
/// (via [`MjcfExportOptions::extra_worldbody_xml`]) rather than a
/// heightfield -- each step's box shares one back edge with every other
/// step and extends down to z=-0.5, so at any x the tallest overlapping box
/// is exactly that tread's height and there is no seam a foot could catch
/// on.
///
/// Promoted out of `articara-namiashi/tests/wbc_walk.rs` (where it started
/// as the WBC/MPC harness's own test fixture) once a second caller needed
/// the identical geometry: `examples/namiashi_rl_teleop.rs` builds the same
/// staircase for the RL policy's own live MuJoCo rollout, and both need to
/// agree bit-for-bit with each other (and with the standalone
/// `go2_rl/sim2sim_namiashi_mujoco.py` validation) for any WBC/MPC-vs-RL
/// comparison to mean anything.
#[derive(Clone, Copy, Debug)]
pub struct StaircaseCfg {
    pub rise_m: f64,
    pub run_m: f64,
    pub n_steps: usize,
    /// Flat floor length from the spawn point (x=0) to the first riser.
    pub approach_m: f64,
    /// Flat floor length after the last riser.
    pub top_platform_m: f64,
    pub half_width_m: f64,
}

impl StaircaseCfg {
    pub fn top_platform_start_x(&self) -> f64 {
        self.approach_m + self.n_steps as f64 * self.run_m
    }

    pub fn total_rise_m(&self) -> f64 {
        self.rise_m * self.n_steps as f64
    }

    /// Ground height at world x, per the same step geometry `worldbody_xml`
    /// builds -- an idealized, exact height query, standing in for whatever
    /// a real height-map (LiDAR + mapping) would eventually estimate. There
    /// is no sensor model here (no noise, no occlusion, no latency); this is
    /// deliberately the best case, to test whether height *knowledge* is the
    /// missing piece before spending effort on how to acquire it.
    pub fn height_at(&self, world_x: f64) -> f64 {
        if world_x < self.approach_m {
            0.0
        } else {
            let step = ((world_x - self.approach_m) / self.run_m).floor() as i64 + 1;
            let step = step.clamp(1, self.n_steps as i64);
            step as f64 * self.rise_m
        }
    }

    /// Nudge a horizontal touchdown target off a riser edge, onto solid
    /// tread: clamps `world_x` to stay at least `margin_m` from either edge
    /// of whichever tread it falls on (the top platform counts as one long
    /// tread). No-op on the approach floor -- there is no edge there to
    /// avoid, and the caller is expected to gate on `height_at > 0` anyway
    /// for the same reason the vertical correction does.
    pub fn snap_to_tread(&self, world_x: f64, margin_m: f64) -> f64 {
        if world_x < self.approach_m {
            return world_x;
        }
        let step = ((world_x - self.approach_m) / self.run_m).floor() as i64 + 1;
        let step = step.clamp(1, self.n_steps as i64);
        let start = self.approach_m + (step - 1) as f64 * self.run_m;
        let end = if step == self.n_steps as i64 {
            self.top_platform_start_x() + self.top_platform_m
        } else {
            self.approach_m + step as f64 * self.run_m
        };
        let lo = start + margin_m;
        let hi = (end - margin_m).max(lo);
        world_x.clamp(lo, hi)
    }

    pub fn worldbody_xml(&self) -> String {
        fn box_geom(name: &str, cx: f64, cy: f64, cz: f64, hx: f64, hy: f64, hz: f64, rgba: &str) -> String {
            format!(
                "    <geom name=\"{name}\" type=\"box\" pos=\"{cx} {cy} {cz}\"                  size=\"{hx} {hy} {hz}\" rgba=\"{rgba}\"/>\n"
            )
        }
        let hy = self.half_width_m;
        let bottom_z = -0.5_f64;
        // Runway behind the spawn point. Not exposed as a config knob, but
        // no longer "a little" -- the 3 cm rise sweep found the robot can
        // genuinely walk itself backward down a staircase it had climbed
        // (a slow, uncorrected yaw drift accumulating to a full 180 deg
        // turn while balanced on a tread, with nothing ever commanding one),
        // and 0.5 m was not enough: it walked off the back of the floor and
        // free-fell, which looked like a climbing failure in the summary
        // numbers and was actually a test-track limitation.
        let margin_behind_m = 8.0_f64;
        let stairs_start_x = self.approach_m;
        let back_x = self.top_platform_start_x() + self.top_platform_m;

        let mut xml = String::new();

        // Approach floor. Emitted first so `render_namiashi.py`'s single
        // find-and-replace for the checker material (keyed on this exact
        // rgba string, the same one `GroundPlaneCfg` emits) lands here,
        // matching the floor look of every other namiashi clip.
        {
            let x0 = -margin_behind_m;
            let x1 = stairs_start_x;
            let hx = (x1 - x0) / 2.0;
            let cx = (x0 + x1) / 2.0;
            let hz = (0.0 - bottom_z) / 2.0;
            let cz = (0.0 + bottom_z) / 2.0;
            xml += &box_geom("stair_approach", cx, 0.0, cz, hx, hy, hz, "0.5 0.5 0.55 1");
        }

        // Steps 1..=n_steps. Box i's front edge is its own riser; its back
        // edge is shared (`back_x`) with every other step, so the region it
        // alone determines the height of is exactly its own tread -- see the
        // struct doc comment for why that makes the profile seamless.
        for i in 1..=self.n_steps {
            let x0 = stairs_start_x + (i as f64 - 1.0) * self.run_m;
            let top_z = i as f64 * self.rise_m;
            let hx = (back_x - x0) / 2.0;
            let cx = (back_x + x0) / 2.0;
            let hz = (top_z - bottom_z) / 2.0;
            let cz = (top_z + bottom_z) / 2.0;
            // Alternating shades so the profile reads without a checker
            // material, which would visually fight with a level tread.
            let rgba = if i % 2 == 0 { "0.58 0.56 0.52 1" } else { "0.50 0.48 0.44 1" };
            xml += &box_geom(&format!("stair_step{i}"), cx, 0.0, cz, hx, hy, hz, rgba);
        }

        xml
    }
}

/// The 第31回かわさきロボット競技大会 competition ring, as a heightfield.
///
/// A heightfield rather than composed primitives because every feature here
/// is single-valued in z -- holes are the plate's thickness being *absent*,
/// the central bowl is a dished surface, the edge banks are bumps. Building
/// a 100 mm circular hole out of boxes needs hundreds of them per plate and
/// still stair-steps the rim; one grid expresses all of it.
///
/// # Where the dimensions come from
///
/// Everything except the bank height was recovered from the rulebook PDF's
/// own vector geometry (`31th_ring_0401.pdf`), not scaled off a raster by
/// eye: the drawing's line work was extracted, the ring square used as the
/// scale reference (538.578 pt = 190 cm), and every feature's corners read
/// out of it. Values land on exact centimetres, which is the check that the
/// scale is right. An earlier version of this type guessed the obstacle
/// centres at +/-48 cm; they are +/-65 cm, and the two start platforms are
/// the same size rather than the 30/45 asymmetry the raster suggested.
///
/// `[pdf]` is measured from that geometry. `[assumed]` is not in the drawing
/// at all -- only the edge bank's height, since the rulebook gives its
/// profile as "断面が半楕円形" with no dimension. Every measurement is still
/// a field rather than a constant, because the rulebook itself notes
/// "安全対策及び加工・配置に起因する寸法、形状誤差があります".
#[derive(Clone, Debug)]
pub struct KawasakiRingCfg {
    /// `[pdf]` Ring plate, square, 190 cm on a side.
    pub ring_m: f64,
    /// `[pdf]` Start platform footprint `(width_along_edge, depth_outward)`.
    /// Both platforms measure 45 x 35 cm: the drawing's 30 cm dimension
    /// belongs to something else, and the apparent red/blue asymmetry was an
    /// artifact of reading the raster. Kept as two fields anyway so an
    /// actual asymmetry stays expressible.
    pub blue_platform_m: (f64, f64),
    /// `[pdf]` See `blue_platform_m` -- same size.
    pub red_platform_m: (f64, f64),
    /// Start platform slab THICKNESS. Its top is flush with the ring
    /// surface, which is how the isometric reads -- a start zone the robot
    /// drives off, not a step it has to descend -- so this only sets how far
    /// the slab hangs below. Must be positive; MuJoCo rejects a zero-size
    /// box, and a flush platform is expressed by where the slab sits, not by
    /// giving it no thickness.
    pub platform_h_m: f64,
    /// `[pdf]` Central bowl obstacle: 45 cm square.
    pub bowl_m: f64,
    /// `[pdf]` Width of the bowl's flat outer frame.
    pub bowl_frame_m: f64,
    /// `[pdf]` Bowl rim height. The drawing shows 2.8 cm on the side view
    /// and 2.5 cm on section A-A; taken as the rim, with the difference
    /// most likely frame-vs-lip.
    pub bowl_rim_h_m: f64,
    /// `[pdf]` Height at the bowl's centre, i.e. how far the dish drops.
    pub bowl_centre_h_m: f64,
    /// `[pdf]` Hole-plate obstacles are 30 cm square, 1.5 cm thick.
    pub plate_m: f64,
    pub plate_h_m: f64,
    /// `[pdf]` Single-hole plate: one 180 mm hole, centred.
    pub round_hole_d_m: f64,
    /// `[pdf]` Four-hole plate: 100 mm holes on a 150 mm square pitch.
    pub quad_hole_d_m: f64,
    pub quad_hole_pitch_m: f64,
    /// `[pdf]` Round-hole plate centres, metres from the ring centre.
    pub round_plate_centres: Vec<(f64, f64)>,
    /// `[pdf]` Four-hole plate centres. Drawn rotated 45 deg (diamond).
    pub quad_plate_centres: Vec<(f64, f64)>,
    /// `[pdf]` Half-width of the edge bank's semi-elliptical section. The
    /// banks measure 2 cm across and sit flush against the ring edge.
    pub bank_half_w_m: f64,
    /// `[photo]` Safety barrier around the field: `(height, gap_outside_ring,
    /// thickness)` in metres. Transparent panels on frames, visible in every
    /// competition photo, and the one piece of the surroundings that is
    /// physical rather than scenery -- a robot that leaves the ring stops
    /// here instead of walking out of the world. `None` omits it.
    ///
    /// `[photo]` throughout: read off event photographs, not a drawing.
    pub barrier: Option<(f64, f64, f64)>,
    /// `[photo]` Coloured border framing the ring on the two start sides,
    /// as `(width, height)`. Scenery -- the red and blue frames that make
    /// which end is which readable at a glance. `None` omits it.
    pub border: Option<(f64, f64)>,
    /// Whether to emit lighting, sky and the two fill lights. Off leaves
    /// MuJoCo's default dim headlight and black void.
    ///
    /// Note the colours here are deliberately LIGHTER than the real field,
    /// which is near-black in every photograph. At the real contrast the
    /// 15 mm obstacle plates disappear into the surface they sit on, and a
    /// simulation you cannot read is not more realistic in any useful sense.
    pub lighting: bool,
    /// `[assumed]` Bank height -- the ONE number the rulebook never gives.
    /// "断面が半楕円形" only says the section is a半楕円, so this picks a
    /// height that is not simply the半円 that a 1 cm half-width would imply.
    pub bank_h_m: f64,
    /// `[pdf]` Edge bank segments as `(x0, y0, x1, y1)` centrelines, metres
    /// from the ring centre. Each runs 1 cm in from its own ring edge.
    pub bank_segments: Vec<(f64, f64, f64, f64)>,
    /// Heightfield grid pitch. 5 mm resolves a 100 mm hole across 20 cells.
    pub cell_m: f64,
    /// Flat floor under the ring, as the side length of a square slab.
    /// `None` omits it, leaving a robot that walks off the edge to fall
    /// indefinitely -- which reads as a diverging simulation rather than as
    /// the ring-out it actually is.
    pub floor_size_m: Option<f64>,
    /// Height of the stand under the ring: the distance from the ring
    /// slab's UNDERSIDE down to the venue floor.
    ///
    /// Measured from the underside on purpose -- quoting it from the ring
    /// surface makes the visible gap depend on `plate_thickness_m`, so the
    /// same number looks different every time the slab changes.
    ///
    /// Competition photos put the ring at about table height, so ~0.7 is
    /// what the real setup looks like; the default stays at the 0.20 that
    /// was asked for, since it is one number to change and guessing at a
    /// height nobody specified is not an improvement.
    pub floor_gap_m: f64,
    /// Thickness of the ring slab itself. Not in the rulebook -- the drawing
    /// dimensions the ring's top face and its obstacles, never how deep the
    /// plate is -- so `[assumed]`, and exposed because it eats into
    /// `floor_drop_m`'s visible gap.
    pub plate_thickness_m: f64,
}

impl Default for KawasakiRingCfg {
    /// Measured from `31th_ring_0401.pdf`; see the type's doc comment.
    fn default() -> Self {
        Self {
            ring_m: 1.90,
            blue_platform_m: (0.45, 0.35),
            red_platform_m: (0.45, 0.35),
            platform_h_m: 0.05,
            bowl_m: 0.45,
            bowl_frame_m: 0.05,
            bowl_rim_h_m: 0.028,
            bowl_centre_h_m: 0.012,
            plate_m: 0.30,
            plate_h_m: 0.015,
            round_hole_d_m: 0.18,
            quad_hole_d_m: 0.10,
            quad_hole_pitch_m: 0.15,
            round_plate_centres: vec![(-0.65, 0.0), (0.65, 0.0)],
            quad_plate_centres: vec![(0.0, -0.65), (0.0, 0.65)],
            bank_half_w_m: 0.01,
            barrier: Some((0.60, 0.55, 0.02)),
            border: Some((0.08, 0.03)),
            lighting: true,
            bank_h_m: 0.015,
            // Each bank is 2 cm across and flush with its own ring edge, so
            // the centreline sits 1 cm in from +/-0.95. Top and bottom run
            // 150 cm centred; the two side banks run 100 cm and are offset
            // in opposite directions, which is what makes the layout
            // rotationally symmetric about the ring centre rather than
            // mirror-symmetric.
            bank_segments: vec![
                (-0.75, 0.94, 0.75, 0.94),
                (-0.75, -0.94, 0.75, -0.94),
                (-0.94, -0.25, -0.94, 0.75),
                (0.94, -0.75, 0.94, 0.25),
            ],
            cell_m: 0.005,
            floor_size_m: Some(5.0),
            floor_gap_m: 0.20,
            plate_thickness_m: 0.05,
        }
    }
}

impl KawasakiRingCfg {
    /// Depth of the floor's top surface below the ring surface, metres --
    /// the slab's own thickness plus the air gap under it.
    pub fn floor_top_z(&self) -> f64 {
        self.plate_thickness_m + self.floor_gap_m
    }

    /// Grid dimensions of the heightfield, `(nrow, ncol)`.
    pub fn grid(&self) -> (usize, usize) {
        let n = (self.ring_m / self.cell_m).round().max(2.0) as usize;
        (n, n)
    }

    /// Tallest feature, metres. MuJoCo scales the normalised grid by this.
    pub fn z_top_m(&self) -> f64 {
        self.bowl_rim_h_m
            .max(self.plate_h_m)
            .max(self.bank_h_m)
            .max(1e-6)
    }

    /// Elevation grid, row-major, normalised to `[0, 1]` for
    /// [`crate::mujoco_sim::MujocoSim::set_hfield_data`].
    ///
    /// Row 0 is `-y`, column 0 is `-x`, matching MuJoCo's own hfield layout.
    pub fn heights(&self) -> Vec<f32> {
        let (nrow, ncol) = self.grid();
        let z_top = self.z_top_m();
        let half = self.ring_m / 2.0;
        let mut out = vec![0.0_f32; nrow * ncol];
        for r in 0..nrow {
            // Cell centres, so a feature edge never lands exactly on a
            // sample and alias to whichever side floating point picks.
            let y = -half + (r as f64 + 0.5) * self.ring_m / nrow as f64;
            for c in 0..ncol {
                let x = -half + (c as f64 + 0.5) * self.ring_m / ncol as f64;
                out[r * ncol + c] = (self.height_at(x, y) / z_top) as f32;
            }
        }
        out
    }

    /// Surface height at a ring-frame point, metres above the plate.
    ///
    /// Features are applied highest-wins rather than in sequence: they do
    /// not overlap in the nominal layout, but a mistyped centre should show
    /// as two obstacles intersecting, not as one silently erasing the other.
    pub fn height_at(&self, x: f64, y: f64) -> f64 {
        let mut z: f64 = 0.0;

        // Central bowl: flat frame at the rim, dishing to the centre.
        let hb = self.bowl_m / 2.0;
        if x.abs() <= hb && y.abs() <= hb {
            let inner = hb - self.bowl_frame_m;
            // Chebyshev radius, so the dish is square-symmetric like the
            // drawing's four triangular faces rather than a circular bowl.
            let t = (x.abs().max(y.abs()) - 0.0) / inner.max(1e-9);
            z = z.max(if x.abs() <= inner && y.abs() <= inner {
                self.bowl_centre_h_m
                    + (self.bowl_rim_h_m - self.bowl_centre_h_m) * t.clamp(0.0, 1.0)
            } else {
                self.bowl_rim_h_m
            });
        }

        let hp = self.plate_m / 2.0;
        for &(cx, cy) in &self.round_plate_centres {
            let (dx, dy) = (x - cx, y - cy);
            if dx.abs() <= hp && dy.abs() <= hp && dx.hypot(dy) > self.round_hole_d_m / 2.0 {
                z = z.max(self.plate_h_m);
            }
        }
        for &(cx, cy) in &self.quad_plate_centres {
            // Drawn as a diamond: rotate the query into the plate's frame.
            let (dx, dy) = (x - cx, y - cy);
            let s = std::f64::consts::FRAC_1_SQRT_2;
            let (px, py) = (s * (dx + dy), s * (dy - dx));
            if px.abs() > hp || py.abs() > hp {
                continue;
            }
            let q = self.quad_hole_pitch_m / 2.0;
            let in_hole = [(-q, -q), (q, -q), (-q, q), (q, q)]
                .iter()
                .any(|&(ox, oy)| (px - ox).hypot(py - oy) <= self.quad_hole_d_m / 2.0);
            if !in_hole {
                z = z.max(self.plate_h_m);
            }
        }

        // Edge banks: semi-elliptical across the segment, flat along it.
        for &(x0, y0, x1, y1) in &self.bank_segments {
            let (vx, vy) = (x1 - x0, y1 - y0);
            let len2 = vx * vx + vy * vy;
            if len2 < 1e-12 {
                continue;
            }
            let t = (((x - x0) * vx + (y - y0) * vy) / len2).clamp(0.0, 1.0);
            let d = (x - (x0 + t * vx)).hypot(y - (y0 + t * vy));
            if d < self.bank_half_w_m {
                let u = d / self.bank_half_w_m;
                z = z.max(self.bank_h_m * (1.0 - u * u).max(0.0).sqrt());
            }
        }
        z
    }

    /// The `<hfield>` declaration for
    /// [`MjcfExportOptions::extra_asset_xml`]. No `file=`, so MuJoCo
    /// allocates the grid and the host fills it.
    pub fn asset_xml(&self, name: &str) -> String {
        let (nrow, ncol) = self.grid();
        // The 4th size component is how far the field extends BELOW z=0,
        // i.e. the ring slab itself -- MuJoCo builds that solid for free.
        // A separate box for the plate would duplicate exactly this volume
        // and put two coplanar faces at z=0, which renders as a speckled
        // mess of z-fighting across the whole field.
        format!(
            "    <hfield name=\"{name}\" nrow=\"{nrow}\" ncol=\"{ncol}\" \
             size=\"{} {} {} {}\"/>\n",
            self.ring_m / 2.0,
            self.ring_m / 2.0,
            self.z_top_m(),
            self.plate_thickness_m,
        )
    }

    /// Ring plate, start platforms and the heightfield geom, for
    /// [`MjcfExportOptions::extra_worldbody_xml`].
    ///
    /// The plate is a solid box under the heightfield rather than the
    /// heightfield's own base thickness, so the ring has sides a robot can
    /// fall off -- driving off the edge is part of this field, unlike the
    /// staircase where it was only ever a test-track artifact.
    pub fn worldbody_xml(&self, name: &str) -> String {
        let half = self.ring_m / 2.0;
        let plate_h = self.plate_thickness_m;
        let floor_top = -self.floor_top_z();
        let mut xml = String::new();

        if self.lighting {
            xml += &scene_lighting_worldbody_xml();
        }

        if let Some(side) = self.floor_size_m {
            // Venue floor. A slab, not an infinite plane: the ring is what
            // the robot is meant to stay on, and a floor with a visible edge
            // makes that read at a glance.
            const FLOOR_H: f64 = 0.05;
            xml += &format!(
                "    <geom name=\"ring_floor\" type=\"box\" pos=\"0 0 {}\" size=\"{} {} {}\" rgba=\"0.42 0.44 0.47 1\"/>\n",
                floor_top - FLOOR_H / 2.0,
                side / 2.0,
                side / 2.0,
                FLOOR_H / 2.0,
            );
            // The stand the ring sits on, filling floor-to-underside. Inset
            // slightly so the ring reads as a top plate on a base rather
            // than as one solid block.
            let inset = 0.06;
            let h = -plate_h - floor_top;
            if h > 1e-6 {
                xml += &format!(
                    "    <geom name=\"ring_stand\" type=\"box\" pos=\"0 0 {}\" size=\"{} {} {}\" rgba=\"0.20 0.22 0.28 1\"/>\n",
                    floor_top + h / 2.0,
                    half - inset,
                    half - inset,
                    h / 2.0,
                );
            }
        }

        xml += &format!(
            "    <geom name=\"ring_field\" type=\"hfield\" hfield=\"{name}\" pos=\"0 0 0\" rgba=\"0.36 0.37 0.40 1\"/>\n"
        );

        for (label, (w, d), sx, sy, rgba) in [
            ("start_red", self.red_platform_m, -1.0, -1.0, "0.72 0.12 0.12 1"),
            ("start_blue", self.blue_platform_m, 1.0, 1.0, "0.12 0.20 0.72 1"),
        ] {
            // Butted against the outside of the ring edge, on opposite
            // corners, matching the isometric.
            xml += &format!(
                "    <geom name=\"{label}\" type=\"box\" pos=\"{} {} {}\" size=\"{} {} {}\" rgba=\"{rgba}\"/>\n",
                sx * (half + d / 2.0),
                sy * (self.ring_m / 2.0 - w / 2.0),
                -self.platform_h_m / 2.0,
                d / 2.0,
                w / 2.0,
                self.platform_h_m / 2.0,
            );
        }

        // Coloured border along each start side, so which end is which reads
        // at a glance the way the red/blue frames do in the photographs.
        if let Some((bw, bh)) = self.border {
            for (label, sx, rgba) in
                [("border_red", -1.0_f64, "0.72 0.12 0.12 1"), ("border_blue", 1.0, "0.12 0.20 0.72 1")]
            {
                xml += &format!(
                    "    <geom name=\"{label}\" type=\"box\" pos=\"{} 0 {}\" size=\"{} {half} {}\" rgba=\"{rgba}\"/>\n",
                    sx * (half + bw / 2.0),
                    bh / 2.0 - plate_h,
                    bw / 2.0,
                    (bh + plate_h) / 2.0,
                );
            }
        }

        // Safety barrier: four transparent panels, standing on the venue
        // floor and clear of the start platforms. Collidable -- this is the
        // one part of the surroundings a robot can actually hit.
        if let Some((bh, gap, bt)) = self.barrier {
            let r = half + gap;
            for (label, x, y, sx, sy) in [
                ("barrier_xp", r, 0.0, bt / 2.0, r + bt),
                ("barrier_xn", -r, 0.0, bt / 2.0, r + bt),
                ("barrier_yp", 0.0, r, r + bt, bt / 2.0),
                ("barrier_yn", 0.0, -r, r + bt, bt / 2.0),
            ] {
                xml += &format!(
                    "    <geom name=\"{label}\" type=\"box\" pos=\"{x} {y} {}\" size=\"{sx} {sy} {}\" rgba=\"0.55 0.70 0.80 0.25\"/>\n",
                    floor_top + bh / 2.0,
                    bh / 2.0,
                );
            }
        }

        xml
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
        let (x, y) = opts.base_xy.unwrap_or((0.0, 0.0));
        [x, y, root_z]
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
    let xml = match (opts.timestep, integrator) {
        (None, None) => xml,
        (dt, ig) => {
            let mut attrs = String::new();
            if let Some(dt) = dt {
                attrs.push_str(&format!(" timestep=\"{dt}\""));
            }
            if let Some(ig) = ig {
                attrs.push_str(&format!(" integrator=\"{ig}\""));
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
    let xml = match opts.native_velocity_servo {
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
    };

    let xml = match &opts.extra_worldbody_xml {
        None => xml,
        Some(extra) => match xml.rfind("</worldbody>") {
            // `xml[..i]` ends with `</worldbody>`'s own indentation, so
            // splicing there donates it to the first line of `extra` and
            // leaves the closing tag at column zero. Hand it back.
            Some(i) => {
                let head = xml[..i].trim_end_matches(' ');
                let indent = &xml[head.len()..i];
                format!("{head}{extra}{indent}{}", &xml[i..])
            }
            None => {
                log::error!("MJCF export: no </worldbody> to splice extra_worldbody_xml into");
                xml
            }
        },
    };

    // Own `<asset>` block after the opening `<mujoco …>` line, the same
    // splice `<option>` uses above. MJCF merges repeated top-level sections,
    // so this does not disturb the mesh `<asset>` the exporter emits.
    let xml = match &opts.extra_asset_xml {
        None => xml,
        Some(extra) => match xml.find('\n') {
            Some(nl) => format!("{}\n  <asset>\n{extra}  </asset>{}", &xml[..nl], &xml[nl..]),
            None => xml,
        },
    };

    match &opts.extra_visual_xml {
        None => xml,
        Some(extra) => match xml.find('\n') {
            Some(nl) => format!("{}\n  <visual>\n{extra}  </visual>{}", &xml[..nl], &xml[nl..]),
            None => xml,
        },
    }
}

/// Lighting, sky and offscreen-buffer settings for a scene that has to be
/// LOOKED at rather than just stepped.
///
/// MuJoCo's default is a dim headlight on a black void: enough to see one
/// robot in a debug view, not enough to read a 1.9 m competition field, and
/// the reason every render of the ring so far came out nearly black. This is
/// deliberately generic rather than ring-specific -- the staircase wants the
/// same treatment the moment anyone films it.
pub fn scene_lighting_visual_xml() -> String {
    // Offscreen default is 640x480, which silently caps `mujoco.Renderer`.
    "    <headlight ambient=\"0.45 0.45 0.48\" diffuse=\"0.55 0.55 0.55\" specular=\"0.15 0.15 0.15\"/>\n    <rgba haze=\"0.62 0.68 0.76 1\"/>\n    <map znear=\"0.01\" zfar=\"50\"/>\n    <global offwidth=\"1920\" offheight=\"1080\"/>\n"
        .to_string()
}

/// Sky and floor textures to go with [`scene_lighting_visual_xml`].
pub fn scene_lighting_asset_xml() -> String {
    "    <texture name=\"sky\" type=\"skybox\" builtin=\"gradient\" rgb1=\"0.32 0.42 0.55\" rgb2=\"0.08 0.10 0.14\" width=\"256\" height=\"256\"/>\n"
        .to_string()
}

/// Key and fill lights for [`scene_lighting_visual_xml`]. Worldbody-legal.
///
/// Two directional lights from opposite quarters rather than one: a single
/// source leaves the far side of every obstacle in shadow, which on a field
/// made of 15 mm plates is most of what there is to see.
pub fn scene_lighting_worldbody_xml() -> String {
    "    <light name=\"key\" pos=\"2 -2 4\" dir=\"-0.4 0.4 -1\" directional=\"true\" diffuse=\"0.55 0.55 0.55\" specular=\"0.1 0.1 0.1\" castshadow=\"true\"/>\n    <light name=\"fill\" pos=\"-3 2 3\" dir=\"0.6 -0.4 -1\" directional=\"true\" diffuse=\"0.28 0.28 0.32\" specular=\"0 0 0\" castshadow=\"false\"/>\n"
        .to_string()
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
