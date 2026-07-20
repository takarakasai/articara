//! Articara ↔ quadruped-gait integration glue.
//!
//! Two responsibilities:
//!
//! 1. **Auto-detect kinematics**: walk the [`RobotModel`]'s kinematic chain
//!    from each user-provided foot link upward, identify the calf / thigh
//!    / hip joints, and compute the link lengths + hip offset that
//!    [`quadruped_gait::LegKinematics`] requires.
//!
//! 2. **Wrap [`quadruped_gait::GaitController`]** with a joint-index cache
//!    so the per-tick "joint name → joint idx" lookup is O(1) instead of
//!    O(N joint names) when feeding `MujocoSim::set_position_target`.
//!
//! The gait crate itself is kept independent of articara so it can be
//! tested without any GUI / physics dependencies; this module is the
//! adaptor layer.

use std::collections::HashMap;

use nalgebra as na;
use quadruped_gait::{
    AnyGaitController as InnerController, ControllerOutput, GaitConfig,
    GaitGenerator as _, GaitMode, KinematicsConfig,
    LegId, LegKinematics, VelocityCmd,
};

use crate::rbd::model::RobotModel;

/// Default leg-foot link names assumed when the user hasn't customised
/// them in the setup UI. Matches `quadruped_gait::DEFAULT_FOOT_LINKS`
/// but re-exported here so callers don't need a direct gait-crate import.
pub const DEFAULT_FOOT_LINKS: [(LegId, &str); 4] = [
    (LegId::FL, "FL_foot"),
    (LegId::FR, "FR_foot"),
    (LegId::RL, "RL_foot"),
    (LegId::RR, "RR_foot"),
];

/// Walk up the kinematic chain from `child_link` until we've collected
/// `wanted` non-fixed joints. Fixed joints are silently traversed (their
/// translation is folded into the chain by virtue of [`compute_transforms`]).
/// Returns the joint indices in foot-to-body order; an error if the chain
/// runs out of joints first.
fn climb_to_active_joints(
    model: &RobotModel,
    foot_link: &str,
    wanted: usize,
) -> Result<Vec<usize>, String> {
    let mut joints = Vec::with_capacity(wanted);
    let mut current = foot_link.to_string();
    while joints.len() < wanted {
        // Find the joint whose child_link == current (linear scan; the
        // model's joint count is small and this only runs at gait setup).
        let parent_idx = model
            .joints
            .iter()
            .position(|j| j.child_link == current)
            .ok_or_else(|| {
                format!(
                    "no joint with child_link={current:?} (chain too short for {} active joints)",
                    wanted,
                )
            })?;
        let joint = &model.joints[parent_idx];
        if joint.joint_type != "fixed" {
            joints.push(parent_idx);
        }
        current = joint.parent_link.clone();
    }
    Ok(joints)
}

/// Look up a joint's world-frame origin position at q = 0. The model's
/// `compute_transforms` returns link transforms; the joint is at
/// `parent_link_transform · joint.origin`.
fn joint_world_pos(
    model: &RobotModel,
    transforms: &HashMap<String, na::Isometry3<f32>>,
    joint_idx: usize,
) -> na::Vector3<f64> {
    let joint = &model.joints[joint_idx];
    let parent_t = transforms
        .get(&joint.parent_link)
        .copied()
        .unwrap_or_else(na::Isometry3::identity);
    let world = parent_t * joint.origin;
    world.translation.vector.cast::<f64>()
}

/// Resolve a joint's rotation axis into the **body root's frame**.
///
/// The URDF / .misa axis vector is expressed in the joint's own frame
/// (which is the child link's frame at q = 0). A joint whose origin
/// declares `rpy="0 0 π/2"` and `axis="1 0 0"` therefore rotates around
/// the parent's **Y** axis, not X — and the auto-detect axis classifier
/// needs to see the resolved body-frame direction, otherwise valid RPP
/// URDFs that spell their pitch axes that way (keel does) get rejected.
fn joint_axis_in_body(
    model: &RobotModel,
    joint_idx: usize,
    transforms_rest: &HashMap<String, na::Isometry3<f32>>,
) -> na::Vector3<f64> {
    let joint = &model.joints[joint_idx];
    let parent_rot = transforms_rest
        .get(&joint.parent_link)
        .map(|tf| tf.rotation)
        .unwrap_or_else(na::UnitQuaternion::identity);
    let joint_frame_rot = parent_rot * joint.origin.rotation;
    (joint_frame_rot * joint.axis).cast::<f64>()
}

/// Compute every link's world transform with **all joint angles = 0**.
///
/// `RobotModel::compute_transforms()` honours `joint_positions`, so a
/// model whose home pose is non-zero would feed rotated link frames into
/// `joint_axis_in_body`. Auto-detect needs the rest-pose axes specifically
/// (the URDF declares them at q = 0), so this helper temporarily zeroes
/// the cache.
fn compute_transforms_at_rest(
    model: &RobotModel,
) -> HashMap<String, na::Isometry3<f32>> {
    let mut clone = model.clone();
    for q in &mut clone.joint_positions {
        *q = 0.0;
    }
    clone.compute_transforms()
}

/// Auto-detect [`LegKinematics`] for one leg from a foot link name.
///
/// Walks up exactly three non-fixed joints (calf → thigh → hip) and
/// derives:
///
/// - hip_joint / thigh_joint / calf_joint names
/// - hip_offset: hip joint position in the body root's frame
/// - hip_to_thigh_y: lateral distance from hip to thigh axis
/// - upper_leg_m, lower_leg_m: link lengths from the q = 0 transforms
/// - nominal_foot_body: foot link's q = 0 world position (in body frame)
///
/// The default fully-extended `nominal_foot_body` may need a manual
/// override after detection (the runtime gait controller wants some
/// retraction so the IK has swing headroom). The host's gait UI exposes
/// that override.
pub fn auto_detect_leg_kinematics(
    model: &RobotModel,
    foot_link: &str,
    leg: LegId,
) -> Result<LegKinematics, String> {
    if !model.link_map.contains_key(foot_link) {
        return Err(format!("foot link {foot_link:?} not found in model"));
    }
    let chain = climb_to_active_joints(model, foot_link, 3)?;
    let calf_idx = chain[0];
    let thigh_idx = chain[1];
    let hip_idx = chain[2];

    // Joint axes are stored in the joint's own frame (= child link's
    // frame at q=0), so a thigh that lives behind a `<joint origin rpy="0
    // 0 π/2">` and declares `axis="1 0 0"` actually rotates around the
    // *parent's* Y axis. Resolve every axis into the body (root-link)
    // frame before classifying, otherwise correctly-built URDFs that
    // happen to spell their pitch axis via an origin rotation get
    // rejected for "doesn't look like a Pitch (Y) axis".
    let transforms_rest = compute_transforms_at_rest(model);
    let hip_axis = joint_axis_in_body(model, hip_idx, &transforms_rest);
    let thigh_axis = joint_axis_in_body(model, thigh_idx, &transforms_rest);
    let calf_axis = joint_axis_in_body(model, calf_idx, &transforms_rest);

    // Sanity-check axes: hip should be predominantly along X (roll),
    // thigh and calf along Y (pitch). Reject other layouts so the
    // analytical IK doesn't silently produce garbage angles.
    if hip_axis.x.abs() < hip_axis.y.abs() || hip_axis.x.abs() < hip_axis.z.abs() {
        return Err(format!(
            "hip joint {} axis {:?} doesn't look like a Roll (X) axis — \
             quadruped-gait's analytical IK assumes RPP topology",
            model.joints[hip_idx].name, hip_axis,
        ));
    }
    if thigh_axis.y.abs() < thigh_axis.x.abs() || thigh_axis.y.abs() < thigh_axis.z.abs() {
        return Err(format!(
            "thigh joint {} axis {:?} doesn't look like a Pitch (Y) axis",
            model.joints[thigh_idx].name, thigh_axis,
        ));
    }
    if calf_axis.y.abs() < calf_axis.x.abs() || calf_axis.y.abs() < calf_axis.z.abs() {
        return Err(format!(
            "calf joint {} axis {:?} doesn't look like a Pitch (Y) axis",
            model.joints[calf_idx].name, calf_axis,
        ));
    }

    let transforms = model.compute_transforms();
    let hip_pos = joint_world_pos(model, &transforms, hip_idx);
    let thigh_pos = joint_world_pos(model, &transforms, thigh_idx);
    let calf_pos = joint_world_pos(model, &transforms, calf_idx);
    let foot_pos: na::Vector3<f64> = transforms
        .get(foot_link)
        .copied()
        .unwrap_or_else(na::Isometry3::identity)
        .translation
        .vector
        .cast::<f64>();

    // Body root: ancestor of the hip joint. Needed only if the URDF puts
    // the root at a non-origin pose; usually the root is at the world
    // origin with model.base_transform identity, so this is a no-op.
    let body_link_name = model.joints[hip_idx].parent_link.clone();
    let body_pos: na::Vector3<f64> = transforms
        .get(&body_link_name)
        .copied()
        .unwrap_or_else(na::Isometry3::identity)
        .translation
        .vector
        .cast::<f64>();

    let hip_offset = hip_pos - body_pos;
    let hip_to_thigh_y = (thigh_pos.y - hip_pos.y).abs();
    let upper_leg = (thigh_pos - calf_pos).norm();
    let lower_leg = (calf_pos - foot_pos).norm();

    if upper_leg < 1e-6 || lower_leg < 1e-6 {
        return Err(format!(
            "degenerate link lengths: upper={upper_leg:.6} lower={lower_leg:.6} \
             — check that the foot link and joint frames don't all coincide",
        ));
    }

    // Build with the auto-derived fields. `LegKinematics::new` computes a
    // default `nominal_foot_body` at the fully-extended pose; override it
    // to the actual q = 0 foot position so the gait controller's stance
    // line lives in the URDF's neutral plane.
    let mut kin = LegKinematics::new(
        leg,
        model.joints[hip_idx].name.clone(),
        model.joints[thigh_idx].name.clone(),
        model.joints[calf_idx].name.clone(),
        foot_link.to_string(),
        hip_offset,
        hip_to_thigh_y,
        upper_leg,
        lower_leg,
    );
    kin.nominal_foot_body = foot_pos - body_pos;
    Ok(kin)
}

/// Auto-detect a complete [`KinematicsConfig`] for all four legs given
/// their foot link names. Returns a list of (LegId, error message) pairs
/// for any leg whose detection failed; the caller (UI) can surface them.
pub fn auto_detect_kinematics_config(
    model: &RobotModel,
    foot_links: &[(LegId, &str); 4],
) -> Result<KinematicsConfig, Vec<(LegId, String)>> {
    let mut errors = Vec::new();
    let mut detected: [Option<LegKinematics>; 4] = [None, None, None, None];
    for (leg, name) in foot_links.iter() {
        match auto_detect_leg_kinematics(model, name, *leg) {
            Ok(kin) => detected[slot_of(*leg)] = Some(kin),
            Err(e) => errors.push((*leg, e)),
        }
    }
    if !errors.is_empty() {
        return Err(errors);
    }
    Ok(KinematicsConfig {
        fl: detected[0].clone().unwrap(),
        fr: detected[1].clone().unwrap(),
        rl: detected[2].clone().unwrap(),
        rr: detected[3].clone().unwrap(),
    })
}

fn slot_of(id: LegId) -> usize {
    match id {
        LegId::FL => 0,
        LegId::FR => 1,
        LegId::RL => 2,
        LegId::RR => 3,
    }
}

/// Auto-detect a per-robot [`SrbdMpcConfig`] from the model's link
/// inertials, leaving all weights / horizon / friction at defaults.
///
/// - `mass_kg`: sum of every link's mass. The SRBD MPC treats the
///   robot as a single rigid body, so the legs' mass shows up in the
///   body lumped term. (For trotting robots the legs are typically
///   <30% of total mass and this approximation is adequate; a future
///   refinement could subtract the swing-leg contribution.)
/// - `inertia_diag_body`: the **heaviest link's** body-frame moment
///   of inertia diagonal. Heaviest-link is more robust than
///   `root_link` because some URDFs (notably namiashi) split the
///   structural root from the inertial root — the named "trunk" can
///   carry zero mass while a child link holds the real inertia. Using
///   `argmax(mass)` lands on the inertial carrier in both layouts.
///   Coarse approximation: the true SRBD inertia would compose all
///   link inertias via the parallel-axis theorem at a nominal pose.
///
/// If no link has positive mass / inertia, the corresponding field
/// falls back to the [`SrbdMpcConfig::default`] value so a degenerate
/// URDF still yields a usable (if poorly-tuned) config rather than
/// panicking.
///
/// Use this for any model whose mass / inertia is known from the
/// URDF; for hand-tuned setups call
/// [`GaitController::set_srbd_mpc_config`] directly with custom
/// values.
pub fn auto_detect_srbd_mpc_config(
    model: &RobotModel,
) -> quadruped_gait::SrbdMpcConfig {
    let mut cfg = quadruped_gait::SrbdMpcConfig::default();

    let mass_total: f64 = model.links.iter().map(|l| l.inertial.mass).sum();
    if mass_total > 1e-6 {
        cfg.mass_kg = mass_total;
    }

    // Find the heaviest link and read its inertia diagonal. Reject
    // near-zero diagonals — the SRBD MPC's QP becomes ill-conditioned
    // with ~0 diagonal entries.
    let heaviest = model
        .links
        .iter()
        .max_by(|a, b| {
            a.inertial
                .mass
                .partial_cmp(&b.inertial.mass)
                .unwrap_or(std::cmp::Ordering::Equal)
        });
    if let Some(link) = heaviest {
        let i = &link.inertial;
        let diag = na::Vector3::new(i.ixx, i.iyy, i.izz);
        if diag.iter().all(|&x| x > 1e-9) {
            cfg.inertia_diag_body = diag;
        }
    }
    cfg
}

/// Auto-fill a [`quadruped_gait::CentroidalMpcConfig`] from the URDF.
/// Sibling of [`auto_detect_srbd_mpc_config`] that targets the
/// centroidal-SRBD MPC.
///
/// - `mass_kg`           : sum over all links' inertials.
/// - `centroidal_inertia_body` : 3×3 angular block of the centroidal
///   composite-rigid-body inertia at q = 0 (`misarta::centroidal::
///   compute_centroidal_inertia`'s top-left 3×3).
/// - `com_offset_body`   : aggregate CoM relative to body root, body
///   frame, at q = 0. The dominant correction the centroidal model
///   provides over the body-root SRBD.
///
/// Used by [`GaitController::build`] to populate the centroidal MPC
/// config so that mode-switching to `GaitMode::CentroidalSrbd` is
/// instant.
pub fn auto_detect_centroidal_mpc_config(
    model: &RobotModel,
) -> quadruped_gait::CentroidalMpcConfig {
    let mut cfg = quadruped_gait::CentroidalMpcConfig::default();

    let mass_total: f64 = model.links.iter().map(|l| l.inertial.mass).sum();
    if mass_total > 1e-6 {
        cfg.mass_kg = mass_total;
    }

    // CoM offset: aggregate CoM at q = 0 minus body root, expressed
    // in body frame (= world frame at q = 0 with identity base).
    let transforms = model.compute_transforms();
    let body_pos: na::Vector3<f64> = transforms
        .get(&model.root_link)
        .map(|t| t.translation.vector.cast::<f64>())
        .unwrap_or_else(na::Vector3::zeros);
    let mut p_com_weighted = na::Vector3::<f64>::zeros();
    let mut total_m = 0.0_f64;
    for link in &model.links {
        if link.inertial.mass <= 0.0 {
            continue;
        }
        let Some(t_link_world) = transforms.get(&link.name) else {
            continue;
        };
        let com_local = link.inertial.origin.translation.vector.cast::<f64>();
        let r_link = t_link_world.rotation.to_rotation_matrix();
        let r_link_f64 = r_link.matrix().cast::<f64>();
        let t_link = t_link_world.translation.vector.cast::<f64>();
        let com_world = r_link_f64 * com_local + t_link;
        p_com_weighted += link.inertial.mass * com_world;
        total_m += link.inertial.mass;
    }
    if total_m > 1e-6 {
        cfg.com_offset_body = (p_com_weighted / total_m) - body_pos;
    }

    // Centroidal angular inertia: take misarta's centroidal CRBI's
    // 3×3 angular block at q = 0. For type-1 SRBD this stays constant
    // across the horizon; the host can override later for a different
    // nominal pose.
    if let Some(mc) = model.misarta_cache.as_ref() {
        let q = mc.model.neutral_q();
        let i6 = misarta::centroidal::compute_centroidal_inertia(&mc.model, &q);
        let mut i_ang = nalgebra::Matrix3::<f64>::zeros();
        for r in 0..3 {
            for c in 0..3 {
                i_ang[(r, c)] = i6[(r, c)];
            }
        }
        // Skip near-singular results (degenerate URDF). Default value
        // stays in `cfg.centroidal_inertia_body`.
        if i_ang.determinant().abs() > 1e-9 {
            cfg.centroidal_inertia_body = i_ang;
        }
    }

    // SQP iterations: 3 = legged_control-style sweet spot. Empirical
    // namiashi comparison (regression suite + viewport):
    //   SQP=1 → forward dx +0.151 (weak), yaw +1.599
    //   SQP=3 → forward dx +0.777 (≈ ideal +0.75), yaw +1.599 ✓
    //   SQP=5 → no measurable improvement over 3 (convergence verified
    //           by `mpc_sqp_3_iters_match_5_iters` unit test)
    // The 5x forward dx improvement at SQP=3 was initially mistaken
    // for a regression because the body now visibly tracks fast
    // forward cmd while still showing the residual cross-coupling
    // that exists at SQP=1; once user verified it as a genuine
    // tracking-magnitude win, default reverted to 3.
    cfg.sqp_iterations = 3;

    cfg
}

/// Sibling of [`auto_detect_centroidal_mpc_config`] for the 24-state
/// full-centroidal MPC. Populates mass + centroidal inertia + CoM
/// offset from the URDF just like the 12-state version, then carries
/// the per-leg `KinematicsConfig` directly into the config (the
/// FullCentroidalMpc uses it for per-node FK).
pub fn auto_detect_full_centroidal_mpc_config(
    model: &RobotModel,
    kin: &quadruped_gait::KinematicsConfig,
) -> quadruped_gait::FullCentroidalMpcConfig {
    // Build the 12-state-equivalent first so we share the physical
    // parameter detection logic, then copy into the 24-state config
    // shape with kinematics added.
    let cent = auto_detect_centroidal_mpc_config(model);
    let mut cfg = quadruped_gait::FullCentroidalMpcConfig::default_with_kin(kin.clone());
    cfg.mass_kg = cent.mass_kg;
    cfg.centroidal_inertia_body = cent.centroidal_inertia_body;
    cfg.com_offset_body = cent.com_offset_body;
    cfg.friction_mu = cent.friction_mu;
    cfg.max_normal_force = cent.max_normal_force;
    cfg.horizon_steps = cent.horizon_steps;
    cfg.dt_per_step = cent.dt_per_step;
    cfg.sqp_iterations = cent.sqp_iterations;
    // q_diag, r_diag, kinematics retained from default_with_kin —
    // they have no analogue in the 12-state cfg (12 vs 24 entries).

    // True-centroidal-coupling data (desk-research gap ①): prepared
    // whenever a misarta model is available so the controller can flip
    // `enable_true_centroidal_coupling` on cheaply later; `None` (and
    // the flag defaulting `false`) leaves today's dynamics untouched.
    if let Some(mc) = model.misarta_cache.as_ref() {
        match quadruped_gait::auto_detect_true_centroidal_coupling(&mc.model, kin) {
            Ok(data) => cfg.true_centroidal_coupling_data = Some(data),
            Err(e) => {
                eprintln!("auto_detect_true_centroidal_coupling: {e} — coupling stays disabled");
            }
        }
    }
    cfg
}

/// Wrapper around [`InnerController`] (= `quadruped_gait::GaitController`)
/// that caches the joint-name → RobotModel-joint-idx mapping. The cache
/// Source for the body pose observation feeding the gait controller's
/// MPC. The host picks one each tick and forwards
/// `(yaw, world_position)` via [`GaitController::set_body_pose_observed`].
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum PoseSource {
    /// Yaw from a Madgwick attitude estimator fed by MuJoCo's IMU
    /// sensor; position from MuJoCo's `xpos` ground truth (Madgwick is
    /// attitude-only, so position needs another source). Closer in
    /// behaviour to a real-robot stack but still uses the sim's
    /// position oracle — switch to [`PoseSource::LegOdometry`] for the
    /// fully-real pipeline.
    #[default]
    ImuFusion,
    /// Yaw from Madgwick, position from kinematics-based leg odometry
    /// (stance-foot-pinning + ω×r + leg Jacobian — see
    /// [`crate::leg_odometry`]). End-to-end matches what a real
    /// quadruped would compute on hardware (encoders + IMU only, no
    /// external positioning).
    LegOdometry,
    /// Yaw + position both from MuJoCo's `xquat` / `xpos` ground truth.
    /// Sim-only oracle — useful as a baseline when debugging the
    /// estimator-based paths.
    GroundTruth,
}

impl PoseSource {
    /// Short user-facing label for the picker UI.
    pub fn label(self) -> &'static str {
        match self {
            PoseSource::ImuFusion => "IMU + Madgwick (pos from sim)",
            PoseSource::LegOdometry => "IMU + leg odometry",
            PoseSource::GroundTruth => "MuJoCo ground truth",
        }
    }
    pub const ALL: [PoseSource; 3] = [
        PoseSource::ImuFusion,
        PoseSource::LegOdometry,
        PoseSource::GroundTruth,
    ];
}

/// matters because the gait controller emits 12 (joint_name, q) pairs per
/// tick, and converting each name to a joint index by hash lookup on the
/// MuJoCo sim is wasteful when the mapping is known to be stable.
pub struct GaitController {
    inner: InnerController,
    /// `[hip_idx, thigh_idx, calf_idx]` per leg in canonical FL/FR/RL/RR order.
    joint_indices: [[usize; 3]; 4],
    /// `[hip_sign, thigh_sign, calf_sign]` per leg — sign multiplier
    /// applied at the IK→MuJoCo handover.
    ///
    /// Why this is needed: the analytical IK in `quadruped_gait::solve_leg_ik`
    /// follows a "positive q_thigh tilts thigh forward" convention. URDF +Y
    /// pitch joints follow the *opposite* convention by the right-hand rule
    /// (`R_y(q)·(0,0,-1) = (-sin q, 0, -cos q)` — positive q sends a
    /// downward vector toward −X). So when the URDF axis is `(0, +1, 0)`
    /// we need to negate q at the boundary; for `(0, -1, 0)` it's already
    /// correct. Hip rolls about ±X follow the same pattern.
    ///
    /// `auto_detect_*` fills in the right signs from the URDF axis values.
    joint_signs: [[f64; 3]; 4],
    /// Whether the controller is currently driving the sim. Off by default
    /// so opening a model + creating a controller doesn't accidentally
    /// command motion.
    enabled: bool,
}

impl GaitController {
    /// Build the wrapper from a `RobotModel`, an auto-detected (or
    /// manually constructed) [`KinematicsConfig`], a [`GaitConfig`],
    /// and the initial [`GaitMode`] (CHAMP or MPC).
    /// Returns an error if any of the joint names in the kinematics
    /// config can't be resolved in the model — typically a sign that
    /// the user's foot link name wasn't right for that robot.
    pub fn build(
        model: &RobotModel,
        kin: KinematicsConfig,
        cfg: GaitConfig,
        mode: GaitMode,
    ) -> Result<Self, String> {
        let mut joint_indices = [[0usize; 3]; 4];
        let mut joint_signs = [[1.0_f64; 3]; 4];
        for (slot, leg_kin) in [&kin.fl, &kin.fr, &kin.rl, &kin.rr].iter().enumerate() {
            let names = [
                &leg_kin.hip_joint,
                &leg_kin.thigh_joint,
                &leg_kin.calf_joint,
            ];
            // Expected dominant axis component per joint (in IK convention):
            // hip about +X, thigh and calf about +Y. We read each joint's
            // actual axis from the model and use its sign (+1 / −1) as a
            // multiplier so the URDF's right-hand-rule rotation lines up
            // with the IK's intent. See the doc comment on `joint_signs`.
            // For hip the IK already uses URDF convention, so we only need
            // to flip when axis.x is negative; for thigh/calf the IK uses
            // the *opposite* of URDF's right-hand rule about +Y, so a +Y
            // axis means we negate (and a −Y axis means we don't).
            let axis_components = [
                |a: na::Vector3<f32>| a.x,
                |a: na::Vector3<f32>| a.y,
                |a: na::Vector3<f32>| a.y,
            ];
            // For thigh and calf, IK output sign is the *opposite* of the
            // URDF axis sign — `+1` axis ⇒ multiply IK output by `-1`.
            let ik_to_urdf_factor = [1.0, -1.0, -1.0];
            for (k, name) in names.iter().enumerate() {
                let idx = *model.joint_map.get(name.as_str()).ok_or_else(|| {
                    format!(
                        "joint {name:?} (referenced by gait kinematics) not in model"
                    )
                })?;
                joint_indices[slot][k] = idx;
                let comp = axis_components[k](model.joints[idx].axis) as f64;
                let urdf_sign = if comp >= 0.0 { 1.0 } else { -1.0 };
                joint_signs[slot][k] = ik_to_urdf_factor[k] * urdf_sign;
            }
        }
        let mut inner = InnerController::new(mode, cfg, kin);
        // Auto-tune the SRBD MPC body-mass / inertia for *this* robot.
        // The default 9 kg / Cheetah-3 inertia is wildly off for most
        // legged robots; running with the default would scale every
        // predicted GRF by the mass ratio and produce a huge τ_ff that
        // either flails the legs or launches the body.
        //
        // We populate **both** SRBD and centroidal MPC configs so that
        // `set_mode` switches between Mpc / CentroidalSrbd at runtime
        // without needing a re-build. The `set_*_mpc_config` calls are
        // no-ops in modes that don't carry that MPC type (CHAMP /
        // wrong-MPC-mode), so this is safe regardless of the active
        // mode at build time.
        inner.set_srbd_mpc_config(auto_detect_srbd_mpc_config(model));
        inner.set_centroidal_mpc_config(auto_detect_centroidal_mpc_config(model));
        inner.set_full_centroidal_mpc_config(
            auto_detect_full_centroidal_mpc_config(model, inner.kinematics()),
        );
        // Re-apply the gait config so flags that the FullCentroidal
        // controller mirrors onto its MPC config (A3 friction_cone_soft,
        // B3 warm_start, A1 mpc_optimized_footstep / q_foot_xy_world)
        // survive the `set_full_centroidal_mpc_config` overwrite above.
        // Without this round-trip the inner controller holds the
        // caller's GaitConfig but the MPC silently runs with the
        // auto-detected defaults — every A1/A3/B3 toggle would be a
        // no-op at build time.
        let cfg_clone = inner.config().clone();
        inner.set_config(cfg_clone);
        Ok(Self {
            inner,
            joint_indices,
            joint_signs,
            enabled: false,
        })
    }

    /// Currently-active generator mode (CHAMP / MPC).
    pub fn mode(&self) -> GaitMode {
        self.inner.mode()
    }

    /// Switch generator mode at runtime. Preserves the velocity
    /// command, gait config, and per-leg knee_forward so the user
    /// experiences a clean handoff. The phase / body integrator state
    /// is reset because the two controllers don't share that
    /// representation — gait restarts at cycle origin.
    pub fn set_mode(&mut self, mode: GaitMode) {
        self.inner.set_mode(mode);
    }

    /// Feed the latest observed body linear and angular velocity
    /// (world frame) from the host's state estimator. Linear feeds
    /// the MPC's capture-point term; angular feeds the SRBD MPC's
    /// `s_now.angular_velocity` so the yaw-rate error against the
    /// reference is real (otherwise in-place rotation can't be
    /// driven). CHAMP ignores both.
    pub fn set_body_state_observed(
        &mut self,
        v_world: nalgebra::Vector3<f64>,
        omega_world: nalgebra::Vector3<f64>,
    ) {
        self.inner.set_body_state_observed(v_world, omega_world);
    }

    /// Feed the latest observed body pose (world-frame yaw + position)
    /// from the host's state estimator (e.g. Madgwick yaw + leg-odom
    /// position, or MuJoCo ground truth). Replaces the controller's
    /// command-integrated `body_state` so the SRBD MPC and footstep
    /// planner reason about the *real* pose. CHAMP ignores it.
    pub fn set_body_pose_observed(
        &mut self,
        world_yaw: f64,
        world_position: nalgebra::Vector3<f64>,
    ) {
        self.inner.set_body_pose_observed(world_yaw, world_position);
    }

    /// Latest SRBD-MPC predicted ground reaction forces (world
    /// frame, per foot, in canonical FL/FR/RL/RR slot order). Returns
    /// `None` when the active mode is CHAMP or the MPC hasn't ticked
    /// yet. Used by the viewport's GRF overlay; not currently fed
    /// back into MuJoCo (Phase 4 work).
    pub fn predicted_grfs(&self) -> Option<&quadruped_gait::MpcSolution> {
        self.inner.predicted_grfs()
    }

    /// Override the SRBD MPC's body-mass / inertia / weight matrices.
    /// No-op when the active mode is CHAMP. Hosts that know their
    /// robot's actual mass should call this once after `build` — the
    /// default 9 kg / Cheetah-3 inertia is wildly off for most robots
    /// and produces grossly over- or under-scaled τ_ff.
    pub fn set_srbd_mpc_config(
        &mut self,
        cfg: quadruped_gait::SrbdMpcConfig,
    ) {
        self.inner.set_srbd_mpc_config(cfg);
    }

    /// Override the SRBD MPC's capture-point feedback gain. No-op
    /// outside [`quadruped_gait::GaitMode::Mpc`]. Pass `0.0` to disable
    /// the closed-loop footstep correction.
    pub fn set_capture_point_gain(&mut self, k: f64) {
        self.inner.set_capture_point_gain(k);
    }

    /// Enable/disable solving the MPC QP on a background thread. Off by
    /// default (synchronous, deterministic) so headless examples / the
    /// walk-stability tests keep their exact behaviour. The GUI turns it
    /// on after `build` so a slow solve (full-centroidal ≈ 0.4 s) can't
    /// stall the eframe update loop and freeze the window.
    pub fn set_async_mpc(&mut self, enabled: bool) {
        self.inner.set_async_mpc(enabled);
    }

    /// Configure the FullCentroidal controller's nonlinear pulse
    /// branch of the capture-point feedback. `k_pulse` is the slope
    /// applied to `(|v_err| − v_db) · sign(v_err)`; `v_db` is the
    /// deadband below which the pulse contributes 0. No-op outside
    /// FullCentroidal mode. See
    /// [`quadruped_gait::capture_point_step`].
    pub fn set_capture_point_pulse(&mut self, k_pulse: f64, v_db: f64) {
        self.inner.set_capture_point_pulse(k_pulse, v_db);
    }

    pub fn capture_point_pulse(&self) -> Option<(f64, f64)> {
        self.inner.capture_point_pulse()
    }

    /// Activate the FullCentroidal controller's goal-pose mode. After
    /// this call, [`Self::tick`] computes the body velocity command at
    /// each tick from `goal − observed_pose`, so the robot actively
    /// recovers its position after a disturbance — the
    /// `goalToTargetTrajectories` analogue from legged_control.
    /// `set_velocity_cmd` implicitly clears the goal. No-op outside
    /// FullCentroidal mode.
    pub fn set_goal_pose_world(
        &mut self,
        goal: quadruped_gait::GoalPoseWorld,
    ) {
        self.inner.set_goal_pose_world(goal);
    }
    pub fn clear_goal_pose(&mut self) {
        self.inner.clear_goal_pose();
    }
    pub fn goal_pose_world(&self) -> Option<quadruped_gait::GoalPoseWorld> {
        self.inner.goal_pose_world()
    }

    /// Toggle the "use MPC-predicted base for footstep target" path
    /// on the FullCentroidal controller (legged_control-style foot
    /// placement against MPC's body prediction). No-op outside
    /// FullCentroidal mode.
    pub fn set_use_mpc_predicted_footstep(&mut self, enable: bool) {
        self.inner.set_use_mpc_predicted_footstep(enable);
    }
    pub fn use_mpc_predicted_footstep(&self) -> Option<bool> {
        self.inner.use_mpc_predicted_footstep()
    }

    /// Toggle the FullCentroidal controller's per-horizon-step dynamic
    /// joint_q reference (samples the open-loop swing/stance foot curve
    /// at each horizon step's projected phase instead of holding
    /// joint_q flat). No-op outside FullCentroidal mode; requires
    /// `legged_control_parity` to have an effect.
    pub fn set_dynamic_joint_q_reference(&mut self, enable: bool) {
        self.inner.set_dynamic_joint_q_reference(enable);
    }
    pub fn dynamic_joint_q_reference(&self) -> Option<bool> {
        self.inner.dynamic_joint_q_reference()
    }

    /// Toggle the FullCentroidal controller's task-space→joint-space
    /// `R` weight mapping for `joint_v` (legged_control/OCS2's own
    /// technique — see the quadruped-gait doc comment). No-op outside
    /// FullCentroidal mode. `None` reverts to the flat per-joint
    /// diagonal `r_diag`.
    pub fn set_task_space_joint_vel_weight(&mut self, r_taskspace: Option<[f64; 3]>) {
        self.inner.set_task_space_joint_vel_weight(r_taskspace);
    }
    pub fn task_space_joint_vel_weight(&self) -> Option<[f64; 3]> {
        self.inner.task_space_joint_vel_weight()
    }

    /// Toggle the FullCentroidal controller's true-centroidal-coupling
    /// bias term (desk-research gap ① — see
    /// `quadruped_gait::FullCentroidalMpcConfig`'s doc comment). No-op
    /// outside FullCentroidal mode, and a no-op even in FullCentroidal
    /// mode if no `misarta` model was available at auto-detect time
    /// (`true_centroidal_coupling_data` stayed `None`).
    pub fn set_true_centroidal_coupling(&mut self, enable: bool) {
        self.inner.set_true_centroidal_coupling(enable);
    }
    pub fn true_centroidal_coupling(&self) -> Option<bool> {
        self.inner.true_centroidal_coupling()
    }

    /// Toggle the FullCentroidal controller's closed-form Bound trim
    /// reference (see `ref/wbc_comparison.md` Sec.5bb/5bc, local doc).
    /// No-op outside FullCentroidal mode, and a no-op even in
    /// FullCentroidal mode unless the active `GaitConfig::gait_type`
    /// is `GaitType::Bound`.
    pub fn set_bound_trim_reference(&mut self, enable: bool) {
        self.inner.set_bound_trim_reference(enable);
    }
    pub fn bound_trim_reference(&self) -> Option<bool> {
        self.inner.bound_trim_reference()
    }

    /// The experimental research knobs of the active controller, as
    /// declared by [`quadruped_gait::exp`]. The GUI renders its
    /// "Experimental flags" section from this metadata, so knobs added
    /// in quadruped-gait appear without host-side changes.
    pub fn experimental_keys(&self) -> &'static [quadruped_gait::ExpKey] {
        self.inner.experimental_keys()
    }

    /// Read an experimental knob (see [`Self::experimental_keys`]).
    pub fn get_experimental(&self, key: &str) -> Option<quadruped_gait::ExpValue> {
        self.inner.get_experimental(key)
    }

    /// Write an experimental knob (see [`Self::experimental_keys`]).
    pub fn set_experimental(
        &mut self,
        key: &str,
        value: quadruped_gait::ExpValue,
    ) -> Result<(), quadruped_gait::ExpError> {
        self.inner.set_experimental(key, value)
    }

    /// Snapshot the current experimental knobs as a named preset
    /// (see [`quadruped_gait::ExpPreset`]).
    pub fn snapshot_experimental(&self, name: &str) -> quadruped_gait::ExpPreset {
        self.inner.snapshot_experimental(name)
    }

    /// Apply a saved preset. Returns the number of knobs applied and
    /// the keys skipped (unknown to this mode / controller version).
    pub fn apply_experimental(
        &mut self,
        preset: &quadruped_gait::ExpPreset,
    ) -> (usize, Vec<String>) {
        self.inner.apply_experimental(preset)
    }

    /// Read the active SRBD MPC config. `None` when running CHAMP.
    pub fn srbd_mpc_config(
        &self,
    ) -> Option<&quadruped_gait::SrbdMpcConfig> {
        self.inner.srbd_mpc_config()
    }

    /// Override the centroidal-SRBD MPC config. No-op outside
    /// [`quadruped_gait::GaitMode::CentroidalSrbd`].
    pub fn set_centroidal_mpc_config(
        &mut self,
        cfg: quadruped_gait::CentroidalMpcConfig,
    ) {
        self.inner.set_centroidal_mpc_config(cfg);
    }

    /// Read the active centroidal MPC config. `None` outside
    /// [`quadruped_gait::GaitMode::CentroidalSrbd`].
    pub fn centroidal_mpc_config(
        &self,
    ) -> Option<&quadruped_gait::CentroidalMpcConfig> {
        self.inner.centroidal_mpc_config()
    }

    /// Override the 24-state full-centroidal MPC config. No-op outside
    /// [`quadruped_gait::GaitMode::FullCentroidal`].
    pub fn set_full_centroidal_mpc_config(
        &mut self,
        cfg: quadruped_gait::FullCentroidalMpcConfig,
    ) {
        self.inner.set_full_centroidal_mpc_config(cfg);
    }

    /// Read the active full-centroidal MPC config. `None` outside
    /// [`quadruped_gait::GaitMode::FullCentroidal`].
    pub fn full_centroidal_mpc_config(
        &self,
    ) -> Option<&quadruped_gait::FullCentroidalMpcConfig> {
        self.inner.full_centroidal_mpc_config()
    }

    /// Enable / disable the legged_control-parity path on the
    /// FullCentroidal controller (no-op for other gait modes). When on,
    /// the MPC contact schedule is built from a per-leg per-step phase
    /// projection and each swing-leg-step receives a planned vertical
    /// foot velocity that the QP enforces via the
    /// `NormalVelocityConstraintCppAd`-equivalent equality.
    pub fn set_legged_control_parity(&mut self, enable: bool) {
        self.inner.set_legged_control_parity(enable);
    }

    /// Read the parity flag. `None` outside FullCentroidal mode.
    pub fn legged_control_parity(&self) -> Option<bool> {
        self.inner.legged_control_parity()
    }

    /// Toggle whether the parity path uses the URDF nominal pose for
    /// the joint_q tracking reference (β experiment — matches
    /// legged_control's `DEFAULT_JOINT_STATE`). No-op outside
    /// FullCentroidal mode.
    pub fn set_parity_use_nominal_q_ref(&mut self, enable: bool) {
        self.inner.set_parity_use_nominal_q_ref(enable);
    }

    pub fn parity_use_nominal_q_ref(&self) -> Option<bool> {
        self.inner.parity_use_nominal_q_ref()
    }

    pub fn is_enabled(&self) -> bool {
        self.enabled
    }

    pub fn enable(&mut self) {
        self.enabled = true;
    }

    pub fn disable(&mut self) {
        self.enabled = false;
        // Reset the underlying phase so the next start begins from cycle
        // origin instead of mid-stride.
        self.inner.reset();
    }

    pub fn set_velocity_cmd(&mut self, cmd: VelocityCmd) {
        self.inner.set_velocity_cmd(cmd);
    }

    pub fn velocity_cmd(&self) -> VelocityCmd {
        self.inner.velocity_cmd()
    }

    pub fn config(&self) -> &GaitConfig {
        self.inner.config()
    }

    pub fn set_config(&mut self, cfg: GaitConfig) {
        self.inner.set_config(cfg);
    }

    pub fn kinematics(&self) -> &KinematicsConfig {
        self.inner.kinematics()
    }

    /// Replace the kinematics config on the inner controller. Joint names
    /// must match the originally-built config so the cached
    /// `joint_indices` / `joint_signs` stay valid; only fields that don't
    /// change those (e.g. `nominal_foot_body`, link lengths) should be
    /// modified between calls.
    pub fn set_kinematics(&mut self, kin: KinematicsConfig) {
        self.inner.set_kinematics(kin);
    }

    /// `[hip_idx, thigh_idx, calf_idx]` per leg in canonical FL/FR/RL/RR
    /// order. Indices are into the `RobotModel::joints` array. Used by
    /// the host to look up MuJoCo joint state (q, q̇) for the leg-
    /// odometry estimator without re-doing the name→index lookup that
    /// `GaitController::build` already performed.
    pub fn joint_indices(&self) -> [[usize; 3]; 4] {
        self.joint_indices
    }

    /// `[hip_sign, thigh_sign, calf_sign]` per leg — the sign multiplier
    /// applied at the IK ↔ URDF handover. The host applies the inverse
    /// (`q_ik = sign · q_urdf`, `q̇_ik = sign · q̇_urdf`) when feeding
    /// MuJoCo joint state back into IK-convention primitives such as
    /// `forward_leg_kinematics` / `foot_jacobian_body`.
    pub fn joint_signs(&self) -> [[f64; 3]; 4] {
        self.joint_signs
    }

    pub fn set_knee_forward(&mut self, leg: LegId, forward: bool) {
        self.inner.set_knee_forward(leg, forward);
    }

    /// Apply a symmetric front/rear [`KneePattern`] (`<<` / `<>` / `><` / `>>`).
    pub fn set_knee_pattern(&mut self, pattern: quadruped_gait::KneePattern) {
        self.inner.set_knee_pattern(pattern);
    }

    /// Read back the current configuration as a [`KneePattern`]. Returns
    /// the best-matching symmetric pattern when per-leg flags are
    /// asymmetric — see [`quadruped_gait::KneePattern::from_knee_forward`].
    pub fn knee_pattern(&self) -> quadruped_gait::KneePattern {
        self.inner.knee_pattern()
    }

    /// Per-leg knee-forward flags `[FL, FR, RL, RR]`.
    pub fn knee_forward(&self) -> [bool; 4] {
        self.inner.knee_forward()
    }

    /// Advance the gait by `dt`. Returns:
    /// - the per-leg controller output (kinematics + phase + foot pose),
    /// - a flat list of 12 `(joint_idx, q)` pairs ready to feed into
    ///   [`crate::mujoco_sim::MujocoSim::set_position_target`],
    /// - a flat list of 12 `(joint_idx, τ_ff)` pairs ready to feed into
    ///   [`crate::mujoco_sim::MujocoSim::set_torque_feedforward`]. In MPC
    ///   mode τ_ff carries the SRBD-MPC ground reaction force mapped
    ///   through the leg Jacobian (`-J^T·f_GRF`) for stance feet; for
    ///   swing feet and for CHAMP mode τ_ff is zero. Sign-corrected per
    ///   joint to match URDF axes.
    pub fn tick(
        &mut self,
        dt: f64,
    ) -> (ControllerOutput, [(usize, f64); 12], [(usize, f64); 12]) {
        let out = self.inner.tick(dt);
        // Stance-leg torque feedforward from the MPC, in IK convention.
        // [None; 4] when running CHAMP or before the first MPC solve.
        let stance_ff = self.inner.stance_grf_torques(&out);
        let mut targets = [(0usize, 0.0); 12];
        let mut torque_ff = [(0usize, 0.0); 12];
        let mut k = 0;
        for slot in 0..4 {
            let qs = [
                out.legs[slot].q_hip,
                out.legs[slot].q_thigh,
                out.legs[slot].q_calf,
            ];
            // Stance torques (IK convention) → URDF via `joint_signs` —
            // the same sign mapping that converts q from IK to URDF
            // applies to τ as well, since q_urdf = sign·q_ik implies
            // τ_urdf[k] = sign[k]·τ_ik[k] (see derivation in
            // `quadruped_gait::foot_jacobian_body`'s callers).
            let taus_ik = stance_ff[slot].unwrap_or([0.0, 0.0, 0.0]);
            for j in 0..3 {
                let ji = self.joint_indices[slot][j];
                let sign = self.joint_signs[slot][j];
                targets[k] = (ji, qs[j] * sign);
                torque_ff[k] = (ji, taus_ik[j] * sign);
                k += 1;
            }
        }
        (out, targets, torque_ff)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Use the bundled namiashi quadruped fixture if present; otherwise
    /// skip the test. Auto-detection is stateful enough that we want a
    /// real URDF to exercise it.
    fn try_load_namiashi() -> Option<RobotModel> {
        let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("tests/fixtures/namiashi/urdf/namiashi.urdf");
        if !path.exists() {
            return None;
        }
        RobotModel::from_file(&path).ok()
    }

    /// Regression for the bug where the IK convention's "positive q_thigh
    /// = thigh forward" disagreed with URDF's right-hand rule about +Y
    /// (positive q rotates a downward vector toward −X, i.e. backward).
    /// The user reported `>>` (knee_forward = true everywhere) producing
    /// rear-bending knees in MuJoCo — the controller was forwarding
    /// IK-frame angles to the simulator without sign-correction. Confirm
    /// `GaitController::build` now stores `-1.0` signs for `+Y` thigh/calf
    /// joints so the URDF's rotation direction matches the IK's intent.
    #[test]
    fn build_picks_correct_sign_for_positive_y_thigh_calf() {
        let Some(model) = try_load_namiashi() else {
            eprintln!("namiashi fixture missing — skipping sign test");
            return;
        };
        let foot_links: [(LegId, &str); 4] = [
            (LegId::FL, "FL_foot"),
            (LegId::FR, "FR_foot"),
            (LegId::RL, "RL_foot"),
            (LegId::RR, "RR_foot"),
        ];
        let kin = auto_detect_kinematics_config(&model, &foot_links).unwrap();
        let cfg = quadruped_gait::GaitConfig::trot();
        let ctrl = GaitController::build(&model, kin, cfg, GaitMode::Champ).unwrap();

        // FL row: hip about +X (sign +1), thigh about +Y (sign −1), calf about +Y (sign −1).
        for slot in 0..4 {
            let s = ctrl.joint_signs[slot];
            assert_eq!(
                s[0].abs(),
                1.0,
                "leg {slot} hip sign magnitude should be 1, got {}",
                s[0],
            );
            assert!(
                (s[1] - -1.0).abs() < 1e-9,
                "leg {slot} thigh sign should be -1 for +Y URDF axis, got {}",
                s[1],
            );
            assert!(
                (s[2] - -1.0).abs() < 1e-9,
                "leg {slot} calf sign should be -1 for +Y URDF axis, got {}",
                s[2],
            );
        }
    }

    #[test]
    fn auto_detect_namiashi_legs_or_skip() {
        let Some(model) = try_load_namiashi() else {
            eprintln!("namiashi fixture missing — skipping auto-detect test");
            return;
        };
        // Use the standard CHAMP foot link names, falling back to "calf"
        // links if the URDF doesn't carry separate foot links.
        let candidates: [&[&str]; 4] = [
            &["FL_foot", "FL_calf"],
            &["FR_foot", "FR_calf"],
            &["RL_foot", "RL_calf"],
            &["RR_foot", "RR_calf"],
        ];
        for (slot, leg) in [LegId::FL, LegId::FR, LegId::RL, LegId::RR].iter().enumerate() {
            let mut found = false;
            for name in candidates[slot] {
                if model.link_map.contains_key(*name) {
                    let result = auto_detect_leg_kinematics(&model, name, *leg);
                    if let Ok(kin) = result {
                        assert!(kin.upper_leg_m > 0.05);
                        assert!(kin.upper_leg_m < 0.5);
                        assert!(kin.lower_leg_m > 0.05);
                        assert!(kin.lower_leg_m < 0.5);
                        found = true;
                        eprintln!("{leg:?} from {name}: L1={:.3}, L2={:.3}, hip_offset={:?}",
                            kin.upper_leg_m, kin.lower_leg_m, kin.hip_offset);
                        break;
                    } else {
                        eprintln!("{leg:?} {name}: detect failed: {:?}", result.err());
                    }
                }
            }
            if !found {
                eprintln!("no working foot link for {leg:?}; tried {:?}", candidates[slot]);
            }
        }
    }

    /// `auto_detect_srbd_mpc_config` must produce a config whose mass
    /// matches the sum of namiashi's link masses (~2.4 kg per the
    /// URDF), and whose inertia diagonal comes from the trunk link
    /// (the URDF root) — *not* the SRBD MPC's Cheetah-3-tuned default.
    /// Catches regressions that would silently fall back to defaults
    /// (e.g., a refactor that breaks the link_map lookup or the
    /// inertial-non-zero gate).
    #[test]
    fn auto_detect_srbd_mpc_config_uses_model_mass_and_inertia() {
        let Some(model) = try_load_namiashi() else {
            eprintln!("namiashi fixture missing — skipping srbd auto-detect test");
            return;
        };
        let cfg = auto_detect_srbd_mpc_config(&model);
        let default = quadruped_gait::SrbdMpcConfig::default();

        // Mass: ≈ 2.4 kg — well below the 9 kg default. Allow ±10%
        // for URDF rounding, but reject "still at default".
        assert!(
            (cfg.mass_kg - 2.4).abs() < 0.3,
            "mass should be ~2.4 kg from namiashi URDF, got {:.3}",
            cfg.mass_kg,
        );
        assert!(
            (cfg.mass_kg - default.mass_kg).abs() > 1e-3,
            "mass should NOT match the Cheetah-3 default {:.3}",
            default.mass_kg,
        );

        // Inertia: namiashi trunk inertia is ixx=0.00189, iyy=0.00857,
        // izz=0.009 — orders of magnitude smaller than the Cheetah-3
        // default. We don't pin exact numbers (URDF could change) but
        // require the result to differ from default and to be positive.
        let diag = cfg.inertia_diag_body;
        assert!(diag.x > 0.0 && diag.y > 0.0 && diag.z > 0.0);
        let differs_from_default = (diag - default.inertia_diag_body).norm() > 1e-3;
        assert!(
            differs_from_default,
            "inertia diag {diag:?} matches default {:?} — model lookup likely broken",
            default.inertia_diag_body,
        );
    }

    /// `GaitController::build` must propagate the auto-detected config
    /// into the inner MPC. Spot-check the round-trip via
    /// `srbd_mpc_config()` accessor — guards against a refactor
    /// dropping the `set_srbd_mpc_config` call inside `build`.
    #[test]
    fn build_propagates_auto_detected_srbd_config() {
        let Some(model) = try_load_namiashi() else {
            eprintln!("namiashi fixture missing — skipping build-propagate test");
            return;
        };
        let foot_links: [(LegId, &str); 4] = [
            (LegId::FL, "FL_foot"),
            (LegId::FR, "FR_foot"),
            (LegId::RL, "RL_foot"),
            (LegId::RR, "RR_foot"),
        ];
        let kin = auto_detect_kinematics_config(&model, &foot_links).unwrap();
        let cfg = quadruped_gait::GaitConfig::trot();
        let gc = GaitController::build(&model, kin, cfg, GaitMode::Mpc).unwrap();
        let active = gc.srbd_mpc_config().expect("MPC mode should expose the config");
        let expected = auto_detect_srbd_mpc_config(&model);
        assert!(
            (active.mass_kg - expected.mass_kg).abs() < 1e-9,
            "build did not apply auto-detected mass: got {:.3}, expected {:.3}",
            active.mass_kg,
            expected.mass_kg,
        );
    }
}
