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

    let calf_axis = model.joints[calf_idx].axis.cast::<f64>();
    let thigh_axis = model.joints[thigh_idx].axis.cast::<f64>();
    let hip_axis = model.joints[hip_idx].axis.cast::<f64>();

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

/// Wrapper around [`InnerController`] (= `quadruped_gait::GaitController`)
/// that caches the joint-name → RobotModel-joint-idx mapping. The cache
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
        Ok(Self {
            inner: InnerController::new(mode, cfg, kin),
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

    /// Feed the latest observed body linear velocity (world frame)
    /// from the host's state estimator. Used by closed-loop
    /// generators (currently MPC) for capture-point feedback. CHAMP
    /// ignores it.
    pub fn set_body_state_observed(
        &mut self,
        v_world: nalgebra::Vector3<f64>,
    ) {
        self.inner.set_body_state_observed(v_world);
    }

    /// Latest SRBD-MPC predicted ground reaction forces (world
    /// frame, per foot, in canonical FL/FR/RL/RR slot order). Returns
    /// `None` when the active mode is CHAMP or the MPC hasn't ticked
    /// yet. Used by the viewport's GRF overlay; not currently fed
    /// back into MuJoCo (Phase 4 work).
    pub fn predicted_grfs(&self) -> Option<&quadruped_gait::MpcSolution> {
        self.inner.predicted_grfs()
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

    /// Advance the gait by `dt`. Returns the per-leg controller output
    /// plus a flat list of `(joint_idx, q)` pairs ready to feed into
    /// [`crate::mujoco_sim::MujocoSim::set_position_target`].
    pub fn tick(&mut self, dt: f64) -> (ControllerOutput, [(usize, f64); 12]) {
        let out = self.inner.tick(dt);
        let mut targets = [(0usize, 0.0); 12];
        let mut k = 0;
        for slot in 0..4 {
            let qs = [
                out.legs[slot].q_hip,
                out.legs[slot].q_thigh,
                out.legs[slot].q_calf,
            ];
            for j in 0..3 {
                targets[k] =
                    (self.joint_indices[slot][j], qs[j] * self.joint_signs[slot][j]);
                k += 1;
            }
        }
        (out, targets)
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
            .join("sample/namiashi_description/urdf/namiashi.urdf");
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
        let ctrl = GaitController::build(&model, kin, cfg).unwrap();

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
}
