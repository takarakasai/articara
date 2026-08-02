//! One builder per priority level of the biped's hierarchical QP.
//!
//! The level ORDER is the driver's business, not this module's -- it changes
//! between double and single support and between experiments, and hiding it
//! behind a fixed list is how a "level 4" tally came to mislabel every
//! single-support run by one. What lives here is how each level's matrices
//! and reference are built, so that two drivers (squat and walk) cannot drift
//! apart in the details.

use misa_wbc::{tasks as wt, AsAffine, Affine, Dynamics, Task};
use nalgebra as na;

use super::contact::{sole_selection, Anchors};

/// P1: track a CoM acceleration reference.
pub fn com(
    qddot: &Affine,
    j_com: &na::DMatrix<f64>,
    djv_com: &na::Vector3<f64>,
    accel_ref: &na::Vector3<f64>,
) -> Task {
    wt::cartesian_acceleration(
        qddot,
        j_com,
        &na::DVector::from_vec(vec![djv_com.x, djv_com.y, djv_com.z]),
        &na::DVector::from_vec(vec![accel_ref.x, accel_ref.y, accel_ref.z]),
    )
}

/// P2: hold the trunk upright, via the WORLD-frame angular Jacobian.
///
/// Not `qddot[0..2]`: those are body-frame and carry a sign inversion. Rows
/// 0..2 of the trunk's own world Jacobian map qddot to world angular
/// acceleration, which is what matches a world-frame roll/pitch error.
pub fn trunk(
    qddot: &Affine,
    j_trunk: &na::DMatrix<f64>,
    djv_trunk: &na::DVector<f64>,
    rp_ref: &[f64; 2],
    nv: usize,
) -> Task {
    trunk_rpy(qddot, j_trunk, djv_trunk, rp_ref, None, nv)
}

/// [`trunk`] with an optional YAW row.
///
/// Roll and pitch alone leave the whole stack with nothing regulating
/// rotation about the vertical. Measured on kyo46rs stepping in place: the
/// swing leg twists (its own orientation being unconstrained), that injects
/// vertical-axis angular momentum, and the base counter-rotates +-8 deg to
/// conserve it while hip_yaw walks to its +-30 deg stop. Regulating the trunk's
/// yaw asks for the quantity that matters and lets the QP decide which joints
/// pay for it, rather than dictating a swing-foot angle.
pub fn trunk_rpy(
    qddot: &Affine,
    j_trunk: &na::DMatrix<f64>,
    djv_trunk: &na::DVector<f64>,
    rp_ref: &[f64; 2],
    yaw_ref: Option<f64>,
    nv: usize,
) -> Task {
    let rows = if yaw_ref.is_some() { 3 } else { 2 };
    let mut j_rp = na::DMatrix::zeros(rows, nv);
    for c in 0..nv {
        j_rp[(0, c)] = j_trunk[(0, c)];
        j_rp[(1, c)] = j_trunk[(1, c)];
        if yaw_ref.is_some() {
            j_rp[(2, c)] = j_trunk[(2, c)];
        }
    }
    let (mut dj, mut r) = (vec![djv_trunk[0], djv_trunk[1]], vec![rp_ref[0], rp_ref[1]]);
    if let Some(y) = yaw_ref {
        dj.push(djv_trunk[2]);
        r.push(y);
    }
    wt::cartesian_acceleration(
        qddot,
        &j_rp,
        &na::DVector::from_vec(dj),
        &na::DVector::from_vec(r),
    )
}

/// Roll/pitch reference acceleration for [`trunk`].
///
/// `dead` is a deadband on the POSITION error only; damping stays live, so
/// the task still resists rate inside the band instead of going open.
///
/// `sign` is +1, the textbook convention. A period when -1 was needed was
/// compensating for a `crba` bug in misarta, not a convention mismatch.
pub fn trunk_rp_ref(
    roll: f64,
    pitch: f64,
    omega_world: &[f64; 3],
    kp: f64,
    kd: f64,
    deadband: f64,
    sign: f64,
) -> [f64; 2] {
    let dead = |e: f64| {
        if e.abs() <= deadband {
            0.0
        } else {
            e - deadband * e.signum()
        }
    };
    [
        sign * (kp * (0.0 - dead(roll)) + kd * (0.0 - omega_world[0])),
        sign * (kp * (0.0 - dead(pitch)) + kd * (0.0 - omega_world[1])),
    ]
}

/// World-frame reference angular acceleration for [`trunk_rpy`], built from a
/// rotation-vector error rather than from Euler components.
///
/// [`trunk_rp_ref`] feeds a ZYX roll/pitch error straight into the WORLD
/// angular-acceleration rows, which is only the same thing at zero heading.
/// A quarter turn later the body's roll axis points along world +y, so the
/// roll correction comes out on the wrong axis; past 90 deg it is also
/// wrong-signed. Measured on kyo46rs: every turn command fell after a fixed
/// ACCUMULATED heading -- 120 deg at wz=0.10, 104 deg at 0.20, 90 deg at 0.40
/// -- while tracking its commanded rate to ~90% right up to the fall.
///
/// `e = log(R_des * R^T)` is the world-frame rotation carrying the body to
/// upright-at-`yaw_des`, so it is correct at any heading. When `yaw_ref` is
/// `None` the desired heading is the current one and the z component is zero,
/// which is what the caller drops.
pub fn trunk_ori_ref(
    r_body: &na::UnitQuaternion<f64>,
    yaw_ref: Option<f64>,
    omega_world: &[f64; 3],
    gains: TrunkGains,
) -> [f64; 3] {
    let (_, _, yaw_now) = r_body.euler_angles();
    let yaw_des = yaw_ref.unwrap_or(yaw_now);
    let r_des = na::UnitQuaternion::from_euler_angles(0.0, 0.0, yaw_des);
    let e = (r_des * r_body.inverse()).scaled_axis();
    let dead = |v: f64| {
        if v.abs() <= gains.deadband { 0.0 } else { v - gains.deadband * v.signum() }
    };
    [
        gains.sign * (gains.kp * dead(e[0]) - gains.kd * omega_world[0]),
        gains.sign * (gains.kp * dead(e[1]) - gains.kd * omega_world[1]),
        gains.kp_yaw * e[2] + gains.kd_yaw * (gains.wz_ref - omega_world[2]),
    ]
}

/// Gains for [`trunk_ori_ref`]. Roll/pitch and yaw keep separate gains because
/// they are tuned against different things -- tilt rejection versus tracking a
/// commanded turn.
#[derive(Clone, Copy, Debug)]
pub struct TrunkGains {
    pub kp: f64,
    pub kd: f64,
    pub deadband: f64,
    pub sign: f64,
    pub kp_yaw: f64,
    pub kd_yaw: f64,
    /// Commanded yaw RATE, fed forward into the damping term.
    pub wz_ref: f64,
}

/// P4: weak posture, so the null space does not wander.
///
/// `actuated` is `(articara joint index, misarta v index)` as
/// [`super::rig::BipedRig::actuated`] returns it.
pub fn posture(
    qddot: &Affine,
    actuated: &[(usize, usize)],
    q_now: &[f64],
    q_seed: &[f64],
    v: &[f64],
    kp: f64,
    kd: f64,
    na_count: usize,
    nv: usize,
) -> Task {
    let mut j_post = na::DMatrix::zeros(na_count, nv);
    let mut post_ref = na::DVector::zeros(na_count);
    for &(ji, vi) in actuated {
        j_post[(vi - 6, vi)] = 1.0;
        post_ref[vi - 6] = kp * (q_seed[ji] - q_now[ji]) + kd * (0.0 - v[vi]);
    }
    wt::cartesian_acceleration(qddot, &j_post, &na::DVector::zeros(na_count), &post_ref)
}

/// Which rows of the swing foot's linear Jacobian to constrain.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum SwingAxes {
    /// Clearance only. What single-leg STANCE wants: constraining x and y as
    /// well pins the swing foot to the world position it happened to occupy
    /// at t=0, on a robot that is deliberately translating its body sideways,
    /// and the reaction for holding it there lands on the stance leg.
    ZOnly,
    /// Full 3-D tracking. What STEPPING wants: the target is a planned
    /// footstep that moves, so there is nothing to fight.
    Xyz,
    /// Position plus YAW only, four rows.
    ///
    /// The middle ground, and usually the right one. Taking all six of the
    /// foot's DoF leaves the swing leg no null space at all -- with the
    /// stance contact already spending 6 rows and CoM + trunk another 5, a
    /// six-row swing task drops the robot in one or two steps at every gain
    /// tried. Yaw is the DoF that actually ran away.
    XyzYaw,
    /// Position AND orientation, six rows.
    ///
    /// Leaving the swing foot's three ROTATIONAL DoF unconstrained is the same
    /// mistake as leaving its translation unconstrained, and it is less
    /// obvious because nothing flies through the air -- the leg simply twists.
    /// Measured on kyo46rs stepping in place with translation-only swing:
    /// hip_yaw walked from 0.04 deg to its +-30 deg STOP over 16 steps, the
    /// feet landed yawed by up to 31 deg, and the base yawed +-8 deg trying to
    /// conserve the angular momentum the twisting legs kept injecting. The QP
    /// was using the free leg's yaw as a null-space dumping ground.
    Pose,
}

/// P3: swing-foot tracking. `kp_xy` is separate from `kp_z` because the two
/// axes are doing different jobs -- z is clearance and must not scuff, x/y is
/// placement and can afford to be soft while the stance leg is the one paying
/// for it.
#[allow(clippy::too_many_arguments)]
pub fn swing(
    qddot: &Affine,
    j_foot: &na::DMatrix<f64>,
    djv: &na::DVector<f64>,
    pos: &na::Vector3<f64>,
    vel: &na::Vector3<f64>,
    target: &na::Vector3<f64>,
    target_vel: &na::Vector3<f64>,
    kp_xy: f64,
    kp_z: f64,
    kd: f64,
    axes: SwingAxes,
) -> Task {
    swing_with_pose(qddot, j_foot, djv, pos, vel, target, target_vel, kp_xy, kp_z, kd, axes, None)
}

/// [`swing`] with an optional orientation target for [`SwingAxes::Pose`].
///
/// `orientation` is `(R_current, R_target, omega, kp, kd)`. The target should
/// come from the PLAN -- the orientation the foot is meant to land in -- not
/// from where the foot happens to be, or the error it is meant to remove
/// becomes the thing it tracks.
#[allow(clippy::too_many_arguments)]
pub fn swing_with_pose(
    qddot: &Affine,
    j_foot: &na::DMatrix<f64>,
    djv: &na::DVector<f64>,
    pos: &na::Vector3<f64>,
    vel: &na::Vector3<f64>,
    target: &na::Vector3<f64>,
    target_vel: &na::Vector3<f64>,
    kp_xy: f64,
    kp_z: f64,
    kd: f64,
    axes: SwingAxes,
    orientation: Option<(
        na::Matrix3<f64>,
        na::Matrix3<f64>,
        na::Vector3<f64>,
        f64,
        f64,
    )>,
) -> Task {
    let a = na::Vector3::new(
        kp_xy * (target.x - pos.x) + kd * (target_vel.x - vel.x),
        kp_xy * (target.y - pos.y) + kd * (target_vel.y - vel.y),
        kp_z * (target.z - pos.z) + kd * (target_vel.z - vel.z),
    );
    match axes {
        SwingAxes::ZOnly => wt::cartesian_acceleration(
            qddot,
            &j_foot.rows(5, 1).into_owned(),
            &na::DVector::from_vec(vec![djv[5]]),
            &na::DVector::from_vec(vec![a.z]),
        ),
        SwingAxes::Xyz => wt::cartesian_acceleration(
            qddot,
            &j_foot.rows(3, 3).into_owned(),
            &na::DVector::from_vec(vec![djv[3], djv[4], djv[5]]),
            &na::DVector::from_vec(vec![a.x, a.y, a.z]),
        ),
        SwingAxes::XyzYaw => {
            let (rot, rot_tgt, omega, kp_r, kd_r) = orientation
                .expect("SwingAxes::XyzYaw needs an orientation target");
            let dr = rot_tgt * rot.transpose();
            let e_yaw = (dr[(1, 0)] - dr[(0, 1)]) * 0.5;
            let a_yaw = kp_r * e_yaw - kd_r * omega.z;
            let mut j = na::DMatrix::zeros(4, j_foot.ncols());
            for c in 0..j_foot.ncols() {
                j[(0, c)] = j_foot[(2, c)];
                for r in 0..3 {
                    j[(1 + r, c)] = j_foot[(3 + r, c)];
                }
            }
            wt::cartesian_acceleration(
                qddot,
                &j,
                &na::DVector::from_vec(vec![djv[2], djv[3], djv[4], djv[5]]),
                &na::DVector::from_vec(vec![a_yaw, a.x, a.y, a.z]),
            )
        }
        SwingAxes::Pose => {
            let (rot, rot_tgt, omega, kp_r, kd_r) = orientation
                .expect("SwingAxes::Pose needs an orientation target");
            // Same small-angle extraction the contact anchor uses: the skew
            // part of R_target * R_current^T.
            let dr = rot_tgt * rot.transpose();
            let e = na::Vector3::new(
                dr[(2, 1)] - dr[(1, 2)],
                dr[(0, 2)] - dr[(2, 0)],
                dr[(1, 0)] - dr[(0, 1)],
            ) * 0.5;
            let a_ang = e * kp_r - omega * kd_r;
            wt::cartesian_acceleration(
                qddot,
                &j_foot.rows(0, 6).into_owned(),
                &na::DVector::from_vec(vec![djv[0], djv[1], djv[2], djv[3], djv[4], djv[5]]),
                &na::DVector::from_vec(vec![a_ang.x, a_ang.y, a_ang.z, a.x, a.y, a.z]),
            )
        }
    }
}

/// Lowest level: gravity-comp torque + the nominal load split.
///
/// The load split is NOT an equal share. In the CoM task's null space,
/// "transfer load between the feet" and "walk the CoP outward" are
/// interchangeable, and a 50/50 force target makes the regulariser pick the
/// second one every time. Measured with the equal-share target: at CoM
/// y = +15 mm the split was still 50.1/49.9 and the stance CoP was already
/// 74% of the way to its edge, when 60.8/39.2 would have held it centred.
pub fn regulariser(
    tau: &Affine,
    tau_gravity: &na::DVector<f64>,
    forces: &impl AsAffine,
    forces_nominal: &na::DVector<f64>,
) -> Task {
    wt::regularize(tau, tau_gravity) + wt::regularize(forces, forces_nominal)
}

/// Build `forces_nominal`: each stance foot's share of body weight, in the
/// vertical row of its wrench block.
pub fn force_nominal(
    forces_size: usize,
    shares: &[f64],
    weight_n: f64,
) -> na::DVector<f64> {
    let mut forces_nominal = na::DVector::zeros(forces_size);
    let share_tot: f64 = shares.iter().sum::<f64>().max(1e-6);
    for (slot, s) in shares.iter().enumerate() {
        forces_nominal[6 * slot + 5] = weight_n * s / share_tot;
    }
    forces_nominal
}

/// Everything the P0 contact level needs that is not per-foot state.
pub struct ContactCfg {
    /// Baumgarte gains on the foot pose error against its anchor.
    pub kp_c: f64,
    pub kd_c: f64,
    pub anchor_leak: f64,
    pub anchor_leak_rot: f64,
    /// Sole plane in the foot link frame: `[centre_x, 0, -below_origin]`.
    pub sole_offset_local: [f64; 3],
    pub friction_mu: f64,
    /// CoP box half-extents, already scaled by any margin factor.
    pub cop_half: (f64, f64),
    pub mu_torsion: f64,
    pub f_max_per_foot: f64,
    /// Drop `patch_contact` entirely, to test whether the CoP box is what is
    /// binding.
    pub no_patch: bool,
    pub dt: f64,
}

pub struct ContactLevel {
    pub task: Task,
    /// Per stance slot: the map from the force block to that foot's
    /// sole-frame wrench, kept so the solved CoP can be measured against the
    /// box that constrained it.
    pub sole_sel: Vec<na::DMatrix<f64>>,
    /// `[vertical Baumgarte accel_ref, vertical pose error]` for the LEFT
    /// foot: what the contact constraint is asking for, against what the foot
    /// is actually doing.
    pub acc_dbg: [f64; 2],
}

/// P0: contact acceleration (Baumgarte) + friction cone + CoP box + `f_max`,
/// added on top of `base` (the torque box, and the EoM too unless it has been
/// promoted to a level of its own).
#[allow(clippy::too_many_arguments)]
pub fn contact_level(
    dyn_ctx: &Dynamics,
    base: Task,
    j_contact: &na::DMatrix<f64>,
    dj_v: &na::DVector<f64>,
    data: &misarta::data::Data<f64>,
    v_dvec: &na::DVector<f64>,
    stance: &[usize],
    left_foot_mi: usize,
    anchors: &mut Anchors,
    load_share: &dyn Fn(usize) -> f64,
    cfg: &ContactCfg,
) -> ContactLevel {
    let forces = dyn_ctx.forces();
    let mut p0 = base;
    let mut sole_sel: Vec<na::DMatrix<f64>> = Vec::with_capacity(stance.len());
    let mut acc_dbg = [0.0_f64; 2];

    for (slot, foot_mi) in stance.iter().copied().enumerate() {
        let js = j_contact.rows(6 * slot, 6).into_owned();
        let djvs = dj_v.rows(6 * slot, 6).into_owned();
        let pos = misarta::se3::translation(&data.oMi[foot_mi]);
        let rot = misarta::se3::rotation_matrix(&data.oMi[foot_mi]);
        let side = usize::from(foot_mi != left_foot_mi);
        anchors.ensure(side, pos, rot);
        anchors.leak(side, pos, rot, cfg.anchor_leak, cfg.anchor_leak_rot, cfg.dt);

        let vel = &js * v_dvec;
        let acc_ref = anchors.accel_ref(side, pos, rot, &vel, cfg.kp_c, cfg.kd_c);
        if foot_mi == left_foot_mi {
            acc_dbg = [acc_ref[5], anchors.lin_error(side, pos).z];
        }

        let sel = sole_selection(&rot, cfg.sole_offset_local, forces.size(), slot);
        let w_sole = &sel * &forces.as_affine();
        sole_sel.push(sel);

        p0 = p0 + wt::cartesian_acceleration(dyn_ctx.qddot(), &js, &djvs, &acc_ref);
        if !cfg.no_patch {
            // f_max carries the load ramp. The CoP box is |m| <= L*fz, so
            // squeezing fz shrinks the box with it -- a foot that is being
            // unloaded stops being able to argue for a CoP it is about to
            // lose.
            let share = load_share(foot_mi);
            let sole_patch = wt::ContactPatch {
                mu: cfg.friction_mu,
                cop_half: cfg.cop_half,
                mu_torsion: cfg.mu_torsion,
                f_max: (cfg.f_max_per_foot * share).max(0.5),
            };
            p0 = p0 + wt::patch_contact(&w_sole, &sole_patch);
        }
    }

    ContactLevel { task: p0, sole_sel, acc_dbg }
}
