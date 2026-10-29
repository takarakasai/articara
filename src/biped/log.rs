//! The trajectory CSV, and the two-source contact-force measurement that
//! feeds it.
//!
//! **Both force columns exist on purpose.** `fqp_*` is what the QP's solved
//! contact wrench implies; `fmj_*` is what MuJoCo's actual contact set
//! produces. They agree to within a millimetre in double support and diverge
//! exactly where the solve degrades, and comparing them is how a systematic
//! error was found that every single-source diagnostic had called healthy:
//! in a settled single-leg pose the QP believed it was applying 24 N of
//! tangential force -- 37% of body weight -- while the plant measured 0.0 N.
//! Dropping either column removes the only cross-check in the file.

use nalgebra as na;
use std::io::Write;

use super::rig::BipedRig;

/// Per-foot ground truth read back out of the plant.
#[derive(Default, Clone, Copy)]
pub struct MeasuredContact {
    /// `[x, y, fz]` -- CoP in the sole frame (relative to the sole centre)
    /// and the vertical load. `fz == 0` means unsupported.
    pub cop: [[f64; 3]; 2],
    /// `|f_tangential| / (mu * fz)`, per foot: how much of the friction cone
    /// the foot is actually using.
    pub slip: [f64; 2],
    /// WORLD position of each contact patch. The link origin sits 35 mm above
    /// the sole, so it swings sideways when the ankle rolls -- watching the
    /// origin cannot tell a foot that slid from a foot that merely tipped.
    /// This can.
    pub patch_w: [[f64; 2]; 2],
    /// World-frame contact force per foot, directly comparable with the QP's.
    pub f_w: [[f64; 3]; 2],
    /// Full ground-contact wrench [mx,my,mz,fx,fy,fz] at the sole centre,
    /// expressed in the actual MuJoCo foot frame. Includes contact couples.
    /// Unlike legacy CoP fields, no stale kinematic `data` pose is used here.
    pub sole_wrench: [[f64; 6]; 2],
    pub sole_wrench_valid: [bool; 2],
}

/// Shift a world contact wrench to a sole origin, then rotate both parts.
pub fn contact_wrench_in_sole(
    contact_position: &na::Vector3<f64>,
    force: &na::Vector3<f64>,
    torque: &na::Vector3<f64>,
    sole_origin: &na::Vector3<f64>,
    sole_rotation: &na::Matrix3<f64>,
) -> na::SVector<f64, 6> {
    let moment =
        sole_rotation.transpose() * (torque + (contact_position - sole_origin).cross(force));
    let force = sole_rotation.transpose() * force;
    na::SVector::<f64, 6>::new(moment.x, moment.y, moment.z, force.x, force.y, force.z)
}

/// Force-weighted mean of the ground contact points on each foot, plus the
/// tangential force, so the stance foot can be tested against its own
/// friction cone rather than assumed to stick.
pub fn measure_contacts(
    rig: &BipedRig,
    data: &misarta::data::Data<f64>,
    friction_mu: f64,
) -> MeasuredContact {
    let mut out = MeasuredContact::default();
    let sole_frames: [Option<(na::Vector3<f64>, na::Matrix3<f64>)>; 2] =
        std::array::from_fn(|side| {
            let link = rig.prof.foot_links[side];
            let position = rig.sim.body_world_position(link)?;
            let rotation = rig
                .sim
                .body_world_orientation(link)?
                .to_rotation_matrix()
                .into_inner();
            let origin = na::Vector3::from_column_slice(&position)
                + rotation
                    * na::Vector3::new(rig.prof.sole_centre_x, 0.0, -rig.prof.sole_below_origin);
            origin
                .iter()
                .chain(rotation.iter())
                .all(|v| v.is_finite())
                .then_some((origin, rotation))
        });
    out.sole_wrench_valid = sole_frames.each_ref().map(Option::is_some);
    let mut acc = [[0.0_f64; 4]; 2]; // [sum fz*x, fz*y, fz*z, sum fz]
    let mut ft = [[0.0_f64; 2]; 2];
    for c in rig.sim.contacts() {
        // A stance contact is foot <-> walkable surface. The surface used to
        // be only the ground PLANE (empty body name); scenery boxes welded
        // to the world -- staircase treads, a spawn platform -- are named
        // bodies, and the old "one side is empty" test silently dropped
        // every contact with them: a foot standing on a tread measured zero
        // force, no CoP, no slip. Identify the foot by name on EITHER side
        // and require the other side to be walkable; a foot touching a
        // robot link (a kicked shin) stays excluded as before.
        let foot_of = |n: &str| -> Option<usize> {
            if n == rig.prof.foot_links[0] {
                Some(0)
            } else if n == rig.prof.foot_links[1] {
                Some(1)
            } else {
                None
            }
        };
        let walkable = |n: &str| n.is_empty() || n.starts_with("stair");
        let (side, foot_is_first) = match (foot_of(&c.body1), foot_of(&c.body2)) {
            (Some(s), None) if walkable(&c.body2) => (s, true),
            (None, Some(s)) if walkable(&c.body1) => (s, false),
            _ => continue,
        };
        // The plane path always had the foot as body2; keep that force
        // convention, flipping when the foot is on the other end.
        let sgn = if foot_is_first { -1.0 } else { 1.0 };
        let fw = [
            sgn * c.force_world[0],
            sgn * c.force_world[1],
            sgn * c.force_world[2],
        ];
        if let Some((origin, rotation)) = &sole_frames[side] {
            let force = na::Vector3::from_column_slice(&fw);
            let torque = na::Vector3::from_column_slice(&c.torque_world) * sgn;
            let wrench = contact_wrench_in_sole(
                &na::Vector3::from_column_slice(&c.pos),
                &force,
                &torque,
                origin,
                rotation,
            );
            for j in 0..6 {
                out.sole_wrench[side][j] += wrench[j];
            }
            out.sole_wrench_valid[side] &= out.sole_wrench[side].iter().all(|v| v.is_finite());
        }
        let fz = fw[2];
        if fz <= 0.0 {
            continue;
        }
        for k in 0..3 {
            acc[side][k] += fz * c.pos[k];
        }
        acc[side][3] += fz;
        ft[side][0] += fw[0];
        ft[side][1] += fw[1];
    }
    for side in 0..2 {
        let fz = acc[side][3];
        if fz <= 1e-6 {
            continue;
        }
        let foot_mi = rig.foot_mi[side];
        let o = misarta::se3::translation(&data.oMi[foot_mi]);
        let r = misarta::se3::rotation_matrix(&data.oMi[foot_mi]);
        let pw = na::Vector3::new(acc[side][0] / fz, acc[side][1] / fz, acc[side][2] / fz);
        let pl = r.transpose() * (pw - o);
        out.cop[side] = [pl.x - rig.prof.sole_centre_x, pl.y, fz];
        let tan = (ft[side][0].powi(2) + ft[side][1].powi(2)).sqrt();
        out.slip[side] = tan / (friction_mu * fz).max(1e-9);
        out.patch_w[side] = [pw.x, pw.y];
        out.f_w[side] = [ft[side][0], ft[side][1], fz];
    }
    out
}

/// The columns that vary per controller, gathered so the writer signature
/// does not run to thirty arguments.
pub struct Row<'a> {
    pub t: f64,
    pub com: na::Vector3<f64>,
    /// Vertical CoM reference, and the lateral one (`com_ref_y` in the file).
    pub com_ref_z: f64,
    pub com_ref_y: f64,
    pub tilt: f64,
    pub trunk_tilt: f64,
    pub n_stance: usize,
    /// z of the left foot (`foot_z`) and of the right (`swing_z`).
    pub foot_z: f64,
    pub swing_z: f64,
    pub foot_vz: f64,
    /// `[Baumgarte vertical accel_ref, vertical pose error]`, left foot.
    pub acc_dbg: [f64; 2],
    pub a_com: na::Vector3<f64>,
    pub rp_ref: [f64; 2],
    pub degraded: bool,
    pub cop_box: (f64, f64),
    pub cop_qp: [[f64; 3]; 2],
    pub f_qp_w: [[f64; 3]; 2],
    pub measured: &'a MeasuredContact,
    pub taus: &'a [f64],
    /// Controller-specific columns, in the order the names were given to
    /// [`TrajLog::create`]. Length must match, or the file silently stops
    /// being parseable by column name.
    pub extra: &'a [f64],
}

/// Writer for the trajectory CSV that `examples/render_com_squat_video.py`
/// reads. Column layout is shared with `kyo46rs_squat.rs` so the replay
/// tooling does not fork.
pub struct TrajLog {
    file: Option<std::fs::File>,
    joints: Vec<&'static str>,
    n_extra: usize,
}

impl TrajLog {
    /// `None` path disables logging entirely. `extra` names controller-
    /// specific columns appended after the shared ones, so the replay tooling
    /// keeps working on a file it does not fully understand.
    pub fn create(path: Option<String>, joints: Vec<&'static str>, extra: &[&str]) -> Self {
        let file = path.map(|path| {
            let mut f = std::fs::File::create(&path).expect("create trajectory log");
            write!(f, "t,x,y,z,qw,qx,qy,qz,com_x,com_y,com_z,com_ref_z,tilt,com_ref_y,n_stance,swing_z").unwrap();
            write!(f, ",trunk_tilt,foot_z,foot_vz,acc_ref_z,e_lin_z,acom_x,acom_y,acom_z,arp_r,arp_p,degraded,cop_lx,cop_ly,slip_l,slip_r,patch_lx,patch_ly").unwrap();
            for side in ["l", "r"] {
                for src in ["qp", "mj"] {
                    for ax in ["x", "y", "z"] {
                        write!(f, ",f{src}_{side}_{ax}").unwrap();
                    }
                }
            }
            for side in ["l", "r"] {
                for src in ["qp", "mj"] {
                    write!(f, ",cop_{src}_{side}_x,cop_{src}_{side}_y,fz_{src}_{side}").unwrap();
                }
            }
            for n in &joints {
                write!(f, ",{n}").unwrap();
            }
            // WBC-commanded torque per joint, plus the joint's effort limit,
            // so a replay can show demand against capability and make
            // saturation visible rather than silently clipped.
            for n in &joints {
                write!(f, ",tau_{n}").unwrap();
            }
            for n in &joints {
                write!(f, ",lim_{n}").unwrap();
            }
            for n in extra {
                write!(f, ",{n}").unwrap();
            }
            writeln!(f).unwrap();
            println!("logging trajectory to {path}");
            f
        });
        TrajLog {
            file,
            joints,
            n_extra: extra.len(),
        }
    }

    pub fn is_enabled(&self) -> bool {
        self.file.is_some()
    }

    pub fn write(&mut self, rig: &BipedRig, r: &Row) {
        let Some(f) = self.file.as_mut() else { return };
        let p = rig.sim.body_world_position(&rig.robot.root_link).unwrap();
        let qq = rig
            .sim
            .body_world_orientation(&rig.robot.root_link)
            .unwrap();
        let t = r.t;
        let z_ref = r.com_ref_z;
        let y_ref = r.com_ref_y;
        let nc = r.n_stance;
        write!(
            f,
            "{t:.4},{:.5},{:.5},{:.5},{:.5},{:.5},{:.5},{:.5},{:.5},{:.5},{:.5},{z_ref:.5},{:.5}",
            p[0], p[1], p[2], qq.w, qq.i, qq.j, qq.k, r.com.x, r.com.y, r.com.z, r.tilt
        )
        .unwrap();
        write!(
            f,
            ",{y_ref:.5},{nc},{:.5},{:.5},{:.6},{:.5},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4}",
            r.swing_z,
            r.trunk_tilt,
            r.foot_z,
            r.foot_vz,
            r.acc_dbg[0],
            r.acc_dbg[1],
            r.a_com.x,
            r.a_com.y,
            r.a_com.z,
            r.rp_ref[0],
            r.rp_ref[1]
        )
        .unwrap();
        write!(
            f,
            ",{},{:.5},{:.5}",
            u8::from(r.degraded),
            r.cop_box.0,
            r.cop_box.1
        )
        .unwrap();
        write!(f, ",{:.4},{:.4}", r.measured.slip[0], r.measured.slip[1]).unwrap();
        write!(
            f,
            ",{:.6},{:.6}",
            r.measured.patch_w[0][0], r.measured.patch_w[0][1]
        )
        .unwrap();
        for side in 0..2 {
            for src in [&r.f_qp_w, &r.measured.f_w] {
                let v = src[side];
                write!(f, ",{:.4},{:.4},{:.4}", v[0], v[1], v[2]).unwrap();
            }
        }
        for side in 0..2 {
            for src in [&r.cop_qp, &r.measured.cop] {
                let c = src[side];
                write!(f, ",{:.6},{:.6},{:.4}", c[0], c[1], c[2]).unwrap();
            }
        }
        for n in &self.joints {
            let a = rig
                .robot
                .joint_map
                .get(*n)
                .map(|&ji| rig.robot.joint_positions[ji])
                .unwrap_or(0.0);
            write!(f, ",{a:.5}").unwrap();
        }
        for n in &self.joints {
            let tq = rig
                .robot
                .joint_map
                .get(*n)
                .map(|&ji| r.taus[ji])
                .unwrap_or(0.0);
            write!(f, ",{tq:.5}").unwrap();
        }
        for n in &self.joints {
            let lm = rig
                .robot
                .joint_map
                .get(*n)
                .map(|&ji| rig.robot.joints[ji].effort)
                .unwrap_or(0.0);
            write!(f, ",{lm:.3}").unwrap();
        }
        assert_eq!(
            r.extra.len(),
            self.n_extra,
            "extra column count changed mid-run; the header no longer describes the rows"
        );
        for x in r.extra {
            write!(f, ",{x:.6}").unwrap();
        }
        writeln!(f).unwrap();
    }
}

#[cfg(test)]
mod wrench_tests {
    use super::*;
    #[test]
    fn contact_wrench_translation_rotation_and_body_reversal() {
        let rotation =
            na::Rotation3::from_euler_angles(0.0, 0.0, std::f64::consts::FRAC_PI_2).into_inner();
        let origin = na::Vector3::new(1.0, 2.0, 3.0);
        let point = origin + na::Vector3::new(1.0, 0.0, 0.0);
        let force = na::Vector3::new(0.0, 2.0, 3.0);
        let torque = na::Vector3::new(4.0, 5.0, 6.0);
        // World moment = [4,2,8]; a 90 degree sole yaw maps it to [2,-4,8].
        let actual = contact_wrench_in_sole(&point, &force, &torque, &origin, &rotation);
        let expected = na::SVector::<f64, 6>::new(2.0, -4.0, 8.0, 2.0, 0.0, 3.0);
        assert!((actual - expected).amax() < 1e-12);
        let reversed = contact_wrench_in_sole(&point, &(-force), &(-torque), &origin, &rotation);
        assert!((reversed + actual).amax() < 1e-12);
    }
    #[test]
    fn separated_contacts_and_free_moments_accumulate_about_same_origin() {
        let origin = na::Vector3::zeros();
        let rotation = na::Matrix3::identity();
        let force = na::Vector3::new(0.0, 0.0, 10.0);
        let torque = na::Vector3::new(0.0, 0.0, 0.2);
        let a = contact_wrench_in_sole(
            &na::Vector3::new(0.1, 0.0, 0.0),
            &force,
            &torque,
            &origin,
            &rotation,
        );
        let b = contact_wrench_in_sole(
            &na::Vector3::new(-0.1, 0.0, 0.0),
            &force,
            &torque,
            &origin,
            &rotation,
        );
        assert!((a + b - na::SVector::<f64, 6>::new(0.0, 0.0, 0.4, 0.0, 0.0, 20.0)).amax() < 1e-12);
    }
}
