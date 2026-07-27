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
    let mut acc = [[0.0_f64; 4]; 2]; // [sum fz*x, fz*y, fz*z, sum fz]
    let mut ft = [[0.0_f64; 2]; 2];
    for c in rig.sim.contacts() {
        let name = if c.body1.is_empty() { &c.body2 } else { &c.body1 };
        let side = match name.as_str() {
            n if n == rig.prof.foot_links[0] => 0,
            n if n == rig.prof.foot_links[1] => 1,
            _ => continue,
        };
        let fz = c.force_world[2];
        if fz <= 0.0 {
            continue;
        }
        for k in 0..3 {
            acc[side][k] += fz * c.pos[k];
        }
        acc[side][3] += fz;
        ft[side][0] += c.force_world[0];
        ft[side][1] += c.force_world[1];
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
        TrajLog { file, joints, n_extra: extra.len() }
    }

    pub fn is_enabled(&self) -> bool {
        self.file.is_some()
    }

    pub fn write(&mut self, rig: &BipedRig, r: &Row) {
        let Some(f) = self.file.as_mut() else { return };
        let p = rig.sim.body_world_position(&rig.robot.root_link).unwrap();
        let qq = rig.sim.body_world_orientation(&rig.robot.root_link).unwrap();
        let t = r.t;
        let z_ref = r.com_ref_z;
        let y_ref = r.com_ref_y;
        let nc = r.n_stance;
        write!(
            f,
            "{t:.4},{:.5},{:.5},{:.5},{:.5},{:.5},{:.5},{:.5},{:.5},{:.5},{:.5},{z_ref:.5},{:.5}",
            p[0], p[1], p[2], qq.w, qq.i, qq.j, qq.k,
            r.com.x, r.com.y, r.com.z,
            r.tilt
        )
        .unwrap();
        write!(
            f,
            ",{y_ref:.5},{nc},{:.5},{:.5},{:.6},{:.5},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4}",
            r.swing_z, r.trunk_tilt, r.foot_z, r.foot_vz,
            r.acc_dbg[0], r.acc_dbg[1], r.a_com.x, r.a_com.y, r.a_com.z, r.rp_ref[0], r.rp_ref[1]
        )
        .unwrap();
        write!(f, ",{},{:.5},{:.5}", u8::from(r.degraded), r.cop_box.0, r.cop_box.1).unwrap();
        write!(f, ",{:.4},{:.4}", r.measured.slip[0], r.measured.slip[1]).unwrap();
        write!(f, ",{:.6},{:.6}", r.measured.patch_w[0][0], r.measured.patch_w[0][1]).unwrap();
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
            let a = rig.robot.joint_map.get(*n).map(|&ji| rig.robot.joint_positions[ji]).unwrap_or(0.0);
            write!(f, ",{a:.5}").unwrap();
        }
        for n in &self.joints {
            let tq = rig.robot.joint_map.get(*n).map(|&ji| r.taus[ji]).unwrap_or(0.0);
            write!(f, ",{tq:.5}").unwrap();
        }
        for n in &self.joints {
            let lm = rig.robot.joint_map.get(*n).map(|&ji| rig.robot.joints[ji].effort).unwrap_or(0.0);
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
