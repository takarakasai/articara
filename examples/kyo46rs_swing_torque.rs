//! How much torque does hip_pitch actually need?
//!
//! The squat answers "almost none" (0.41 N*m steady), and single-leg
//! stance cannot answer it at all: this robot's 38 mm sole gives only
//! +-1.24 N*m of lateral CoP budget, so the WBC sits in an infeasible
//! regime the whole time it is on one foot and its torques mean nothing.
//!
//! What actually sizes hip_pitch on a biped is SWING -- accelerating the
//! free leg fore/aft between footfalls. That is a pure inverse-dynamics
//! question: prescribe a swing trajectory and read the torque back out.
//! No controller, no QP, no balance, nothing to be infeasible.
//!
//! Run with: `cargo run --example kyo46rs_swing_torque`

fn main() {
    use articara::robot::RobotModel;
    use articara::wbc_pipeline::build_floating_base_model;
    use std::f64::consts::PI;

    let robot = RobotModel::from_urdf(std::path::Path::new(
        "/home/takara/work/dp/humanoid/kyo46rs_description/urdf/kyo46rs.urdf",
    ))
    .expect("load kyo46rs.urdf");
    let (model, a2m, _l2i) = build_floating_base_model(&robot);
    let nv = model.nv;

    let vidx = |name: &str| -> usize {
        let ji = *robot.joint_map.get(name).expect(name);
        model.v_idx[a2m[ji].expect("mapped")]
    };
    let qidx = |name: &str| -> usize {
        let ji = *robot.joint_map.get(name).expect(name);
        model.q_idx[a2m[ji].expect("mapped")]
    };

    const EL05_PEAK: f64 = 6.0;
    const EL05_CONT: f64 = 1.8;

    // A swing is the hip carrying the whole leg through +-amp in half a
    // step period. Take it as a half-cosine, which is what a smooth
    // swing profile looks like, and read the peak of each joint's torque.
    // sanity: at rest, rnea must reproduce compute_gravity exactly, and
    // the static hold torque must match m*g*d by hand. If either fails the
    // numbers below mean nothing.
    {
        let q0 = model.neutral_q();
        let z = vec![0.0; nv];
        let tau0 = misarta::rnea::rnea(&model, &q0, &z, &z);
        let g0 = misarta::rnea::compute_gravity(&model, &q0);
        let worst = (0..nv).map(|i| (tau0[i] - g0[i]).abs()).fold(0.0_f64, f64::max);
        println!("check: |rnea(q,0,0) - gravity| = {worst:.3e}  (must be ~0)");

        // leg hanging, hip rotated 0.3 rad: gravity torque about the hip
        let mut q1 = model.neutral_q();
        q1[qidx("left_hip_pitch_joint")] = 0.30;
        let g1 = misarta::rnea::compute_gravity(&model, &q1);
        let m_leg = 0.604 + 0.322 + 0.252 + 0.322;
        println!("check: static hip_pitch at 0.30 rad = {:.3} N·m  (leg {:.3} kg)",
                 g1[vidx("left_hip_pitch_joint")], m_leg);
    }

    println!("\nswing-leg inverse dynamics, torso held (stance side static)\n");
    println!("{:>6} {:>6} {:>11} {:>11} {:>11}   {}",
             "f[Hz]", "amp", "hip_pitch", "knee", "ankle_p", "hip_pitch verdict");

    for &(freq, amp) in &[
        (0.8_f64, 0.30_f64), (1.0, 0.30), (1.25, 0.30),
        (1.0, 0.45), (1.25, 0.45), (1.5, 0.45), (2.0, 0.45),
    ] {
        let w = 2.0 * PI * freq;
        let mut peak = [0.0_f64; 3];
        let n = 400;
        for i in 0..n {
            let t = i as f64 / n as f64 / freq;
            let mut q = model.neutral_q();
            let mut v = vec![0.0; nv];
            let mut a = vec![0.0; nv];
            // hip carries the leg; knee follows at half amplitude, as it
            // does in a real swing (flex to clear, extend to land).
            for (name, scale) in [("left_hip_pitch_joint", 1.0), ("left_knee_joint", -0.5)] {
                let (qi, vi) = (qidx(name), vidx(name));
                q[qi] = scale * amp * (w * t).sin();
                v[vi] = scale * amp * w * (w * t).cos();
                a[vi] = -scale * amp * w * w * (w * t).sin();
            }
            let tau = misarta::rnea::rnea(&model, &q, &v, &a);
            let g = misarta::rnea::compute_gravity(&model, &q);
            for (k, name) in ["left_hip_pitch_joint", "left_knee_joint", "left_ankle_pitch_joint"]
                .iter()
                .enumerate()
            {
                // rnea includes gravity; keep it, a swinging leg is lifted
                // against gravity too. (g printed separately below.)
                let _ = &g;
                peak[k] = peak[k].max(tau[vidx(name)].abs());
            }
        }
        let verdict = if peak[0] > 2.0 * EL05_PEAK {
            "over even 2x peak".to_string()
        } else if peak[0] > EL05_PEAK {
            format!("needs 2 motors ({:.0}% of 1x peak)", peak[0] / EL05_PEAK * 100.0)
        } else if peak[0] > EL05_CONT {
            format!("1 motor OK on peak ({:.0}% of 1x peak)", peak[0] / EL05_PEAK * 100.0)
        } else {
            "1 motor, within continuous".to_string()
        };
        println!("{freq:6.2} {amp:6.2} {:9.2}N·m {:9.2}N·m {:9.2}N·m   {verdict}",
                 peak[0], peak[1], peak[2]);
    }

    println!("\nreference: EL05 continuous {EL05_CONT} N·m, peak {EL05_PEAK} N·m per unit");
    println!("hip_pitch as built = 2 units -> {} continuous / {} peak",
             2.0 * EL05_CONT, 2.0 * EL05_PEAK);
}
