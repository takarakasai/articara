//! Does the contact Jacobian go singular as the unloading leg straightens?
fn main() {
    use articara::robot::RobotModel;
    use articara::wbc_pipeline::build_floating_base_model;
    use nalgebra as na;
    let robot = RobotModel::from_urdf(std::path::Path::new(
        "/home/takara/work/dp/humanoid/kyo46rs_description/urdf/kyo46rs.urdf",
    )).unwrap();
    let (model, a2m, l2i) = build_floating_base_model(&robot);
    let nv = model.nv;
    let rfoot = *l2i.get("right_foot_link").unwrap();
    let qi = |n: &str| model.q_idx[a2m[*robot.joint_map.get(n).unwrap()].unwrap()];

    println!("right leg, hip+knee+ankle held on a straight-leg locus (ankle keeps the sole flat)");
    println!("{:>10} {:>12} {:>12} {:>14}", "knee[deg]", "sigma_min", "cond", "1/sigma_min");
    for kd in [40.0_f64, 30.0, 20.0, 12.0, 8.0, 4.0, 2.0, 1.0, 0.5, 0.1] {
        let k = kd.to_radians();
        let mut q = model.neutral_q();
        q[qi("right_hip_pitch_joint")] = -k / 2.0;
        q[qi("right_knee_joint")] = k;
        q[qi("right_ankle_pitch_joint")] = -k / 2.0;
        let j = misarta::jacobian::compute_joint_jacobian(&model, &q, rfoot);
        // the six leg columns only: what the leg itself can do at the foot
        let cols = ["right_hip_yaw_joint","right_hip_roll_joint","right_hip_pitch_joint",
                    "right_knee_joint","right_ankle_pitch_joint","right_ankle_roll_joint"];
        let mut jl = na::DMatrix::zeros(6, 6);
        for (c, n) in cols.iter().enumerate() {
            let vi = model.v_idx[a2m[*robot.joint_map.get(*n).unwrap()].unwrap()];
            for r in 0..6 { jl[(r, c)] = j[(r, vi)]; }
        }
        let sv = jl.singular_values();
        let (mx, mn) = (sv.max(), sv.min());
        println!("{kd:10.1} {mn:12.5} {:12.1} {:14.1}", mx / mn, 1.0 / mn);
        let _ = nv;
    }
}
