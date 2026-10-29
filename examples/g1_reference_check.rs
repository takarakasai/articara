//! Independent misarta check of Python's sampled whole-body reference.
use articara::{robot::RobotModel, wbc_pipeline::build_floating_base_model};
use nalgebra as na;
fn vector(value: &serde_json::Value) -> Vec<f64> {
    value
        .as_array()
        .unwrap()
        .iter()
        .map(|x| x.as_f64().unwrap())
        .collect()
}
fn main() {
    let args: Vec<_> = std::env::args().collect();
    assert_eq!(args.len(), 3, "g1_reference_check URDF reference.json");
    let robot = RobotModel::from_urdf(std::path::Path::new(&args[1])).unwrap();
    let (model, a2m, links) = build_floating_base_model(&robot);
    let plan: serde_json::Value =
        serde_json::from_slice(&std::fs::read(&args[2]).unwrap()).unwrap();
    let names: Vec<_> = plan["joint_names"]
        .as_array()
        .unwrap()
        .iter()
        .map(|n| n.as_str().unwrap())
        .collect();
    let ids: Vec<_> = names
        .iter()
        .map(|n| a2m[robot.joint_map[*n]].unwrap())
        .collect();
    let r = &plan["base_rotation"];
    let rotation = na::Matrix3::from_fn(|i, j| r[i][j].as_f64().unwrap());
    let quaternion = na::UnitQuaternion::from_matrix(&rotation);
    let mut com_error = 0.0_f64;
    let mut h_error = 0.0_f64;
    let mut torque_error = 0.0_f64;
    let mut base_error = 0.0_f64;
    let mut checked = 0;
    for (k, pose) in plan["q"].as_array().unwrap().iter().enumerate() {
        let mut q = model.neutral_q();
        q[..3].copy_from_slice(&vector(pose)[..3]);
        q[3..7].copy_from_slice(&[quaternion.i, quaternion.j, quaternion.k, quaternion.w]);
        let mut v = vec![0.0; model.nv];
        let mut a = vec![0.0; model.nv];
        let bv = rotation.transpose()
            * na::Vector3::from_column_slice(&vector(&plan["base_velocity"][k]));
        let ba = rotation.transpose()
            * na::Vector3::from_column_slice(&vector(&plan["base_acceleration"][k]));
        v[3..6].copy_from_slice(bv.as_slice());
        a[3..6].copy_from_slice(ba.as_slice());
        for (j, &mi) in ids.iter().enumerate() {
            q[model.q_idx[mi]] = pose[3 + j].as_f64().unwrap();
            v[model.v_idx[mi]] = plan["qd"][k][j].as_f64().unwrap();
            a[model.v_idx[mi]] = plan["qdd"][k][j].as_f64().unwrap();
        }
        let com = misarta::centroidal::compute_com(&model, &q);
        com_error =
            com_error.max((com - na::Vector3::from_column_slice(&vector(&plan["com"][k]))).norm());
        let h = misarta::centroidal::compute_centroidal_momentum_matrix(&model, &q)
            * na::DVector::from_column_slice(&v);
        h_error = h_error
            .max((h.rows(0, 3) - na::Vector3::from_column_slice(&vector(&plan["h"][k]))).norm());
        if plan["wrenches"][k].is_null() {
            continue;
        }
        let mut required = misarta::rnea::rnea(&model, &q, &v, &a);
        let data = misarta::fk::forward_kinematics(&model, &q);
        for (slot, side) in plan["wrenches"][k]["sides"]
            .as_array()
            .unwrap()
            .iter()
            .enumerate()
        {
            let mi = links[if side.as_u64().unwrap() == 0 {
                "left_ankle_roll_link"
            } else {
                "right_ankle_roll_link"
            }];
            let w = vector(&plan["wrenches"][k]["local_force_moment"][slot]);
            let rf = misarta::se3::rotation_matrix(&data.oMi[mi]);
            let f = rf * na::Vector3::from_column_slice(&w[..3]);
            let moment = rf * na::Vector3::from_column_slice(&w[3..])
                + (rf * na::Vector3::new(0.035, 0., -0.035)).cross(&f);
            let mut wrench = na::DVector::zeros(6);
            wrench.rows_mut(0, 3).copy_from(&moment);
            wrench.rows_mut(3, 3).copy_from(&f);
            let jac = misarta::jacobian::compute_joint_jacobian_from_data(&model, &q, &data, mi);
            required -= jac.transpose() * wrench;
        }
        base_error = base_error.max(required.rows(0, 6).amax());
        for (j, &mi) in ids.iter().enumerate() {
            let damping = if names[j].contains("knee") {
                6.95
            } else if names[j].contains("hip") {
                2.75
            } else if names[j].contains("ankle") {
                1.167
            } else {
                1.0
            };
            let vi = model.v_idx[mi];
            let tau = required[vi] + 0.01 * a[vi] + damping * v[vi];
            torque_error = torque_error.max((tau - plan["torque"][k][j].as_f64().unwrap()).abs());
        }
        checked += 1;
    }
    println!(
        "{}",
        serde_json::json!({"samples_checked":checked,"max_com_error_m":com_error,
        "max_angular_momentum_error":h_error,"max_joint_torque_error_nm":torque_error,
        "max_base_residual_mixed_nm_n":base_error})
    );
    assert!(
        checked == plan["q"].as_array().unwrap().len(),
        "infeasible samples"
    );
    assert!(
        com_error < 1e-6 && h_error < 0.02 && torque_error < 0.5 && base_error < 1.0,
        "independent dynamics check failed"
    );
}
