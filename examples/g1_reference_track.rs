//! Isolated whole-configuration WBC tracking experiment; never changes walk defaults.
use articara::biped::{
    contact::{Anchors, contact_jacobians},
    log::measure_contacts,
    profile::G1_23DOF,
    rig::{BipedRig, RigOptions},
    tasks as bt,
    tick_audit::TickAudit,
};
use misa_wbc::{Dynamics, Formulation, SolveConfig, Solver, tasks};
use nalgebra as na;
use std::io::Write;
fn vec(v: &serde_json::Value) -> Vec<f64> {
    v.as_array()
        .unwrap()
        .iter()
        .map(|x| x.as_f64().unwrap())
        .collect()
}
fn quintic(q0: f64, v0: f64, a0: f64, q1: f64, v1: f64, a1: f64, h: f64, u: f64) -> [f64; 3] {
    let c0 = q0;
    let c1 = v0 * h;
    let c2 = a0 * h * h / 2.;
    let d = q1 - c0 - c1 - c2;
    let e = v1 * h - c1 - 2. * c2;
    let f = a1 * h * h - 2. * c2;
    let c3 = 10. * d - 4. * e + f / 2.;
    let c4 = -15. * d + 7. * e - f;
    let c5 = 6. * d - 3. * e + f / 2.;
    [
        c0 + u * (c1 + u * (c2 + u * (c3 + u * (c4 + u * c5)))),
        (c1 + u * (2. * c2 + u * (3. * c3 + u * (4. * c4 + 5. * c5 * u)))) / h,
        (2. * c2 + u * (6. * c3 + u * (12. * c4 + 20. * c5 * u))) / (h * h),
    ]
}
struct Reference {
    p: serde_json::Value,
    t: Vec<f64>,
    names: Vec<String>,
    r: na::UnitQuaternion<f64>,
}
impl Reference {
    fn sample(&self, t: f64) -> (Vec<[f64; 3]>, Vec<usize>) {
        let k = self
            .t
            .partition_point(|x| *x <= t)
            .saturating_sub(1)
            .min(self.t.len() - 2);
        let h = self.t[k + 1] - self.t[k];
        let u = ((t - self.t[k]) / h).clamp(0., 1.);
        let mut out = Vec::new();
        for j in 0..self.names.len() + 3 {
            let (vel, acc, jj) = if j < 3 {
                ("base_velocity", "base_acceleration", j)
            } else {
                ("qd", "qdd", j - 3)
            };
            let read = |field: &str, i: usize, col: usize| self.p[field][i][col].as_f64().unwrap();
            out.push(quintic(
                read("q", k, j),
                read(vel, k, jj),
                read(acc, k, jj),
                read("q", k + 1, j),
                read(vel, k + 1, jj),
                read(acc, k + 1, jj),
                h,
                u,
            ));
        }
        let a: Vec<usize> = self.p["contact_sides"][k]
            .as_array()
            .unwrap()
            .iter()
            .map(|v| v.as_u64().unwrap() as usize)
            .collect();
        let b: Vec<usize> = self.p["contact_sides"][k + 1]
            .as_array()
            .unwrap()
            .iter()
            .map(|v| v.as_u64().unwrap() as usize)
            .collect();
        let active = if u < 1e-9 {
            a
        } else if u > 1. - 1e-9 {
            b
        } else {
            a.into_iter().filter(|s| b.contains(s)).collect()
        };
        (out, active)
    }
    fn planned_wrench(&self, t: f64, side: usize) -> [f64; 6] {
        let k = self.t.partition_point(|x| *x <= t).saturating_sub(1);
        let node = &self.p["wrenches"][k];
        assert!(!node.is_null(), "planned wrench is missing at t={t}");
        let sides = node["sides"].as_array().unwrap();
        if let Some(slot) = sides
            .iter()
            .position(|v| v.as_u64().unwrap() as usize == side)
        {
            let values = vec(&node["local_force_moment"][slot]);
            values.try_into().unwrap()
        } else {
            [0.; 6]
        }
    }
    fn next_liftoff(&self, t: f64, side: usize) -> Option<f64> {
        let start = self.t.partition_point(|x| *x <= t);
        for k in start..self.t.len() {
            let before = self.p["contact_sides"][k - 1].as_array().unwrap();
            let after = self.p["contact_sides"][k].as_array().unwrap();
            let has =
                |v: &Vec<serde_json::Value>| v.iter().any(|x| x.as_u64().unwrap() as usize == side);
            if has(before) && !has(after) {
                return Some(self.t[k]);
            }
        }
        None
    }
    fn reset(&self, rig: &mut BipedRig, t: f64) {
        let (x, _) = self.sample(t);
        let data = rig.sim.mj_data_mut();
        data.qvel_mut().fill(0.);
        data.qpos_mut()[..7].copy_from_slice(&[
            x[0][0], x[1][0], x[2][0], self.r.w, self.r.i, self.r.j, self.r.k,
        ]);
        for (i, name) in self.names.iter().enumerate() {
            let joint = data.joint(name).unwrap();
            joint.view_mut(data).qpos[0] = x[i + 3][0];
            rig.robot.joint_positions[rig.robot.joint_map[name]] = x[i + 3][0];
        }
        data.forward();
    }
}
fn main() {
    let args: Vec<_> = std::env::args().collect();
    assert_eq!(
        args.len(),
        4,
        "g1_reference_track reference.json new_output_dir screen|track"
    );
    let out = std::path::Path::new(&args[2]);
    assert!(!out.exists(), "output already exists");
    std::fs::create_dir_all(out).unwrap();
    let p: serde_json::Value = serde_json::from_slice(&std::fs::read(&args[1]).unwrap()).unwrap();
    let r = na::Matrix3::from_fn(|i, j| p["base_rotation"][i][j].as_f64().unwrap());
    let reference = Reference {
        t: vec(&p["times"]),
        names: p["joint_names"]
            .as_array()
            .unwrap()
            .iter()
            .map(|n| n.as_str().unwrap().to_owned())
            .collect(),
        r: na::UnitQuaternion::from_matrix(&r),
        p,
    };
    let mut options = RigOptions::from_profile(&G1_23DOF);
    options.sim_dt = 0.0005;
    options.joint_damping_overrides = vec![
        ("hip".into(), 2.75),
        ("knee".into(), 6.95),
        ("ankle".into(), 1.167),
    ];
    let mut rig = BipedRig::build(G1_23DOF, &options);
    rig.qd_clamp = 1e6;
    let number = |name: &str, default: f64| {
        std::env::var(name)
            .ok()
            .map(|v| v.parse::<f64>().unwrap())
            .unwrap_or(default)
    };
    let dt = number("TRACK_DT", 0.005);
    let base_weight = number("TRACK_BASE_WEIGHT", 4.0);
    let contact_kp = number("TRACK_CONTACT_KP", 150.0);
    let planned_wrench = std::env::var("TRACK_WRENCH_REF").ok().as_deref() == Some("planned");
    let event_advance = number("TRACK_EVENT_ADVANCE", 1.0);
    let swing_kp = number("TRACK_SWING_KP", 150.0);
    let swing_kd = number("TRACK_SWING_KD", 25.0);
    let support_bias = number("TRACK_SUPPORT_BIAS", 0.0);
    let support_mode = std::env::var("TRACK_SUPPORT_MODE").unwrap_or_else(|_| "cop".into());
    assert!(support_mode == "cop" || support_mode == "com_lag");
    assert!((0.0..=0.04).contains(&support_bias));
    assert!(event_advance >= 1.0);
    let substeps = (dt / options.sim_dt).round() as u32;
    assert!(substeps > 0 && (substeps as f64 * options.sim_dt - dt).abs() < 1e-10);
    let end = *reference.t.last().unwrap();
    let n = (end / dt).round() as usize;
    let mut self_ticks = 0;
    let mut nonfoot_ticks = 0;
    let mut max_ref_v = 0.0_f64;
    let mut min_margin = f64::INFINITY;
    if args[3] == "screen" {
        for tick in 0..=n {
            let t = tick as f64 * dt;
            reference.reset(&mut rig, t);
            let (x, _) = reference.sample(t);
            for (j, name) in reference.names.iter().enumerate() {
                let ji = rig.robot.joint_map[name];
                let mi = rig.a2m[ji].unwrap();
                let row = rig.model.v_idx[mi] - 6;
                max_ref_v = max_ref_v.max(x[j + 3][1].abs() / rig.v_max[row]);
                min_margin = min_margin
                    .min((x[j + 3][0] - rig.q_min[row]).min(rig.q_max[row] - x[j + 3][0]));
            }
            let contacts = rig.sim.contacts();
            self_ticks += usize::from(contacts.iter().any(|c| c.is_self_collision()));
            nonfoot_ticks += usize::from(contacts.iter().any(|c| {
                !c.is_self_collision()
                    && !rig
                        .prof
                        .foot_links
                        .iter()
                        .any(|f| c.body1 == *f || c.body2 == *f)
            }));
        }
        let result = serde_json::json!({"samples":n+1,"self_contact_ticks":self_ticks,"nonfoot_ground_ticks":nonfoot_ticks,"max_interpolated_velocity_utilization":max_ref_v,"min_position_margin_rad":min_margin,"scope":"MuJoCo profile primitive collision geometry; 5 ms samples; no mesh certification"});
        std::fs::write(
            out.join("summary.json"),
            serde_json::to_vec_pretty(&result).unwrap(),
        )
        .unwrap();
        println!("{result}");
        return;
    }
    assert_eq!(args[3], "track");
    reference.reset(&mut rig, 0.);
    let mut audit = TickAudit::create(out.join("audit.csv").to_str().unwrap(), &rig);
    let mut csv = std::io::BufWriter::new(std::fs::File::create(out.join("tracking.csv")).unwrap());
    writeln!(csv,"t,reference_t,q_error_max,tilt,fz_l,fz_r,qp_fz_l,qp_fz_r,planned_mask,actual_mask,degraded,eom_residual,foot_l_z,foot_r_z").unwrap();
    let mut diagnostics =
        std::io::BufWriter::new(std::fs::File::create(out.join("support.csv")).unwrap());
    writeln!(diagnostics,"t,reference_t,com_x,com_y,com_z,com_ref_x,com_ref_y,com_ref_z,base_x,base_y,base_z,base_ref_x,base_ref_y,base_ref_z,cop_rx,cop_ry,fz_r,slip_r,foot_r_rotation_error,foot_r_z,foot_l_z,qp_fz_r,actual_mask,planned_mask,support_bias_y").unwrap();
    let initial = rig.sync();
    let right_rotation_initial = misarta::se3::rotation_matrix(&initial.data.oMi[rig.foot_mi[1]]);
    let mut solver = Solver::new();
    let mut anchors = Anchors::new();
    let mut reason = "finished".to_owned();
    let mut last_t = 0.;
    let mut stable = 0.;
    let mut max_q_err = 0.0_f64;
    let mut degraded_count = 0;
    let mut unsupported_s = 0.0;
    let mut max_unsupported_s = 0.0_f64;
    let mut reference_t = 0.0_f64;
    let mut advanced_ticks = 0usize;
    let config = bt::ContactCfg {
        kp_c: contact_kp,
        kd_c: 2.0 * contact_kp.sqrt(),
        anchor_leak: 0.2,
        anchor_leak_rot: 0.,
        sole_offset_local: [0.035, 0., -0.035],
        friction_mu: 0.6,
        cop_half: (0.085, 0.030),
        mu_torsion: 0.01,
        f_max_per_foot: rig.weight() * 2.,
        no_patch: false,
        dt,
    };
    let limits = tasks::JointLimitCbf {
        q_min: rig.q_min.clone(),
        q_max: rig.q_max.clone(),
        v_max: rig.v_max.clone(),
        a_max: na::DVector::from_element(rig.na, 50.),
        alpha1: na::DVector::from_element(rig.na, 10.),
        alpha2: na::DVector::from_element(rig.na, 10.),
        alpha3: na::DVector::from_element(rig.na, 10.),
    };
    for tick in 0..n {
        let t = tick as f64 * dt;
        let (x, planned) = reference.sample(reference_t);
        let st = rig.sync();
        let measured = measure_contacts(&rig, &st.data, 0.6);
        let contacts = rig.sim.contacts();
        let sides: Vec<_> = planned
            .iter()
            .copied()
            .filter(|&side| {
                contacts.iter().any(|c| {
                    !c.is_self_collision()
                        && (c.body1 == rig.prof.foot_links[side]
                            || c.body2 == rig.prof.foot_links[side])
                })
            })
            .collect();
        unsupported_s = if sides.is_empty() {
            unsupported_s + dt
        } else {
            0.0
        };
        max_unsupported_s = max_unsupported_s.max(unsupported_s);
        if unsupported_s > 0.1 {
            reason = "no_planned_foot_in_actual_contact_100ms".into();
            break;
        }
        let stance: Vec<_> = sides.iter().map(|&s| rig.foot_mi[s]).collect();
        for s in 0..2 {
            if !sides.contains(&s) {
                anchors.release(s);
            }
        }
        let (jc, djc) = contact_jacobians(
            &rig.model, &st.q, &st.v, &st.data, &st.v_dvec, &stance, rig.nv,
        );
        let dyns = Dynamics::new(Formulation::Explicit, &st.mass, &st.h, &jc, rig.na);
        let mut qa = na::DVector::zeros(rig.na);
        let mut va = qa.clone();
        let mut desired = na::DVector::zeros(rig.nv);
        let mut qerr = 0.0_f64;
        let rot = st.body_quat.to_rotation_matrix();
        let er = (reference.r * st.body_quat.inverse()).scaled_axis();
        let angular = rot.inverse() * (er * 150. - na::Vector3::from(st.v_ang_w) * 25.);
        desired.rows_mut(0, 3).copy_from(&angular);
        let world_v = rot * na::Vector3::new(st.v[3], st.v[4], st.v[5]);
        let mut lin = na::Vector3::zeros();
        let ref_k = reference
            .t
            .partition_point(|v| *v <= reference_t)
            .saturating_sub(1);
        let com_ref_y = reference.p["com"][ref_k][1].as_f64().unwrap();
        let com_y = misarta::centroidal::compute_com(&rig.model, &st.q)[1];
        let bias_y = if sides.contains(&1) && !sides.contains(&0) {
            if support_mode == "com_lag" {
                -(com_y - com_ref_y).max(0.0).min(support_bias)
            } else if measured.cop[1][1] < -0.020 {
                -support_bias
            } else {
                0.0
            }
        } else {
            0.0
        };
        for j in 0..3 {
            let bias = if j == 1 { bias_y } else { 0.0 };
            lin[j] =
                x[j][2] + 150. * (x[j][0] + bias - st.body_pos[j]) + 25. * (x[j][1] - world_v[j]);
        }
        // Body-frame free-flyer acceleration includes transport term.
        let linear = rot.inverse() * lin
            - na::Vector3::new(st.v[0], st.v[1], st.v[2])
                .cross(&na::Vector3::new(st.v[3], st.v[4], st.v[5]));
        desired.rows_mut(3, 3).copy_from(&linear);
        for (j, name) in reference.names.iter().enumerate() {
            let ji = rig.robot.joint_map[name];
            let mi = rig.a2m[ji].unwrap();
            let vi = rig.model.v_idx[mi];
            let err = x[j + 3][0] - st.q[rig.model.q_idx[mi]];
            qerr = qerr.max(err.abs());
            qa[vi - 6] = st.q[rig.model.q_idx[mi]];
            va[vi - 6] = st.v[vi];
            let swing = (name.starts_with("left_") && !planned.contains(&0))
                || (name.starts_with("right_") && !planned.contains(&1));
            let (kp, kd) = if swing {
                (swing_kp, swing_kd)
            } else {
                (150., 25.)
            };
            desired[vi] = x[j + 3][2] + kp * err + kd * (x[j + 3][1] - st.v[vi]);
        }
        let base = tasks::box_bound(dyns.tau(), &rig.torque_max)
            + bt::joint_limits(
                dyns.qddot(),
                &rig.actuated(),
                &qa,
                &va,
                &limits,
                rig.na,
                rig.nv,
            );
        let p0 = bt::contact_level(
            &dyns,
            base,
            &jc,
            &djc,
            &st.data,
            &st.v_dvec,
            &stance,
            rig.foot_mi[0],
            &mut anchors,
            &|_| 1. / sides.len() as f64,
            &config,
            &|_| bt::ContactMode::Rigid,
            &|_, _| None,
        );
        let eom = dyns.dynamics_task().unwrap();
        // Resolve redundant contact forces before minimizing motor effort.
        // Target zero sole tangential force/moment; required net moment is
        // still enforced by the equations of motion at higher priority.
        let mut wrench_reference = na::DVector::zeros(6 * sides.len());
        for (slot, &side) in sides.iter().enumerate() {
            let mi = rig.foot_mi[side];
            let rf = misarta::se3::rotation_matrix(&st.data.oMi[mi]);
            let local = if planned_wrench {
                reference.planned_wrench(reference_t, side)
            } else {
                [0., 0., rig.weight() / sides.len() as f64, 0., 0., 0.]
            };
            let force = rf * na::Vector3::new(local[0], local[1], local[2]);
            let moment = rf * na::Vector3::new(local[3], local[4], local[5])
                + (rf * na::Vector3::new(0.035, 0.0, -0.035)).cross(&force);
            wrench_reference.rows_mut(6 * slot, 3).copy_from(&moment);
            wrench_reference.rows_mut(6 * slot + 3, 3).copy_from(&force);
        }
        let levels = vec![
            eom.clone(),
            p0.task,
            tasks::cartesian_acceleration(
                dyns.qddot(),
                &na::DMatrix::from_diagonal(&na::DVector::from_fn(rig.nv, |i, _| {
                    if (3..6).contains(&i) {
                        base_weight
                    } else {
                        1.0
                    }
                })),
                &na::DVector::zeros(rig.nv),
                &na::DVector::from_fn(rig.nv, |i, _| {
                    desired[i]
                        * if (3..6).contains(&i) {
                            base_weight
                        } else {
                            1.0
                        }
                }),
            ),
            tasks::regularize(&dyns.forces(), &wrench_reference),
            tasks::regularize(dyns.tau(), &na::DVector::zeros(rig.na)),
        ];
        let sol = solver
            .solve(
                &levels,
                &SolveConfig {
                    feasibility_tol: 1e-8,
                    ..Default::default()
                },
            )
            .unwrap();
        let degraded = !matches!(sol.status, misa_wbc::SolveStatus::Optimal);
        degraded_count += usize::from(degraded);
        let residual = (&eom.a * &sol.x - &eom.b).amax();
        if residual > 1e-5 {
            reason = "eom_residual".into();
            break;
        }
        let extracted = dyns.extract(&sol.x);
        let mut tau = vec![0.; rig.robot.joints.len()];
        for (ji, vi) in rig.actuated() {
            tau[ji] = extracted.tau[vi - 6].clamp(-rig.torque_max[vi - 6], rig.torque_max[vi - 6]);
        }
        let mut fq = [0.; 2];
        for (slot, &side) in sides.iter().enumerate() {
            fq[side] = extracted.forces[6 * slot + 5];
        }
        let pre = audit.sample(&rig);
        rig.sim.set_wbc_torques(&tau);
        rig.step(substeps);
        let post = rig.sync();
        let after = measure_contacts(&rig, &post.data, 0.6);
        audit.write(
            &rig,
            t,
            dt,
            &pre,
            &tau,
            [measured.f_w[0][2], measured.f_w[1][2]],
            fq,
            [after.f_w[0][2], after.f_w[1][2]],
        );
        let (roll, pitch, _) = post.body_quat.euler_angles();
        let tilt = roll.abs().max(pitch.abs());
        let lf = rig.sim.body_world_position(rig.prof.foot_links[0]).unwrap()[2];
        let rf = rig.sim.body_world_position(rig.prof.foot_links[1]).unwrap()[2];
        writeln!(csv,"{t:.6},{reference_t:.6},{qerr:.9},{tilt:.9},{:.9},{:.9},{:.9},{:.9},{},{},{},{residual:.12},{lf:.9},{rf:.9}",after.f_w[0][2],after.f_w[1][2],fq[0],fq[1],planned.iter().fold(0,|a,s|a|(1<<s)),sides.iter().fold(0,|a,s|a|(1<<s)),usize::from(degraded)).unwrap();
        let com = misarta::centroidal::compute_com(&rig.model, &post.q);
        let ref_k = reference
            .t
            .partition_point(|v| *v <= reference_t)
            .saturating_sub(1);
        let com_ref = vec(&reference.p["com"][ref_k]);
        let right_rotation = misarta::se3::rotation_matrix(&post.data.oMi[rig.foot_mi[1]]);
        let right_rot_error =
            na::UnitQuaternion::from_matrix(&(right_rotation_initial.transpose() * right_rotation))
                .angle();
        writeln!(diagnostics,"{t:.6},{reference_t:.6},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{right_rot_error:.9},{rf:.9},{lf:.9},{:.9},{},{},{bias_y:.9}",
            com[0],com[1],com[2],com_ref[0],com_ref[1],com_ref[2],post.body_pos[0],post.body_pos[1],post.body_pos[2],x[0][0],x[1][0],x[2][0],
            after.cop[1][0],after.cop[1][1],after.f_w[1][2],after.slip[1],fq[1],
            sides.iter().fold(0,|a,s|a|(1<<s)),planned.iter().fold(0,|a,s|a|(1<<s))).unwrap();
        max_q_err = max_q_err.max(qerr);
        last_t = t + dt;
        let mut step = dt;
        if event_advance > 1.0 {
            for &side in &planned {
                if !sides.contains(&side)
                    && sides.iter().any(|&s| s != side)
                    && reference.planned_wrench(reference_t, side)[2] < 50.0
                {
                    if let Some(edge) = reference.next_liftoff(reference_t, side) {
                        step = step.max((edge - reference_t).min(dt * event_advance));
                    }
                }
            }
        }
        if step > dt + 1e-9 {
            advanced_ticks += 1;
        }
        reference_t = (reference_t + step).min(end);
        if t >= end - 2.
            && tilt < 0.05
            && after.f_w.iter().all(|f| f[2] > 0.1 * rig.weight())
            && post.v_dvec.norm() < 0.2
        {
            stable += dt;
        }
        if tilt > 0.3 {
            reason = "tilt".into();
            break;
        }
        if rig.sim.contacts().iter().any(|c| c.is_self_collision()) {
            self_ticks += 1;
            reason = "self_collision".into();
            break;
        }
    }
    let result = serde_json::json!({"last_t":last_t,"termination":reason,"stable_terminal_seconds":stable,"max_joint_tracking_error_rad":max_q_err,"degraded_ticks":degraded_count,"self_contact_ticks":self_ticks,"motion_completed":reason=="finished"&&stable>=1.99,"dt":dt,"sim_dt":0.0005,"base_weight":base_weight,"contact_kp":contact_kp,"planned_wrench":planned_wrench,"event_advance":event_advance,"swing_kp":swing_kp,"swing_kd":swing_kd,"support_bias":support_bias,"support_mode":support_mode,"advanced_ticks":advanced_ticks,"reference_t":reference_t,"max_unsupported_s":max_unsupported_s});
    std::fs::write(
        out.join("summary.json"),
        serde_json::to_vec_pretty(&result).unwrap(),
    )
    .unwrap();
    println!("{result}");
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn quintic_endpoints() {
        let left = quintic(0.2, 0.3, -0.4, 0.8, -0.2, 0.1, 0.7, 0.);
        let right = quintic(0.2, 0.3, -0.4, 0.8, -0.2, 0.1, 0.7, 1.);
        for (a, b) in left.iter().zip([0.2, 0.3, -0.4]) {
            assert!((a - b).abs() < 1e-12);
        }
        for (a, b) in right.iter().zip([0.8, -0.2, 0.1]) {
            assert!((a - b).abs() < 1e-12);
        }
    }
}
