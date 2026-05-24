//! Sanity check: load Go2, auto-detect leg kinematics, run a CHAMP gait
//! controller forward 1 s and verify the IK produces sensible joint
//! targets. No MuJoCo dependency — exercises only the kinematic /
//! gait pipeline.

use articara::gait::{auto_detect_kinematics_config, GaitController};
use articara::mjcf;
use articara::robot::*;
use nalgebra as na;
use quadruped_gait::{GaitConfig, GaitMode, LegId, VelocityCmd};
use std::path::Path;

fn main() {
    let path = Path::new("models/unitree_go2/go2.xml");
    let mut model = mjcf::import_mjcf(path).expect("Load Go2");
    println!(
        "Loaded: {} links, {} joints",
        model.links.len(),
        model.joints.len()
    );

    // Go2 has no separate <body> for the feet — the foot collision sphere
    // is a geom inside each calf body at pos (-0.002, 0, -0.213). Add
    // fixed-joint child bodies so the gait module's `auto_detect_leg_*`
    // helpers can climb the chain calf → thigh → hip.
    let foot_offset_local = na::Vector3::new(-0.002_f32, 0.0, -0.213);
    for (leg, parent_calf) in [
        ("FL_foot", "FL_calf"),
        ("FR_foot", "FR_calf"),
        ("RL_foot", "RL_calf"),
        ("RR_foot", "RR_calf"),
    ] {
        let origin = na::Isometry3::from_parts(
            na::Translation3::from(foot_offset_local),
            na::UnitQuaternion::identity(),
        );
        model
            .add_child(
                parent_calf,
                leg,
                &format!("{leg}_fixed"),
                "fixed",
                origin,
                na::Vector3::z(),
                GeomData::Sphere { radius: 0.022 },
                [0.5, 0.5, 0.5, 1.0],
                0.0,
                0.0,
            )
            .unwrap_or_else(|e| panic!("add_child {leg}: {e}"));
    }
    model.rebuild_misarta_model();

    // Set Go2's standing-pose joint angles (from the MJCF's `home`
    // keyframe: hip=0, thigh=0.9, calf=-1.8) so the auto-detector
    // computes `nominal_foot_body` at the actual standing height rather
    // than at q=0 (= fully-extended legs).
    for (joint_name, q) in [
        ("FL_hip_joint", 0.0),
        ("FL_thigh_joint", 0.9),
        ("FL_calf_joint", -1.8),
        ("FR_hip_joint", 0.0),
        ("FR_thigh_joint", 0.9),
        ("FR_calf_joint", -1.8),
        ("RL_hip_joint", 0.0),
        ("RL_thigh_joint", 0.9),
        ("RL_calf_joint", -1.8),
        ("RR_hip_joint", 0.0),
        ("RR_thigh_joint", 0.9),
        ("RR_calf_joint", -1.8),
    ] {
        let ji = model.joint_map[joint_name];
        model.joint_positions[ji] = q;
    }

    // Auto-detect leg kinematics with the new foot links.
    let foot_links = [
        (LegId::FL, "FL_foot"),
        (LegId::FR, "FR_foot"),
        (LegId::RL, "RL_foot"),
        (LegId::RR, "RR_foot"),
    ];
    let kin = match auto_detect_kinematics_config(&model, &foot_links) {
        Ok(k) => k,
        Err(errs) => {
            for (leg, e) in errs {
                eprintln!("  {leg:?}: {e}");
            }
            panic!("auto_detect_kinematics_config failed");
        }
    };
    println!("\n=== Detected leg kinematics ===");
    for (label, leg) in [("FL", &kin.fl), ("FR", &kin.fr), ("RL", &kin.rl), ("RR", &kin.rr)] {
        println!(
            "  {label}: hip_offset={:?}  upper={:.4}  lower={:.4}  hip_to_thigh_y={:.4}",
            (leg.hip_offset.x, leg.hip_offset.y, leg.hip_offset.z),
            leg.upper_leg_m,
            leg.lower_leg_m,
            leg.hip_to_thigh_y,
        );
        println!(
            "       hip_joint={}  thigh_joint={}  calf_joint={}  foot_link={}",
            leg.hip_joint, leg.thigh_joint, leg.calf_joint, leg.foot_link,
        );
        println!(
            "       nominal_foot_body=({:.3}, {:.3}, {:.3})",
            leg.nominal_foot_body.x, leg.nominal_foot_body.y, leg.nominal_foot_body.z,
        );
    }

    // Build CHAMP controller with the canned trot configuration.
    let cfg = GaitConfig::trot();
    let mut ctrl =
        GaitController::build(&model, kin, cfg, GaitMode::Champ).expect("GaitController::build");
    ctrl.enable();

    // Forward 0.3 m/s, no lateral / yaw.
    ctrl.set_velocity_cmd(VelocityCmd {
        vx: 0.3,
        vy: 0.0,
        wz: 0.0,
    });

    println!("\n=== Tick 1 s @ 100 Hz, vx=0.3 m/s ===");
    let dt = 0.01_f64;
    let mut max_abs_q = [0.0_f64; 12];
    let mut last_targets = [(0usize, 0.0_f64); 12];
    for step in 0..100 {
        let (out, targets, _tau_ff) = ctrl.tick(dt);
        for (i, (_, q)) in targets.iter().enumerate() {
            max_abs_q[i] = max_abs_q[i].max(q.abs());
        }
        last_targets = targets;
        if step == 0 || step == 50 || step == 99 {
            let fl = &out.legs[0];
            println!(
                "  step {step:3}: FL foot_body=({:.3}, {:.3}, {:.3})  phase={:?}  reachable={}",
                fl.foot_body.x, fl.foot_body.y, fl.foot_body.z, fl.phase, fl.reachable,
            );
        }
    }

    println!("\n=== Final joint targets (URDF convention) ===");
    let labels = [
        "FL hip", "FL thigh", "FL calf",
        "FR hip", "FR thigh", "FR calf",
        "RL hip", "RL thigh", "RL calf",
        "RR hip", "RR thigh", "RR calf",
    ];
    for (i, lab) in labels.iter().enumerate() {
        let (ji, q) = last_targets[i];
        let j = &model.joints[ji];
        let inside = q >= j.lower && q <= j.upper;
        let mark = if inside { "OK" } else { "OUT-OF-RANGE" };
        println!(
            "  {lab:<10} ji={ji:>3}  q={q:+.4}  limits=[{:.3},{:.3}]  max|q|seen={:.4}  {mark}",
            j.lower, j.upper, max_abs_q[i],
        );
    }
}
