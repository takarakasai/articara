//! Headless MuJoCo runs of a sequence (`articara::motion_sim`).
#![cfg(feature = "mujoco")]

use std::path::PathBuf;

use articara::motion_run::RunSummary;
use articara::motion_sim::{Push, RunConditions, Support, simulate};
use articara::robot::RobotModel;

fn namiashi() -> RobotModel {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/namiashi/namiashi.misa");
    RobotModel::from_misa(&path).unwrap()
}

fn jump(model: &RobotModel) -> articara::robot::Sequence {
    model.sequences.iter().find(|s| s.name == "jump").unwrap().clone()
}

#[test]
fn ground_run_stands_and_respects_torque_limits() {
    let model = namiashi();
    let seq = jump(&model);
    let run = simulate(&model, &seq, &model.joint_positions, &RunConditions::default(), "nominal").unwrap();

    // Plan time 0 … duration (1 s) + tail (0.5 s).
    assert!(run.times[0].abs() < 0.01);
    assert!((run.times.last().unwrap() - 1.5).abs() < 0.01, "{}", run.times.last().unwrap());
    assert_eq!(run.source, "MuJoCo, on ground");

    // Motors saturate: no joint ever exceeds its declared effort.
    let tau = run.tau.as_ref().unwrap();
    for (c, name) in run.joints.iter().enumerate() {
        let effort = model.joints[model.joint_map[name]].effort;
        if effort > 0.0 {
            let peak = tau[c].iter().fold(0.0f64, |m, v| m.max(v.abs()));
            assert!(peak <= effort * 1.001, "{name}: {peak} > {effort}");
        }
    }

    let summary = RunSummary::new(&model, &run, None);
    let tilt = summary.max_tilt.unwrap().to_degrees();
    assert!(tilt < 30.0, "fell over: tilt {tilt}°");
    assert!(summary.min_base_z.unwrap() > 0.0);
}

#[test]
fn fixed_base_does_not_move_and_push_is_reflected_in_label() {
    let model = namiashi();
    let seq = jump(&model);
    let cond = RunConditions {
        support: Support::Fixed,
        kp_scale: 0.5,
        push: Some(Push { link: None, at: 0.2, force: [30.0, 0.0, 0.0], duration: 0.1 }),
        ..Default::default()
    };
    assert_eq!(cond.label(), "MuJoCo, fixed base, kp×0.50, push 30 N @ 0.20s");
    let run = simulate(&model, &seq, &model.joint_positions, &cond, "hang").unwrap();
    let z = run.base_z.as_ref().unwrap();
    let (lo, hi) = z.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), v| (lo.min(*v), hi.max(*v)));
    assert!(hi - lo < 1e-6, "fixed base moved {lo}..{hi}");
}
