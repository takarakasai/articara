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

#[test]
fn one_at_a_time_varies_each_axis_alone() {
    use articara::motion_sim::{SweepAxis, one_at_a_time};
    let nominal = RunConditions::default();
    let conds = one_at_a_time(
        &nominal,
        &[
            SweepAxis::GroundFriction(vec![0.4, 0.7, 1.0]),
            SweepAxis::EffortScale(vec![0.8]),
            SweepAxis::Push { magnitudes: vec![20.0], direction: [0.0, 2.0, 0.0], at: 0.3, duration: 0.1 },
        ],
    );
    let labels: Vec<_> = conds.iter().map(RunConditions::label).collect();
    // 0.7 is nominal friction, so it isn't repeated.
    assert_eq!(
        labels,
        [
            "MuJoCo, on ground",
            "MuJoCo, on ground, μ 0.40",
            "MuJoCo, on ground, μ 1.00",
            "MuJoCo, on ground, effort×0.80",
            "MuJoCo, on ground, push 20 N @ 0.30s",
        ]
    );
    assert_eq!(conds[4].push.as_ref().unwrap().force, [0.0, 20.0, 0.0]);
}

#[test]
fn weak_motors_are_caught_by_the_verdict() {
    let model = namiashi();
    let seq = jump(&model);
    let plan = articara::motion::Trajectory::from_sequence(&model, &seq, &model.joint_positions).unwrap();
    let criteria = articara::motion_run::Criteria::default();
    let verdict = |cond: RunConditions| {
        let run = simulate(&model, &seq, &model.joint_positions, &cond, "v").unwrap();
        RunSummary::new(&model, &run, Some(&plan)).verdict(&criteria)
    };
    // Motors at 30 % can't hold the crouch: tracking error blows up.
    let weak = verdict(RunConditions { effort_scale: 0.3, ..Default::default() });
    assert!(weak.iter().any(|w| w.contains("tracking")), "{weak:?}");
}
