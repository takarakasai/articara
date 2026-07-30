//! ChickenHead demo data generator (kinematic).
//!
//! ChickenHead is a **pure kinematic** controller: for a measured trunk pitch
//! `θ_trunk` it commands the head joint to `q* = sign·(θ_ref − θ_trunk)`
//! (clamped to the joint limits), so the head holds a fixed world pitch. This
//! generator feeds a representative "bound / rocking" trunk-pitch disturbance
//! through the **real** [`articara::chicken_head::ChickenHeadConfig`] (built
//! from namiashi's `arm_pitch_joint`, so the axis sign and limits are the
//! model's own) and records the head's world pitch with ChickenHead on vs off.
//!
//! With ChickenHead **on** the head stays level (world pitch ≈ 0); **off** it
//! rides the trunk. The `scripts/chicken_head_demo.py` renderer turns the CSV
//! into the demo video.
//!
//! Writes CSV (`--out`, default `/tmp/chicken_head_demo.csv`) columns (rad):
//!   t, trunk_pitch, head_q_on, head_world_on, head_q_off, head_world_off
//!
//! Run:  cargo run --example chicken_head_demo

use std::path::PathBuf;

use articara::chicken_head::{ChickenHeadConfig, StabAxis};
use articara::robot::RobotModel;

fn namiashi_misa() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests")
        .join("fixtures")
        .join("namiashi")
        .join("namiashi.misa")
}

/// Representative trunk-pitch disturbance (rad) at time `t` (s): a lively
/// rocking motion — a dominant slow rock plus a faster harmonic, with a brief
/// "stumble" lurch near the middle to show the head recovering.
fn trunk_pitch_disturbance(t: f64) -> f64 {
    let tau = std::f64::consts::TAU;
    let rock = 12.0_f64.to_radians() * (tau * 0.55 * t).sin();
    let harmonic = 4.0_f64.to_radians() * (tau * 1.4 * t + 0.7).sin();
    // Gaussian lurch centred at t = 4.5 s.
    let lurch = 14.0_f64.to_radians() * (-((t - 4.5) / 0.35).powi(2)).exp();
    rock + harmonic + lurch
}

fn main() {
    let out_path = std::env::args()
        .skip_while(|a| a != "--out")
        .nth(1)
        .unwrap_or_else(|| "/tmp/chicken_head_demo.csv".to_string());

    let file = namiashi_misa();
    if !file.exists() {
        eprintln!("namiashi fixture missing at {} — cannot generate demo", file.display());
        std::process::exit(1);
    }
    let robot = RobotModel::from_misa(&file).expect("load namiashi .misa");

    // The real shipping controller, built from the model (axis sign + limits).
    let mut chicken = ChickenHeadConfig::for_joint(&robot, "arm_pitch_joint", StabAxis::Pitch)
        .expect("namiashi arm_pitch_joint");
    chicken.enabled = true;
    chicken.target_world_angle = 0.0; // hold the head level

    let dt = 1.0 / 120.0; // 120 fps of data
    let duration = 8.0;
    let n = (duration / dt) as usize;

    let mut csv =
        String::from("t,trunk_pitch,head_q_on,head_world_on,head_q_off,head_world_off\n");
    let mut trunk_rms = 0.0;
    let mut on_rms = 0.0;
    let mut off_rms = 0.0;

    for k in 0..n {
        let t = k as f64 * dt;
        let trunk_pitch = trunk_pitch_disturbance(t);
        let trunk_quat = nalgebra::UnitQuaternion::from_euler_angles(0.0, trunk_pitch, 0.0);

        // ChickenHead ON: real controller output.
        let head_q_on = chicken.target_angle(&trunk_quat);
        let head_world_on = trunk_pitch + chicken.axis_sign * head_q_on;

        // ChickenHead OFF: head fixed at neutral, rides the trunk.
        let head_q_off = 0.0;
        let head_world_off = trunk_pitch + chicken.axis_sign * head_q_off;

        csv.push_str(&format!(
            "{t:.4},{trunk_pitch:.6},{head_q_on:.6},{head_world_on:.6},\
             {head_q_off:.6},{head_world_off:.6}\n"
        ));
        trunk_rms += trunk_pitch.powi(2);
        on_rms += head_world_on.powi(2);
        off_rms += head_world_off.powi(2);
    }
    std::fs::write(&out_path, csv).expect("write csv");

    let deg = |x: f64| (x / n as f64).sqrt().to_degrees();
    eprintln!("wrote {out_path} ({n} rows @ {:.0} fps)", 1.0 / dt);
    eprintln!(
        "trunk-pitch RMS = {:.2}°   head-world RMS: ON = {:.2}°  OFF = {:.2}°  \
         (ON ≈ 0 ⇒ head held level)",
        deg(trunk_rms),
        deg(on_rms),
        deg(off_rms),
    );
}
