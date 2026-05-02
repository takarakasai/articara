//! One-shot converter: URDF (+ optional `.misarta.toml` sidecar) → `.misa`.
//!
//! Usage:
//! ```bash
//! cargo run --example convert_to_misa -- <urdf-path> <output-misa-path>
//! ```
//!
//! The output `.misa` ends up next to the existing model. Mesh references
//! are normalised: `package://name/sub/foo.stl` becomes `sub/foo.stl`
//! relative to the `.misa` location.

use std::path::PathBuf;

use articara::robot::RobotModel;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = std::env::args().collect();
    if args.len() != 3 {
        eprintln!("usage: {} <urdf-path> <output.misa>", args[0]);
        std::process::exit(1);
    }
    let urdf_path = PathBuf::from(&args[1]);
    let misa_path = PathBuf::from(&args[2]);

    println!("Loading URDF: {}", urdf_path.display());
    let mut model = RobotModel::from_urdf(&urdf_path)?;

    // Apply .misarta.toml sidecar if present (poses, sequences, actuators,
    // collision pairs, sensors, etc.)
    let report = model.load_sidecar_config();
    if let Some(report) = report {
        println!(
            "  + sidecar applied: {} loop_closure, {} pose, {}/{} actuator",
            report.n_loop_closures,
            report.n_poses,
            report.n_actuators_applied,
            report.n_actuators_total,
        );
        if !report.unmatched_actuators.is_empty() {
            eprintln!(
                "  ! {} unmatched actuator joint(s): {}",
                report.unmatched_actuators.len(),
                report.unmatched_actuators.join(", "),
            );
        }
    }

    println!(
        "  links: {}  joints: {}  root: '{}'",
        model.links.len(),
        model.joints.len(),
        model.root_link,
    );

    // Make sure the output dir exists
    if let Some(dir) = misa_path.parent() {
        std::fs::create_dir_all(dir)?;
    }

    println!("Writing .misa: {}", misa_path.display());
    model.save_as_misa(&misa_path)?;

    // Verify by re-loading
    println!("Verifying round-trip…");
    let loaded = RobotModel::from_misa(&misa_path)?;
    assert_eq!(model.links.len(), loaded.links.len(), "link count mismatch");
    assert_eq!(model.joints.len(), loaded.joints.len(), "joint count mismatch");
    println!(
        "  ✔ Round-trip OK ({} links, {} joints)",
        loaded.links.len(),
        loaded.joints.len(),
    );

    Ok(())
}
