//! Sanity check: load Unitree Go2 (MuJoCo Menagerie MJCF) and report
//! joints / links / limits / actuators. Doubles as a smoke test for
//! `<default class>` and `<actuator>` parsing in the MJCF importer.
//!
//! Asset is not versioned here — fetch the model first with:
//!
//! ```bash
//! curl -fsSL -o /tmp/m.zip https://codeload.github.com/google-deepmind/mujoco_menagerie/zip/refs/heads/main
//! unzip -q /tmp/m.zip "mujoco_menagerie-main/unitree_go2/*" -d ./
//! mv mujoco_menagerie-main/unitree_go2 models/
//! rm -r mujoco_menagerie-main /tmp/m.zip
//! ```
//!
//! Then: `cargo run --no-default-features --example go2_load`

use articara::mjcf;
use std::path::Path;

fn main() {
    let path = Path::new("models/unitree_go2/go2.xml");
    println!("Loading: {}", path.display());
    match mjcf::import_mjcf(path) {
        Ok(model) => {
            println!("OK: name={}", model.name);
            println!("  links:           {}", model.links.len());
            println!("  joints:          {}", model.joints.len());
            let movable = model
                .joints
                .iter()
                .filter(|j| j.joint_type != "fixed")
                .count();
            println!("  movable joints:  {movable}");
            println!("  loop_closures:   {}", model.loop_closures.len());
            println!("  mimics:          {}", model.mimics.len());
            println!("  sensors:         {}", model.sensors.len());
            println!("  root_link:       {}", model.root_link);
            println!("--- Joints ---");
            for j in &model.joints {
                let rpy = j.origin.rotation.euler_angles();
                let t = j.origin.translation;
                println!(
                    "  {name:>18}  type={ty:<10} axis=[{ax:.2} {ay:.2} {az:.2}] t=[{tx:.3} {ty2:.3} {tz:.3}] rpy=[{r:.2} {p:.2} {y:.2}] mode={mode:<11} range=[{lo:.3},{up:.3}] eff={eff:.2} damp={d:.3} arm={a:.4}",
                    name = j.name,
                    ty = j.joint_type,
                    ax = j.axis.x, ay = j.axis.y, az = j.axis.z,
                    tx = t.x, ty2 = t.y, tz = t.z,
                    r = rpy.0, p = rpy.1, y = rpy.2,
                    mode = j.actuator_mode.label(),
                    lo = j.lower, up = j.upper,
                    eff = j.effort,
                    d = j.joint_damping,
                    a = j.armature,
                );
            }
            println!("--- Links ---");
            for l in &model.links {
                println!(
                    "  {name:>14}  mass={mass:7.4}  visuals={vc}  collisions={cc}",
                    name = l.name,
                    mass = l.inertial.mass,
                    vc = l.visuals.len(),
                    cc = l.collisions.len(),
                );
            }
        }
        Err(e) => {
            eprintln!("ERR: {e}");
            std::process::exit(1);
        }
    }
}
