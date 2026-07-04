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
                let mut mesh_n = 0;
                let mut sphere_n = 0;
                let mut box_n = 0;
                let mut cyl_n = 0;
                let mut cap_n = 0;
                let mut mesh_verts_sum = 0;
                for v in &l.visuals {
                    match &v.geometry {
                        articara::rbd::model::GeomData::Mesh { mesh, .. } => {
                            mesh_n += 1;
                            mesh_verts_sum += mesh.num_triangles() * 3 * 6;
                        }
                        articara::rbd::model::GeomData::Sphere { .. } => sphere_n += 1,
                        articara::rbd::model::GeomData::Box { .. } => box_n += 1,
                        articara::rbd::model::GeomData::Cylinder { .. } => cyl_n += 1,
                        articara::rbd::model::GeomData::Capsule { .. } => cap_n += 1,
                    }
                }
                println!(
                    "  {name:>14}  mass={mass:7.4}  visuals={vc}  (mesh={mesh_n} sphere={sphere_n} box={box_n} cyl={cyl_n} cap={cap_n})  mesh_verts={mesh_verts_sum}",
                    name = l.name,
                    mass = l.inertial.mass,
                    vc = l.visuals.len(),
                );
            }
        }
        Err(e) => {
            eprintln!("ERR: {e}");
            std::process::exit(1);
        }
    }
}
