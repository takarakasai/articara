//! Does dropping mesh COLLISION geoms change the DYNAMICS?
fn main() {
    use articara::robot::RobotModel;
    use articara::rbd::model::GeomData;
    let path = std::path::Path::new(
        "/home/takara/work/dp/articara/models/unitree_g1_src/robots/g1_description/g1_23dof.urdf");
    let load = || RobotModel::from_urdf(path).unwrap();

    let full = load();
    let mut stripped = load();
    for l in stripped.links.iter_mut() {
        l.collisions.retain(|c| !matches!(c.geometry, GeomData::Mesh { .. }));
    }
    for (tag, r) in [("full", &full), ("stripped", &stripped)] {
        let m: f64 = r.links.iter().map(|l| l.inertial.mass as f64).sum();
        let nin = r.links.iter().filter(|l| l.inertial.mass > 0.0).count();
        let ncol: usize = r.links.iter().map(|l| l.collisions.len()).sum();
        let nvis: usize = r.links.iter().map(|l| l.visuals.len()).sum();
        println!("{tag:9} mass={m:.6} kg  links_with_inertial={nin}  collisions={ncol}  visuals={nvis}");
    }
    // and does the exported MJCF state inertia explicitly, or leave it to
    // MuJoCo's compiler to infer from geoms?
    use articara::mjcf::{export_mjcf_with_options, MjcfExportOptions};
    let xml = export_mjcf_with_options(&full, MjcfExportOptions::default());
    println!("exported MJCF: <inertial occurrences = {}", xml.matches("<inertial").count());
    println!("               inertiafromgeom       = {}", xml.matches("inertiafromgeom").count());
    println!("               mass=                 = {}", xml.matches("mass=").count());
}
