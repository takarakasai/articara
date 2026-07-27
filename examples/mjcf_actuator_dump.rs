fn main() {
    use articara::mjcf::{export_mjcf_with_options, MjcfExportOptions};
    use articara::robot::RobotModel;
    let p = std::env::var("URDF").unwrap_or_else(|_|
        "/home/takara/work/dp/humanoid/kyo46rs_description/urdf/kyo46rs.urdf".into());
    let robot = RobotModel::from_urdf(std::path::Path::new(&p)).unwrap();
    let mut o = MjcfExportOptions { timestep: Some(0.001), ..Default::default() };
    o.add_actuators = true;
    let xml = export_mjcf_with_options(&robot, o);
    for l in xml.lines() {
        let t = l.trim();
        if t.starts_with("<actuator") || t.starts_with("</actuator")
            || t.starts_with("<motor") || t.starts_with("<option") {
            println!("{t}");
        }
    }
}
