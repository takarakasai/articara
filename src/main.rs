mod app;
mod camera;
mod format;
mod ik;
mod isaac;
mod mjcf;
mod primitives;
mod renderer;
mod robot;
mod sdf;

use std::path::PathBuf;

fn main() -> eframe::Result {
    env_logger::init();

    let initial_path = std::env::args().nth(1).map(PathBuf::from);

    let options = eframe::NativeOptions {
        viewport: eframe::egui::ViewportBuilder::default()
            .with_inner_size([1280.0, 800.0])
            .with_title("RoboView - URDF Dynamics Editor"),
        renderer: eframe::Renderer::Glow,
        depth_buffer: 24,
        multisampling: 4,
        ..Default::default()
    };

    eframe::run_native(
        "RoboView",
        options,
        Box::new(move |cc| {
            let mut app = app::RoboViewApp::new(cc);
            if let Some(path) = initial_path {
                app.load_model(path);
            }
            Ok(Box::new(app))
        }),
    )
}
