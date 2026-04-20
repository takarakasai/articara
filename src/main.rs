mod app;
mod camera;
mod dynamics;
mod format;
mod history;
mod isaac;
mod mjcf;
mod primitives;
mod rbd;
mod renderer;
mod robot;
mod sdf;
#[cfg(feature = "scripting")]
mod scripting_model;
mod usd;
mod usd_import;

use std::path::PathBuf;

fn main() -> eframe::Result {
    env_logger::init();

    let args: Vec<String> = std::env::args().collect();

    // --script <file> [model]: run a Rhai script headlessly and exit
    #[cfg(feature = "scripting")]
    {
        if let Some(pos) = args.iter().position(|a| a == "--script") {
            let script_path = args.get(pos + 1).expect("--script requires a file path");
            let model_path = args.get(pos + 2);

            let source = std::fs::read_to_string(script_path)
                .unwrap_or_else(|e| {
                    eprintln!("Error reading {script_path}: {e}");
                    std::process::exit(1);
                });

            let mut engine = scripting_model::ModelScriptEngine::new();

            // Pre-load model if specified
            if let Some(mp) = model_path {
                let robot = robot::RobotModel::from_file(std::path::Path::new(mp))
                    .unwrap_or_else(|e| {
                        eprintln!("Error loading {mp}: {e}");
                        std::process::exit(1);
                    });
                engine.set_model(robot);
            }

            match engine.eval(&source) {
                Ok(lines) => {
                    for line in &lines {
                        println!("{line}");
                    }
                    std::process::exit(0);
                }
                Err(e) => {
                    eprintln!("{e}");
                    std::process::exit(1);
                }
            }
        }
    }

    let initial_path = args.get(1)
        .filter(|a| !a.starts_with('-'))
        .map(PathBuf::from);

    let options = eframe::NativeOptions {
        viewport: eframe::egui::ViewportBuilder::default()
            .with_inner_size([1280.0, 800.0])
            .with_title("Articara - Robot Dynamics Editor")
            .with_decorations(false),
        renderer: eframe::Renderer::Glow,
        depth_buffer: 24,
        multisampling: 4,
        ..Default::default()
    };

    eframe::run_native(
        "Articara",
        options,
        Box::new(move |cc| {
            let mut app = app::ArticaraApp::new(cc);
            if let Some(path) = initial_path {
                app.load_model(path);
            }
            Ok(Box::new(app))
        }),
    )
}
