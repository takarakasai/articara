mod app;
mod attitude_estimator;
mod camera;
mod dynamics;
mod format;
mod gait;
mod history;
mod isaac;
mod mesh_paths;
mod mjcf;
#[cfg(feature = "mujoco")]
mod mujoco_sim;
#[cfg(feature = "mujoco")]
mod mujoco_version;
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

    // MuJoCo runtime version pre-check. mujoco-rs panics deep in its
    // crate when the linked libmujoco doesn't match its FFI bindings;
    // pre-checking here surfaces the mismatch as a clear log line and
    // a cached flag the dynamics panel can read, instead of a cryptic
    // backtrace later. See `src/mujoco_version.rs`.
    #[cfg(feature = "mujoco")]
    {
        let r = mujoco_version::init();
        match r {
            mujoco_version::CheckResult::Compatible(v) => {
                log::info!("MuJoCo runtime {v} matches expected version — OK");
            }
            mujoco_version::CheckResult::Mismatch { .. } => {
                log::error!("{}", r.diagnostic());
                eprintln!("⚠ {}", r.diagnostic());
                eprintln!(
                    "    The app will keep running but MuJoCo-backed \
                     features will be disabled to avoid the panic. \
                     See MUJOCO_SETUP.md for installation steps."
                );
            }
        }
    }

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
