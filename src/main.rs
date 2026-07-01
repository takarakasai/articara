mod app;
mod attitude_estimator;
mod camera;
mod dynamics;
mod format;
mod gait;
mod history;
mod isaac;
mod leg_odometry;
mod mesh_ops;
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
#[cfg(feature = "mujoco")]
mod wbc_pipeline;
#[cfg(feature = "viz")]
mod viz_feed;

use std::path::PathBuf;

fn main() -> eframe::Result {
    env_logger::Builder::from_env(
        env_logger::Env::default().default_filter_or("warn"),
    )
    .init();

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

    // ── CLI parsing ────────────────────────────────────────────────────────
    //
    //   --script-headless <file> [model]
    //       Run a Rhai script with no GUI and exit. Original headless mode
    //       (renamed from `--script`); useful for CI / batch runs.
    //
    //   --script <file>
    //       Open the GUI, queue the named Rhai script for auto-run on the
    //       first frame, and open the Script Console so the run is visible.
    //       Combined with `--model <path>` (or a positional model arg) this
    //       gives a fully no-click reproducible session.
    //
    //   --model <path>
    //       Explicit model file. Equivalent to passing the path as the
    //       first positional arg.
    //
    //   <model>          (positional)
    //       Same as `--model <path>` for the legacy 1-arg invocation.

    // Headless mode is checked first — if it matches we never enter the
    // GUI codepath.
    #[cfg(feature = "scripting")]
    {
        if let Some(pos) = args.iter().position(|a| a == "--script-headless") {
            let script_path = args
                .get(pos + 1)
                .expect("--script-headless requires a file path");
            let model_path = args.get(pos + 2);

            let source = std::fs::read_to_string(script_path)
                .unwrap_or_else(|e| {
                    eprintln!("Error reading {script_path}: {e}");
                    std::process::exit(1);
                });

            let mut engine = scripting_model::ModelScriptEngine::new();

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

    // GUI-side flag scan. `--script <path>` queues a startup script;
    // `--model <path>` (or the first positional arg) loads a model. These
    // can be combined for a no-click reproducible session.
    let mut initial_path: Option<PathBuf> = None;
    #[cfg(feature = "scripting")]
    let mut initial_script: Option<PathBuf> = None;
    {
        let mut i = 1usize;
        while i < args.len() {
            match args[i].as_str() {
                #[cfg(feature = "scripting")]
                "--script" => {
                    initial_script = args.get(i + 1).map(PathBuf::from);
                    if initial_script.is_none() {
                        eprintln!("--script requires a file path");
                        std::process::exit(1);
                    }
                    i += 2;
                }
                "--model" => {
                    initial_path = args.get(i + 1).map(PathBuf::from);
                    if initial_path.is_none() {
                        eprintln!("--model requires a file path");
                        std::process::exit(1);
                    }
                    i += 2;
                }
                a if !a.starts_with('-') => {
                    if initial_path.is_none() {
                        initial_path = Some(PathBuf::from(a));
                    }
                    i += 1;
                }
                _ => i += 1,
            }
        }
    }

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
            #[cfg(feature = "scripting")]
            if let Some(script) = initial_script {
                app.queue_initial_script(script);
            }
            Ok(Box::new(app))
        }),
    )
}
