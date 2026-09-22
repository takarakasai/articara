//! 任意の対応形式間でロボット定義を変換する（URDF / MJCF / SDF / USD / .misa）。
//!
//! 入出力の形式は拡張子から判別する。既存の例は URDF → .misa（`convert_to_misa`）と
//! .misa → URDF（`misa_to_urdf`）の 2 方向だけで、たとえば MuJoCo Menagerie の
//! MJCF を Isaac に載せる経路（mjcf → misa → urdf → convert_urdf.py）が塞がっていた。
//!
//! ```bash
//! cargo run --release --example convert_robot -- <input> <output>
//! ```

use std::path::PathBuf;

use articara::robot::RobotModel;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = std::env::args().collect();
    if args.len() != 3 {
        eprintln!("usage: {} <input> <output>", args[0]);
        eprintln!("  対応: .urdf .xacro .xml(MJCF) .sdf .world .usda .misa");
        std::process::exit(1);
    }
    let src = PathBuf::from(&args[1]);
    let dst = PathBuf::from(&args[2]);
    let model = RobotModel::from_file(&src)?;
    if let Some(dir) = dst.parent() {
        std::fs::create_dir_all(dir)?;
    }
    match dst.extension().and_then(|e| e.to_str()) {
        Some("misa") => model.save_as_misa(&dst)?,
        Some("urdf") | Some("xacro") => model.export_urdf_to_file(&dst)?,
        _ => {
            articara::format::FormatRegistry::default_registry().export(&model, &dst)?;
        }
    }
    println!(
        "[convert_robot] {} -> {} ({} bytes)",
        src.display(),
        dst.display(),
        std::fs::metadata(&dst)?.len()
    );
    Ok(())
}
