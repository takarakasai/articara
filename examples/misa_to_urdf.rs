//! `.misa` → URDF。IsaacLab の `convert_urdf.py --merge-joints` に食わせて
//! USD を作るための一段目（go2_rl `doc/locomotion_rl_playbook.md` 段 3）。
//!
//! namiashi の USD はこの経路で作られていたが、書き出し側の口が公開されて
//! おらず、新機体を通すときに詰まる。逆向き（URDF → .misa）だけが
//! `convert_to_misa` として例になっていた。
//!
//! ```bash
//! cargo run --release --example misa_to_urdf -- <input.misa> <output.urdf>
//! ```

use std::path::PathBuf;

use articara::robot::RobotModel;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = std::env::args().collect();
    if args.len() != 3 {
        eprintln!("usage: {} <input.misa> <output.urdf>", args[0]);
        std::process::exit(1);
    }
    let misa = PathBuf::from(&args[1]);
    let urdf = PathBuf::from(&args[2]);
    let model = RobotModel::from_misa(&misa)?;
    if let Some(dir) = urdf.parent() {
        std::fs::create_dir_all(dir)?;
    }
    model.export_urdf_to_file(&urdf)?;
    println!(
        "[misa_to_urdf] {} -> {} ({} bytes)",
        misa.display(),
        urdf.display(),
        std::fs::metadata(&urdf)?.len()
    );
    Ok(())
}
