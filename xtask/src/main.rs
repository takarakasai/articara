// xtask: cross-platform task runner for articara.
//
// Detects the local MuJoCo installation, exports the environment variables
// required by `mujoco-rs`'s build.rs, and forwards all remaining arguments
// to `cargo`. Replaces setup-mujoco.sh / setup-mujoco.ps1.
//
// Usage:
//   cargo xtask build --features mujoco
//   cargo xtask run --features mujoco --example demo
//   cargo xtask test --features mujoco

use std::env;
use std::path::{Path, PathBuf};
use std::process::{Command, ExitCode};

struct MujocoEnv {
    /// Directory containing the dynamic library (libmujoco.dylib / .so / mujoco.dll).
    lib_dir: PathBuf,
    /// MuJoCo install root (used for DYLD_FRAMEWORK_PATH on macOS, MUJOCO_HOME on Windows).
    root: PathBuf,
}

fn main() -> ExitCode {
    let args: Vec<String> = env::args().skip(1).collect();
    if args.is_empty() {
        print_usage();
        return ExitCode::from(2);
    }

    let mut cmd = Command::new(env::var_os("CARGO").unwrap_or_else(|| "cargo".into()));
    cmd.args(&args);

    match detect_mujoco() {
        Some(mj) => {
            eprintln!("xtask: MuJoCo detected");
            eprintln!("xtask:   root    = {}", mj.root.display());
            eprintln!("xtask:   lib_dir = {}", mj.lib_dir.display());
            apply_mujoco_env(&mut cmd, &mj);
        }
        None => {
            eprintln!("xtask: MuJoCo not detected; forwarding to cargo without MuJoCo env vars");
        }
    }

    match cmd.status() {
        Ok(s) if s.success() => ExitCode::SUCCESS,
        Ok(s) => ExitCode::from(s.code().unwrap_or(1).clamp(0, 255) as u8),
        Err(e) => {
            eprintln!("xtask: failed to spawn cargo: {e}");
            ExitCode::FAILURE
        }
    }
}

fn print_usage() {
    eprintln!("usage: cargo xtask <cargo-subcommand> [args...]");
    eprintln!();
    eprintln!("examples:");
    eprintln!("  cargo xtask build --features mujoco");
    eprintln!("  cargo xtask run   --features mujoco --example demo");
    eprintln!("  cargo xtask test  --features mujoco");
}

fn apply_mujoco_env(cmd: &mut Command, mj: &MujocoEnv) {
    cmd.env("MUJOCO_DYNAMIC_LINK_DIR", &mj.lib_dir);

    if cfg!(target_os = "macos") {
        cmd.env("DYLD_FRAMEWORK_PATH", &mj.root);
        cmd.env("DYLD_LIBRARY_PATH", &mj.lib_dir);
    } else if cfg!(target_os = "linux") {
        let prev = env::var("LD_LIBRARY_PATH").unwrap_or_default();
        let new = if prev.is_empty() {
            mj.lib_dir.display().to_string()
        } else {
            format!("{}:{}", mj.lib_dir.display(), prev)
        };
        cmd.env("LD_LIBRARY_PATH", new);
    } else if cfg!(target_os = "windows") {
        cmd.env("MUJOCO_HOME", &mj.root);
        let prev = env::var("PATH").unwrap_or_default();
        let new = if prev.is_empty() {
            mj.lib_dir.display().to_string()
        } else {
            format!("{};{}", mj.lib_dir.display(), prev)
        };
        cmd.env("PATH", new);
    }
}

fn detect_mujoco() -> Option<MujocoEnv> {
    if cfg!(target_os = "macos") {
        detect_macos()
    } else if cfg!(target_os = "linux") {
        detect_linux()
    } else if cfg!(target_os = "windows") {
        detect_windows()
    } else {
        None
    }
}

fn detect_macos() -> Option<MujocoEnv> {
    // 1. Homebrew Cask (Apple silicon): /opt/homebrew/Caskroom/mujoco/<ver>
    // 2. Homebrew Cask (Intel):         /usr/local/Caskroom/mujoco/<ver>
    let caskrooms = [
        Path::new("/opt/homebrew/Caskroom/mujoco"),
        Path::new("/usr/local/Caskroom/mujoco"),
    ];
    for cask in caskrooms {
        if let Some(latest) = latest_subdir(cask) {
            if let Some(env) = framework_under(&latest) {
                return Some(env);
            }
        }
    }

    // 3. ~/mujoco
    if let Some(home) = env::var_os("HOME").map(PathBuf::from) {
        let root = home.join("mujoco");
        if let Some(env) = framework_under(&root).or_else(|| lib_under(&root, "libmujoco.dylib")) {
            return Some(env);
        }
    }

    // 4. /Applications/MuJoCo.app
    let app_root = PathBuf::from("/Applications/MuJoCo.app/Contents/Frameworks");
    if let Some(env) = framework_under(&app_root) {
        return Some(env);
    }

    None
}

fn detect_linux() -> Option<MujocoEnv> {
    let mut candidates: Vec<PathBuf> = Vec::new();
    if let Some(home) = env::var_os("HOME").map(PathBuf::from) {
        // Pick the newest ~/.mujoco/mujoco-* if present, otherwise fall back to
        // the pinned 3.8.0 path that the legacy shell script used.
        if let Some(latest) = latest_matching_subdir(&home.join(".mujoco"), "mujoco-") {
            candidates.push(latest);
        }
        candidates.push(home.join(".mujoco/mujoco-3.8.0"));
    }
    candidates.push(PathBuf::from("/opt/mujoco"));

    for root in candidates {
        if !root.is_dir() {
            continue;
        }
        let lib = root.join("lib");
        if lib.join("libmujoco.so").exists()
            || lib.join("libmujoco.so.3").exists()
            || has_versioned_so(&lib, "libmujoco.so.")
        {
            return Some(MujocoEnv { lib_dir: lib, root });
        }
    }
    None
}

fn detect_windows() -> Option<MujocoEnv> {
    let mut candidates = vec![
        PathBuf::from(r"C:\mujoco"),
        PathBuf::from(r"C:\Program Files\mujoco"),
        PathBuf::from(r"C:\Program Files (x86)\mujoco"),
    ];
    if let Some(home) = env::var_os("USERPROFILE").map(PathBuf::from) {
        candidates.push(home.join("mujoco"));
        candidates.push(home.join(r".mujoco\mujoco-3.8.0"));
        candidates.push(home.join(r"scoop\apps\mujoco\current"));
    }
    for root in candidates {
        if !root.is_dir() {
            continue;
        }
        for sub in ["lib", "bin"] {
            let dir = root.join(sub);
            if dir.join("mujoco.dll").exists() {
                return Some(MujocoEnv {
                    lib_dir: dir,
                    root,
                });
            }
        }
    }
    None
}

fn framework_under(root: &Path) -> Option<MujocoEnv> {
    let lib_dir = root.join("mujoco.framework/Versions/A");
    if lib_dir.join("libmujoco.dylib").exists() {
        Some(MujocoEnv {
            lib_dir,
            root: root.to_path_buf(),
        })
    } else {
        None
    }
}

fn lib_under(root: &Path, libname: &str) -> Option<MujocoEnv> {
    let lib_dir = root.join("lib");
    if lib_dir.join(libname).exists() {
        Some(MujocoEnv {
            lib_dir,
            root: root.to_path_buf(),
        })
    } else {
        None
    }
}

fn latest_subdir(parent: &Path) -> Option<PathBuf> {
    let entries = std::fs::read_dir(parent).ok()?;
    let mut dirs: Vec<PathBuf> = entries
        .flatten()
        .map(|e| e.path())
        .filter(|p| p.is_dir())
        .collect();
    dirs.sort();
    dirs.pop()
}

fn latest_matching_subdir(parent: &Path, prefix: &str) -> Option<PathBuf> {
    let entries = std::fs::read_dir(parent).ok()?;
    let mut dirs: Vec<PathBuf> = entries
        .flatten()
        .map(|e| e.path())
        .filter(|p| {
            p.is_dir()
                && p.file_name()
                    .and_then(|n| n.to_str())
                    .map_or(false, |n| n.starts_with(prefix))
        })
        .collect();
    dirs.sort();
    dirs.pop()
}

fn has_versioned_so(dir: &Path, prefix: &str) -> bool {
    let Ok(entries) = std::fs::read_dir(dir) else {
        return false;
    };
    entries.flatten().any(|e| {
        e.file_name()
            .to_str()
            .is_some_and(|n| n.starts_with(prefix))
    })
}
