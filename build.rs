// articara/build.rs
//
// MuJoCo linkage is configured by `mujoco-rs`'s own build.rs, which reads the
// MUJOCO_DYNAMIC_LINK_DIR environment variable. That variable is exported by
// the `xtask` crate (see .cargo/config.toml) before cargo is invoked.
//
// This build script only emits a hint when the variable is missing while the
// `mujoco` feature is on, so a bare `cargo build --features mujoco` produces a
// readable suggestion instead of a raw pkg-config error.

fn main() {
    println!("cargo:rerun-if-env-changed=MUJOCO_DYNAMIC_LINK_DIR");
    println!("cargo:rerun-if-env-changed=MUJOCO_STATIC_LINK_DIR");

    let feature_on = std::env::var_os("CARGO_FEATURE_MUJOCO").is_some();
    let dynamic_set = std::env::var_os("MUJOCO_DYNAMIC_LINK_DIR").is_some();
    let static_set = std::env::var_os("MUJOCO_STATIC_LINK_DIR").is_some();

    if feature_on && !dynamic_set && !static_set {
        println!("cargo:warning=MUJOCO_DYNAMIC_LINK_DIR is not set.");
        println!("cargo:warning=Use `cargo xtask build --features mujoco` for auto-detection,");
        println!("cargo:warning=or `source ./setup-mujoco.sh` (PowerShell: `. .\\setup-mujoco.ps1`) before `cargo build`.");
    }
}
