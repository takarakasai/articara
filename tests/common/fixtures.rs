//! Shared fixture paths for the format / editor regression suites.

use std::path::PathBuf;

/// Return the absolute path of the `tests/fixtures` directory.
pub fn fixtures_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests").join("fixtures")
}

/// Return the fixture URDF path.
pub fn fixture_urdf() -> PathBuf {
    fixtures_dir().join("urdf").join("test_robot.urdf")
}

/// Return the fixture SDF path.
pub fn fixture_sdf() -> PathBuf {
    fixtures_dir().join("sdf").join("test_robot.sdf")
}

/// Return the fixture MJCF path.
pub fn fixture_mjcf() -> PathBuf {
    fixtures_dir().join("mjcf").join("test_robot.xml")
}

/// Return the real namiashi URDF path (for full integration tests).
pub fn namiashi_urdf() -> PathBuf {
    fixtures_dir().join("namiashi").join("urdf").join("namiashi.urdf")
}
