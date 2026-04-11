//! Robot description format detection and dispatching.

use std::path::Path;

/// Supported robot description formats.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RobotFormat {
    Urdf,
    Sdf,
    Mjcf,
    IsaacUsd,
}

impl RobotFormat {
    /// All supported formats for UI listing.
    pub const ALL: &[RobotFormat] = &[
        RobotFormat::Urdf,
        RobotFormat::Sdf,
        RobotFormat::Mjcf,
        RobotFormat::IsaacUsd,
    ];

    /// Whether this format supports import (loading).
    pub fn supports_import(self) -> bool {
        matches!(self, RobotFormat::Urdf | RobotFormat::Sdf | RobotFormat::Mjcf)
    }

    /// Whether this format supports export.
    pub fn supports_export(self) -> bool {
        true // all formats can be exported
    }

    /// Display name.
    pub fn label(self) -> &'static str {
        match self {
            RobotFormat::Urdf => "URDF",
            RobotFormat::Sdf => "SDF",
            RobotFormat::Mjcf => "MJCF",
            RobotFormat::IsaacUsd => "Isaac (USD Python)",
        }
    }

    /// File extension.
    pub fn extension(self) -> &'static str {
        match self {
            RobotFormat::Urdf => "urdf",
            RobotFormat::Sdf => "sdf",
            RobotFormat::Mjcf => "xml",
            RobotFormat::IsaacUsd => "py",
        }
    }

    /// Detect format from file extension.
    pub fn detect(path: &Path) -> Option<RobotFormat> {
        let ext = path.extension()?.to_str()?.to_lowercase();
        match ext.as_str() {
            "urdf" | "xacro" => Some(RobotFormat::Urdf),
            "sdf" | "world" => Some(RobotFormat::Sdf),
            "xml" | "mjcf" => {
                // Peek at contents to distinguish MJCF from other XML
                if let Ok(content) = std::fs::read_to_string(path) {
                    if content.contains("<mujoco") {
                        return Some(RobotFormat::Mjcf);
                    }
                    if content.contains("<sdf") {
                        return Some(RobotFormat::Sdf);
                    }
                    if content.contains("<robot") {
                        return Some(RobotFormat::Urdf);
                    }
                }
                Some(RobotFormat::Mjcf) // default for .xml
            }
            _ => None,
        }
    }
}

impl std::fmt::Display for RobotFormat {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.label())
    }
}
