//! Robot description format detection and dispatching.

use std::path::Path;

/// Supported robot description formats.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RobotFormat {
    Urdf,
    Sdf,
    Mjcf,
    IsaacUsd,
    /// articara/misarta native master format (TOML; `.misa`).
    Misa,
}

impl RobotFormat {
    /// All supported formats for UI listing.
    pub const ALL: &[RobotFormat] = &[
        RobotFormat::Misa,
        RobotFormat::Urdf,
        RobotFormat::Sdf,
        RobotFormat::Mjcf,
        RobotFormat::IsaacUsd,
    ];

    /// Whether this format supports import (loading).
    #[allow(dead_code)]
    pub fn supports_import(self) -> bool {
        matches!(
            self,
            RobotFormat::Urdf
                | RobotFormat::Sdf
                | RobotFormat::Mjcf
                | RobotFormat::IsaacUsd
                | RobotFormat::Misa
        )
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
            RobotFormat::IsaacUsd => "Isaac (USD ASCII)",
            RobotFormat::Misa => "Misa (native)",
        }
    }

    /// File extension.
    #[allow(dead_code)]
    pub fn extension(self) -> &'static str {
        match self {
            RobotFormat::Urdf => "urdf",
            RobotFormat::Sdf => "sdf",
            RobotFormat::Mjcf => "xml",
            RobotFormat::IsaacUsd => "usda",
            RobotFormat::Misa => "misa",
        }
    }

    /// Detect format from file extension only (no file I/O).
    /// For `.xml` files, defaults to MJCF. Use `detect()` for content-aware detection.
    #[allow(dead_code)]
    pub fn detect_from_extension(path: &Path) -> Option<RobotFormat> {
        let ext = path.extension()?.to_str()?.to_lowercase();
        match ext.as_str() {
            "urdf" | "xacro" => Some(RobotFormat::Urdf),
            "sdf" | "world" => Some(RobotFormat::Sdf),
            "xml" | "mjcf" => Some(RobotFormat::Mjcf),
            "usda" | "usd" => Some(RobotFormat::IsaacUsd),
            "misa" => Some(RobotFormat::Misa),
            _ => None,
        }
    }

    /// Detect format from file extension and optionally file contents.
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
            "usda" | "usd" => Some(RobotFormat::IsaacUsd),
            "misa" => Some(RobotFormat::Misa),
            _ => None,
        }
    }
}

impl std::fmt::Display for RobotFormat {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.label())
    }
}

// ─── FormatHandler trait + registry ────────────────────────────────────────

/// One concrete compatibility issue raised when the user is about to
/// export to a format that can't natively express something currently in
/// their model. Surface these from
/// [`analyze_export_compatibility`] before writing the file so the user
/// can confirm the loss (or cancel and re-export to a richer target).
///
/// Severity defines the dialog tone:
/// - `Drop`     — the entity won't appear in the exported file at all.
///   For sidecar-aware workflows this is fine (the `.misarta.toml`
///   preserves the data) but consumers reading only the exported file
///   will lose it.
/// - `Approximate` — the entity is converted to a representable shape
///   (e.g. capsule → cylinder + 2 spheres). Geometry survives but with
///   slightly different physics (rounded ends become flat).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ExportIssue {
    /// Short label for grouping in the dialog (e.g. "Sensors").
    pub feature: String,
    /// How many entities of this kind are affected.
    pub count: usize,
    pub severity: ExportSeverity,
    /// Human-readable explanation rendered in the warning dialog.
    pub message: String,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ExportSeverity {
    Drop,
    Approximate,
}

impl ExportSeverity {
    pub fn label(self) -> &'static str {
        match self {
            ExportSeverity::Drop => "DROP",
            ExportSeverity::Approximate => "APPROX",
        }
    }
}

/// Run a pre-export compatibility check between the loaded model and
/// the target format. Returns an empty vec when the format can express
/// everything the model contains; otherwise one [`ExportIssue`] per
/// feature category.
///
/// The function deliberately does NOT decide whether to proceed — that's
/// up to the caller (typically a confirmation dialog). It's a pure
/// query.
///
/// **Misa target short-circuit**: when the target is the `.misa` master
/// format, this returns an empty vec without running any per-feature
/// checks. `.misa` is articara's canonical representation and round-trips
/// every field of `RobotModel` losslessly — there is by definition
/// nothing to warn about. The caller gets a fast path that matches the
/// "master is the source of truth" mental model.
pub fn analyze_export_compatibility(
    model: &crate::robot::RobotModel,
    target: &dyn FormatHandler,
) -> Vec<ExportIssue> {
    if target.name() == "Misa" {
        // Misa is the master format — by construction it preserves
        // everything in `RobotModel`. Skip the per-feature checks both
        // for correctness (no false-positive warnings) and to make the
        // "no warnings on save-as-master" UX explicit in the code.
        return Vec::new();
    }
    let caps = target.capabilities();
    let mut issues = Vec::new();

    if !caps.mimic && !model.mimics.is_empty() {
        issues.push(ExportIssue {
            feature: "Mimic joints".into(),
            count: model.mimics.len(),
            severity: ExportSeverity::Drop,
            message: format!(
                "{} cannot natively express linear-coupled (mimic) joints. \
                 The relationship is preserved in `.misarta.toml` but \
                 consumers reading only the exported file will see the \
                 joints as independent.",
                target.name(),
            ),
        });
    }

    if !caps.sensors && !model.sensors.is_empty() {
        issues.push(ExportIssue {
            feature: "Sensors".into(),
            count: model.sensors.len(),
            severity: ExportSeverity::Drop,
            message: format!(
                "{} has no native sensor primitives. Sensor entries are \
                 preserved in `.misarta.toml` only.",
                target.name(),
            ),
        });
    }

    if !caps.collision_pairs && !model.collision_pairs.is_empty() {
        issues.push(ExportIssue {
            feature: "Collision pairs".into(),
            count: model.collision_pairs.len(),
            severity: ExportSeverity::Drop,
            message: format!(
                "{} cannot enumerate per-pair collision overrides. The \
                 SRDF-style allow/disallow list is preserved only in \
                 `.misarta.toml`.",
                target.name(),
            ),
        });
    }

    if !caps.closed_loops && !model.loop_closures.is_empty() {
        issues.push(ExportIssue {
            feature: "Closed-loop constraints".into(),
            count: model.loop_closures.len(),
            severity: ExportSeverity::Drop,
            message: format!(
                "{} is tree-structured and cannot express closed kinematic \
                 loops. The constraints are preserved in `.misarta.toml`; \
                 consumers reading only the exported file will see an open \
                 chain.",
                target.name(),
            ),
        });
    }

    // Capsule → URDF approximation. URDF doesn't have a `<capsule>` tag,
    // so the exporter decomposes capsules into a cylinder + two end
    // spheres. Geometry remains visually correct but the physics for
    // contacts changes slightly (flat-ended cylinder vs spherical caps).
    let mut capsule_total = 0;
    for link in &model.links {
        for v in &link.visuals {
            if matches!(v.geometry, crate::robot::GeomData::Capsule { .. }) {
                capsule_total += 1;
            }
        }
        for c in &link.collisions {
            if matches!(c.geometry, crate::robot::GeomData::Capsule { .. }) {
                capsule_total += 1;
            }
        }
    }
    if capsule_total > 0 && target.name() == "URDF" {
        issues.push(ExportIssue {
            feature: "Capsule shapes".into(),
            count: capsule_total,
            severity: ExportSeverity::Approximate,
            message:
                "URDF has no `<capsule>` primitive. Each capsule is \
                 decomposed into a cylinder plus two end spheres on \
                 export — visually correct but slightly different \
                 contact behaviour."
                    .into(),
        });
    }

    issues
}

/// What features a particular [`FormatHandler`] can express on round-trip.
///
/// Used by the host (`articara` UI / scripting) to surface gaps to the user
/// up-front: "USD has no native mimic — exporting will drop your mimic
/// entries". The list is intentionally coarse — fine-grained details are
/// the handler's responsibility.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct FormatCapabilities {
    pub mimic: bool,
    pub sensors: bool,
    pub collision_pairs: bool,
    pub closed_loops: bool,
    pub actuators: bool,
}

/// Plug-in interface for a single robot description format.
///
/// `RobotModel` is the in-memory canonical representation (mirrored by
/// `.misarta.toml` on disk); each format handler converts to / from it.
/// Adding a new format means writing one impl of this trait and
/// registering it on the [`FormatRegistry`] — no surgery to the core
/// loaders is required.
pub trait FormatHandler: Send + Sync {
    /// Display name (e.g. "URDF").
    fn name(&self) -> &str;
    /// File extensions this handler claims (lowercase, without the dot).
    /// First entry is the canonical one used by `export(..)`.
    fn extensions(&self) -> &[&'static str];
    /// Capability matrix.
    fn capabilities(&self) -> FormatCapabilities;
    /// Read a file into a `RobotModel`.
    fn import(&self, path: &Path) -> Result<crate::robot::RobotModel, String>;
    /// Write the model to a file in this format.
    fn export(
        &self,
        model: &crate::robot::RobotModel,
        path: &Path,
    ) -> Result<(), String>;
    /// Should `detect()` use file contents to disambiguate? Override for
    /// formats that share an extension (e.g. `.xml`).
    fn matches_content(&self, _content: &str) -> bool {
        false
    }
}

/// Process-wide registry of [`FormatHandler`]s.
///
/// `default_registry()` returns one populated with the built-in URDF / SDF
/// / MJCF / USD handlers; downstream consumers can `register()` more.
pub struct FormatRegistry {
    handlers: Vec<Box<dyn FormatHandler>>,
}

impl FormatRegistry {
    pub fn new() -> Self {
        Self { handlers: Vec::new() }
    }

    /// Built-in registry covering Misa (native) / URDF / SDF / MJCF / USD.
    pub fn default_registry() -> Self {
        let mut r = Self::new();
        r.register(Box::new(handlers::MisaHandler));
        r.register(Box::new(handlers::UrdfHandler));
        r.register(Box::new(handlers::SdfHandler));
        r.register(Box::new(handlers::MjcfHandler));
        r.register(Box::new(handlers::UsdHandler));
        r
    }

    pub fn register(&mut self, handler: Box<dyn FormatHandler>) {
        self.handlers.push(handler);
    }

    pub fn handlers(&self) -> &[Box<dyn FormatHandler>] {
        &self.handlers
    }

    /// Pick a handler by file path. Uses extension first, then content
    /// sniffing for ambiguous cases (e.g. `.xml` shared by MJCF / SDF / URDF).
    pub fn handler_for(&self, path: &Path) -> Option<&dyn FormatHandler> {
        let ext = path
            .extension()
            .and_then(|e| e.to_str())
            .map(|s| s.to_lowercase())
            .unwrap_or_default();
        // Pass 1: unique extension match.
        let by_ext: Vec<&dyn FormatHandler> = self
            .handlers
            .iter()
            .filter(|h| h.extensions().contains(&ext.as_str()))
            .map(|h| h.as_ref())
            .collect();
        if by_ext.len() == 1 {
            return Some(by_ext[0]);
        }
        if by_ext.is_empty() {
            return None;
        }
        // Pass 2: ambiguous extension — fall back to content sniff.
        if let Ok(content) = std::fs::read_to_string(path) {
            if let Some(h) = by_ext.iter().find(|h| h.matches_content(&content)) {
                return Some(*h);
            }
        }
        // Pass 3: pick the first handler that matches the extension.
        Some(by_ext[0])
    }

    pub fn import(
        &self,
        path: &Path,
    ) -> Result<crate::robot::RobotModel, String> {
        let h = self
            .handler_for(path)
            .ok_or_else(|| format!("No handler for {:?}", path))?;
        h.import(path)
    }

    pub fn export(
        &self,
        model: &crate::robot::RobotModel,
        path: &Path,
    ) -> Result<(), String> {
        let h = self
            .handler_for(path)
            .ok_or_else(|| format!("No handler for {:?}", path))?;
        h.export(model, path)
    }
}

impl Default for FormatRegistry {
    fn default() -> Self {
        Self::default_registry()
    }
}

mod handlers {
    use super::{FormatCapabilities, FormatHandler};
    use std::path::Path;

    /// URDF (.urdf, .xacro). Tree-only; no native sensors / collision pairs.
    pub struct UrdfHandler;
    impl FormatHandler for UrdfHandler {
        fn name(&self) -> &str { "URDF" }
        fn extensions(&self) -> &[&'static str] { &["urdf", "xacro"] }
        fn capabilities(&self) -> FormatCapabilities {
            FormatCapabilities {
                mimic: true,
                sensors: false,
                collision_pairs: false,
                closed_loops: false,
                actuators: false,
            }
        }
        fn matches_content(&self, content: &str) -> bool {
            content.contains("<robot")
        }
        fn import(&self, path: &Path) -> Result<crate::robot::RobotModel, String> {
            crate::robot::RobotModel::from_urdf(path)
        }
        fn export(
            &self,
            model: &crate::robot::RobotModel,
            path: &Path,
        ) -> Result<(), String> {
            model.export_urdf_to_file(path)
        }
    }

    /// SDF (.sdf, .world). Native sensors + bitmask-based collision filter.
    pub struct SdfHandler;
    impl FormatHandler for SdfHandler {
        fn name(&self) -> &str { "SDF" }
        fn extensions(&self) -> &[&'static str] { &["sdf", "world"] }
        fn capabilities(&self) -> FormatCapabilities {
            FormatCapabilities {
                mimic: true,
                sensors: true,
                collision_pairs: true,
                closed_loops: true,
                actuators: true,
            }
        }
        fn matches_content(&self, content: &str) -> bool {
            content.contains("<sdf")
        }
        fn import(&self, path: &Path) -> Result<crate::robot::RobotModel, String> {
            crate::sdf::import_sdf(path)
        }
        fn export(
            &self,
            model: &crate::robot::RobotModel,
            path: &Path,
        ) -> Result<(), String> {
            crate::sdf::export_sdf_to_file(model, path)
        }
    }

    /// MJCF (.xml, .mjcf). Rich sensors + tendons; no native mimic but
    /// `<equality><joint>` provides linear-coupling equivalent.
    pub struct MjcfHandler;
    impl FormatHandler for MjcfHandler {
        fn name(&self) -> &str { "MJCF" }
        fn extensions(&self) -> &[&'static str] { &["xml", "mjcf"] }
        fn capabilities(&self) -> FormatCapabilities {
            FormatCapabilities {
                mimic: true,
                sensors: true,
                collision_pairs: true,
                closed_loops: true,
                actuators: true,
            }
        }
        fn matches_content(&self, content: &str) -> bool {
            content.contains("<mujoco")
        }
        fn import(&self, path: &Path) -> Result<crate::robot::RobotModel, String> {
            crate::mjcf::import_mjcf(path)
        }
        fn export(
            &self,
            model: &crate::robot::RobotModel,
            path: &Path,
        ) -> Result<(), String> {
            let xml = crate::mjcf::export_mjcf(model);
            std::fs::write(path, xml).map_err(|e| format!("Write MJCF: {e}"))
        }
    }

    /// `.misa` — articara/misarta native master format. Designed to
    /// represent every concept articara holds in memory, so it has all
    /// capabilities enabled. This is the canonical format; URDF / MJCF
    /// / USD exist as derivative export targets.
    pub struct MisaHandler;
    impl FormatHandler for MisaHandler {
        fn name(&self) -> &str { "Misa" }
        fn extensions(&self) -> &[&'static str] { &["misa"] }
        fn capabilities(&self) -> FormatCapabilities {
            FormatCapabilities {
                mimic: true,
                sensors: true,
                collision_pairs: true,
                closed_loops: true,
                actuators: true,
            }
        }
        fn import(&self, path: &Path) -> Result<crate::robot::RobotModel, String> {
            crate::robot::RobotModel::from_misa(path)
        }
        fn export(
            &self,
            _model: &crate::robot::RobotModel,
            _path: &Path,
        ) -> Result<(), String> {
            // Export path is not yet implemented; tracked in
            // doc/refactor_20260502.md ToDo (RobotModel::to_misa).
            Err("Misa export (RobotModel::to_misa) is not yet implemented".into())
        }
    }

    /// USD ASCII (.usda, .usd). Joint Drives + filtered pairs; sensors
    /// only via Isaac extensions (not yet wired).
    pub struct UsdHandler;
    impl FormatHandler for UsdHandler {
        fn name(&self) -> &str { "Isaac USD" }
        fn extensions(&self) -> &[&'static str] { &["usda", "usd"] }
        fn capabilities(&self) -> FormatCapabilities {
            FormatCapabilities {
                mimic: false,
                sensors: false,
                collision_pairs: true,
                closed_loops: true,
                actuators: true,
            }
        }
        fn import(&self, path: &Path) -> Result<crate::robot::RobotModel, String> {
            crate::usd_import::import_usda(path)
        }
        fn export(
            &self,
            model: &crate::robot::RobotModel,
            path: &Path,
        ) -> Result<(), String> {
            let dir = path.parent().ok_or("USD export: invalid path")?;
            crate::usd::export_usda_to_dir(model, dir).map(|_| ())
        }
    }
}
