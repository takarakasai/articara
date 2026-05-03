//! Shared mesh-path helpers for the URDF / SDF / MJCF exporters.
//!
//! Different export targets need different mesh path conventions:
//!
//! - In-process loading (e.g. MuJoCo via `MjModel::from_xml_string`):
//!   needs **absolute** paths because there's no on-disk anchor.
//! - File-on-disk export to be shared / archived: needs paths
//!   **relative to the output directory**, with the mesh files copied
//!   alongside, so the export is self-contained.
//! - Backwards compatibility with URDF-centric workflows: needs to
//!   **preserve** the original URI (`package://...`) untouched.
//!
//! [`MeshPathStyle`] selects between these; [`emit_path`] applies the
//! style to a single URI and [`copy_meshes_to`] performs the
//! associated file copy when the style is [`MeshPathStyle::RelativeToDir`].
//!
//! Source-side resolution ([`resolve_source`]) handles every URI flavour
//! the loaders produce — `package://name/sub/foo.stl` (URDF / SDF),
//! `file:///abs/path` (rare), plain relative `meshes/foo.stl` (.misa
//! convention), and bare absolute paths — so callers don't have to
//! special-case the source format.

use std::collections::HashSet;
use std::path::{Path, PathBuf};

use crate::robot::{GeomData, RobotModel};

/// How a mesh reference should appear in the exported model file.
#[derive(Debug, Clone)]
pub enum MeshPathStyle {
    /// Emit absolute filesystem paths. Use for in-process loading
    /// (the file is consumed immediately and never moved). Not portable
    /// across machines.
    Absolute,
    /// Emit `meshes/<basename>` relative to the supplied directory.
    /// The caller must also copy the source meshes via
    /// [`copy_meshes_to`] so the result is self-contained.
    RelativeToDir(PathBuf),
    /// Emit the URI exactly as it appears in [`GeomData::Mesh::filename`].
    /// Backwards-compatible with URDF / SDF tooling that expects to
    /// resolve `package://` references on its own.
    Preserve,
}

impl Default for MeshPathStyle {
    fn default() -> Self {
        Self::Absolute
    }
}

/// Resolve a mesh URI to its absolute on-disk source location, taking
/// the source model's layout into account.
///
/// Handles every URI flavour articara's loaders produce:
/// - `package://<pkg>/sub/foo.stl` → resolved against the URDF package
///   root (grandparent of `model.source_path`).
/// - `file:///abs/path` → strip prefix.
/// - Already-absolute path → returned as-is.
/// - Plain relative `meshes/foo.stl` (the `.misa` convention) → joined
///   with `model.source_path`'s parent.
///
/// Returns `None` only if `model.source_path` is unset and the URI is
/// not already absolute (genuinely unresolvable).
pub fn resolve_source(uri: &str, model: &RobotModel) -> Option<PathBuf> {
    if uri.starts_with("package://") || uri.starts_with("file://") {
        // URDF / SDF style. `resolve_package_path` only needs a package
        // root (grandparent of the URDF) — the closure-style code in
        // mjcf.rs / sdf.rs / robot.rs all derive it the same way.
        let pkg = model
            .source_path
            .as_ref()
            .and_then(|p| p.parent())
            .and_then(|d| d.parent());
        let resolved = pkg
            .map(|p| crate::robot::resolve_package_path(uri, p))
            .unwrap_or_else(|| PathBuf::from(uri));
        return Some(absolute_path(&resolved));
    }

    let p = Path::new(uri);
    if p.is_absolute() {
        return Some(p.to_path_buf());
    }

    // Plain relative URI — the `.misa` convention is "relative to the
    // model file itself".
    model
        .source_path
        .as_ref()
        .and_then(|p| p.parent())
        .map(|d| absolute_path(&d.join(uri)))
}

/// Apply [`MeshPathStyle`] to a mesh URI and return the string that
/// should appear in the exported file.
///
/// `Preserve` returns the URI unchanged; `Absolute` runs
/// [`resolve_source`] and falls back to the raw URI when resolution
/// fails; `RelativeToDir` returns `"meshes/<basename>"` regardless of
/// the source layout (the basename stays stable across every export
/// target).
///
/// Note: `RelativeToDir` produces only the *path string* — the actual
/// file copy happens via [`copy_meshes_to`].
pub fn emit_path(uri: &str, model: &RobotModel, style: &MeshPathStyle) -> String {
    match style {
        MeshPathStyle::Preserve => uri.to_string(),
        MeshPathStyle::Absolute => resolve_source(uri, model)
            .map(|p| p.to_string_lossy().into_owned())
            .unwrap_or_else(|| uri.to_string()),
        MeshPathStyle::RelativeToDir(_) => {
            let basename = Path::new(uri)
                .file_name()
                .map(|n| n.to_string_lossy().into_owned())
                .unwrap_or_else(|| "mesh.stl".into());
            format!("meshes/{basename}")
        }
    }
}

/// Copy every mesh referenced by `model` into `<dest_dir>/meshes/`,
/// returning the number of files actually copied.
///
/// Pairs with [`MeshPathStyle::RelativeToDir`] — emit the same target
/// path style in the model file and call this helper to materialise the
/// referenced files. Mesh files that resolve to themselves (already in
/// place) are skipped silently.
///
/// Missing source meshes log a warning and are skipped — the export
/// continues so a single broken reference doesn't abort the whole save.
pub fn copy_meshes_to(model: &RobotModel, dest_dir: &Path) -> Result<usize, String> {
    let meshes_dir = dest_dir.join("meshes");
    std::fs::create_dir_all(&meshes_dir)
        .map_err(|e| format!("create meshes dir {:?}: {e}", meshes_dir))?;

    let mut copied: HashSet<PathBuf> = HashSet::new();
    let mut count = 0usize;

    for link in &model.links {
        let geoms = link
            .visuals
            .iter()
            .map(|v| &v.geometry)
            .chain(link.collisions.iter().map(|c| &c.geometry));
        for geom in geoms {
            let GeomData::Mesh { filename: Some(uri), .. } = geom else {
                continue;
            };
            let Some(src_abs) = resolve_source(uri, model) else {
                log::warn!("copy_meshes_to: cannot resolve mesh URI {uri:?}");
                continue;
            };
            if !copied.insert(src_abs.clone()) {
                continue;
            }
            if !src_abs.exists() {
                log::warn!(
                    "copy_meshes_to: mesh source not found, skipping: {:?}",
                    src_abs
                );
                continue;
            }
            let basename = Path::new(uri)
                .file_name()
                .ok_or_else(|| format!("mesh URI has no basename: {uri}"))?;
            let dst = meshes_dir.join(basename);
            if src_abs != dst {
                std::fs::copy(&src_abs, &dst).map_err(|e| {
                    format!("copy mesh {:?} → {:?}: {e}", src_abs, dst)
                })?;
                count += 1;
            }
        }
    }

    Ok(count)
}

/// Make a path absolute without requiring the target to exist
/// (`canonicalize` is too strict for procedural-mesh placeholders or
/// still-broken references). Relative paths are joined with the current
/// working directory.
pub fn absolute_path(p: &Path) -> PathBuf {
    if p.is_absolute() {
        return p.to_path_buf();
    }
    std::env::current_dir()
        .map(|cwd| cwd.join(p))
        .unwrap_or_else(|_| p.to_path_buf())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn emit_relative_uses_basename() {
        let style = MeshPathStyle::RelativeToDir(PathBuf::from("/tmp/out"));
        // dummy model — emit_path doesn't use it for RelativeToDir.
        let model = RobotModel {
            name: "x".into(),
            links: vec![],
            joints: vec![],
            link_map: Default::default(),
            joint_map: Default::default(),
            root_link: "".into(),
            children_joints: Default::default(),
            materials: Default::default(),
            joint_positions: vec![],
            source_path: None,
            base_transform: nalgebra::Isometry3::identity(),
            misarta_cache: None,
            loop_closures: vec![],
            poses: vec![],
            collision_pairs: vec![],
            sequences: vec![],
            mimics: vec![],
            sensors: vec![],
            gaits: vec![],
        };
        assert_eq!(
            emit_path("package://my_pkg/meshes/trunk.stl", &model, &style),
            "meshes/trunk.stl"
        );
        assert_eq!(emit_path("meshes/foo.stl", &model, &style), "meshes/foo.stl");
        assert_eq!(emit_path("/abs/bar.stl", &model, &style), "meshes/bar.stl");
    }

    #[test]
    fn emit_preserve_returns_input_unchanged() {
        let model = RobotModel {
            name: "x".into(), links: vec![], joints: vec![], link_map: Default::default(),
            joint_map: Default::default(), root_link: "".into(),
            children_joints: Default::default(), materials: Default::default(),
            joint_positions: vec![], source_path: None,
            base_transform: nalgebra::Isometry3::identity(), misarta_cache: None,
            loop_closures: vec![], poses: vec![], collision_pairs: vec![],
            sequences: vec![], mimics: vec![], sensors: vec![], gaits: vec![],
        };
        let inputs = ["package://a/b/c.stl", "/abs.stl", "rel/foo.stl", "file:///x.stl"];
        for u in inputs {
            assert_eq!(emit_path(u, &model, &MeshPathStyle::Preserve), u);
        }
    }
}
