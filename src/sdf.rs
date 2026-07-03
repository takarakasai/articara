//! SDF (Simulation Description Format) import and export — articara
//! boundary layer.
//!
//! Parsing and emission live in `misarta_formats::sdf` (A5, see
//! `doc/refactor_20260702.md` §4.7); this layer converts
//! [`crate::robot::RobotModel`] ⇄ [`misarta::native::MisaFile`] at the
//! boundary and applies the mesh-path policy
//! ([`crate::mesh_paths::MeshPathStyle`]), which needs the model's
//! on-disk layout. Mesh `<uri>` references arrive from the importer
//! already normalised to SDF-dir-relative paths, so the standard
//! `.misa` load path resolves them.

use std::path::{Path, PathBuf};

use crate::robot::*;

// ========== Import ==========

/// Parse an SDF file and return a RobotModel.
pub fn import_sdf(path: &Path) -> Result<RobotModel, String> {
    let out = misarta_formats::sdf::import(path)?;
    for w in &out.warnings {
        log::warn!("SDF import {path:?}: {w}");
    }
    RobotModel::from_misa_file(&out.file, path)
}

// ========== Export ==========

/// Export a RobotModel to SDF XML string with [`MeshPathStyle::Preserve`]
/// (URI verbatim from the `RobotModel`).
///
/// [`MeshPathStyle::Preserve`]: crate::mesh_paths::MeshPathStyle::Preserve
pub fn export_sdf(model: &RobotModel) -> String {
    export_sdf_with_style(model, &crate::mesh_paths::MeshPathStyle::Preserve)
}

/// Export a RobotModel to SDF XML, applying `mesh_path_style` to every
/// `<uri>` emission.
pub fn export_sdf_with_style(
    model: &RobotModel,
    mesh_path_style: &crate::mesh_paths::MeshPathStyle,
) -> String {
    let mut file = match model.to_misa() {
        Ok(f) => f,
        Err(e) => {
            log::error!("SDF export: cannot build MisaFile: {e}");
            return String::new();
        }
    };
    crate::mesh_paths::rewrite_mesh_refs(&mut file, model, mesh_path_style);
    misarta_formats::sdf::export(&file)
}

/// Export SDF to a file with `meshes/<basename>` relative paths and
/// copy referenced mesh files into `<output_dir>/meshes/`.
///
/// The result is a self-contained directory the user can ship — same
/// shape produced by [`crate::mjcf::export_mjcf_to_file`] and
/// [`crate::robot::RobotModel::export_urdf_to_file`].
///
/// Source-format-agnostic: works with `.misa` source (where
/// `GeomData::Mesh.filename` is a plain relative path like
/// `meshes/trunk.stl`) just as well as URDF source (`package://...`).
/// Both flavours run through [`crate::mesh_paths::resolve_source`].
pub fn export_sdf_to_file(model: &RobotModel, output_path: &Path) -> Result<(), String> {
    let output_dir = output_path
        .parent()
        .map(|p| p.to_path_buf())
        .unwrap_or_else(|| PathBuf::from("."));

    let style = crate::mesh_paths::MeshPathStyle::RelativeToDir(output_dir.clone());
    let xml = export_sdf_with_style(model, &style);
    std::fs::write(output_path, &xml).map_err(|e| format!("Write SDF: {e}"))?;

    let copy_count = crate::mesh_paths::copy_meshes_to(model, &output_dir)?;
    log::info!(
        "Exported SDF to {:?}, copied {} mesh file(s)",
        output_path,
        copy_count
    );
    Ok(())
}
