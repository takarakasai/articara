//! USD ASCII (.usda) export for robot models — articara boundary layer.
//!
//! Generation lives in `misarta_formats::usd` (A5, see
//! `doc/refactor_20260702.md` §4.7); this layer converts
//! [`crate::robot::RobotModel`] → [`misarta::native::MisaFile`] and
//! supplies the two pieces of data the schema doesn't carry:
//!
//! - the **posed world transform** per link (the editor's current FK via
//!   [`RobotModel::compute_transforms`]), and
//! - **inline mesh vertices** (USD embeds mesh geometry; the schema only
//!   holds file references).
//!
//! The output is suitable for loading in NVIDIA Isaac Sim / Omniverse.

use std::collections::HashMap;

use nalgebra as na;

use crate::robot::*;
use misarta_formats::usd::{GeomSlot, UsdExportRefs};

/// Export the robot model as a USD ASCII (.usda) string.
pub fn export_usda(model: &RobotModel) -> String {
    // USD has no native mimic/sensor concepts in plain UsdPhysics; warn so
    // the user knows they're being dropped. Isaac Sim provides extensions
    // (e.g. ContactSensor, IMU) that we'd need to wire separately.
    if !model.mimics.is_empty() {
        log::warn!(
            "USD export: dropping {} mimic relationship(s) — USD has no native equivalent (use Isaac Articulation drives manually)",
            model.mimics.len(),
        );
    }
    if !model.sensors.is_empty() {
        log::warn!(
            "USD export: dropping {} sensor(s) — Isaac extension prims aren't yet emitted by articara",
            model.sensors.len(),
        );
    }

    let file = match model.to_misa() {
        Ok(f) => f,
        Err(e) => {
            log::error!("USD export: cannot build MisaFile: {e}");
            return String::new();
        }
    };

    // Posed world transforms from the editor's FK (not the q = 0 rest
    // chain the format layer would otherwise assume).
    let transforms: HashMap<String, na::Isometry3<f64>> = model
        .compute_transforms()
        .into_iter()
        .map(|(name, iso)| (name, iso.cast::<f64>()))
        .collect();
    let tf_fn = |link: &str| -> Option<na::Isometry3<f64>> { transforms.get(link).copied() };

    // Loaded mesh vertices per geom occurrence.
    let mesh_fn = |link: &str, slot: GeomSlot| -> Option<Vec<f32>> {
        let li = *model.link_map.get(link)?;
        let l = model.links.get(li)?;
        let geometry = match slot {
            GeomSlot::Visual(i) => &l.visuals.get(i)?.geometry,
            GeomSlot::Collision(i) => &l.collisions.get(i)?.geometry,
        };
        match geometry {
            GeomData::Mesh { mesh, .. } => Some(mesh.to_flat_vertices_f32()),
            _ => None,
        }
    };

    misarta_formats::usd::export(
        &file,
        &UsdExportRefs {
            link_world_tf: Some(&tf_fn),
            mesh_vertices: Some(&mesh_fn),
        },
    )
}

/// Export a USDA file to the given directory.
///
/// Writes `{model.name}.usda` inside `output_dir`.
pub fn export_usda_to_dir(
    model: &RobotModel,
    output_dir: &std::path::Path,
) -> Result<std::path::PathBuf, String> {
    std::fs::create_dir_all(output_dir).map_err(|e| format!("Create dir: {e}"))?;

    let filename = format!("{}.usda", sanitize_name(&model.name));
    let path = output_dir.join(&filename);
    let usda = export_usda(model);
    std::fs::write(&path, &usda).map_err(|e| format!("Write USDA: {e}"))?;

    log::info!("USD ASCII export: {:?}", path);
    Ok(path)
}

/// Sanitise a name for use as a USD prim-path component / file stem.
/// USD prim names must match `[a-zA-Z_][a-zA-Z0-9_]*`.
fn sanitize_name(name: &str) -> String {
    let mut out = String::with_capacity(name.len());
    for (i, c) in name.chars().enumerate() {
        if c.is_ascii_alphanumeric() || c == '_' {
            if i == 0 && c.is_ascii_digit() {
                out.push('_');
            }
            out.push(c);
        } else {
            out.push('_');
        }
    }
    if out.is_empty() {
        out.push_str("prim");
    }
    out
}

// =========================================================================
//  Tests
// =========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sanitize_various_names() {
        assert_eq!(sanitize_name("base_link"), "base_link");
        assert_eq!(sanitize_name("link-1"), "link_1");
        assert_eq!(sanitize_name("123abc"), "_123abc");
        assert_eq!(sanitize_name(""), "prim");
        assert_eq!(sanitize_name("my link!"), "my_link_");
    }

    #[test]
    fn export_empty_model() {
        let model = RobotModel::new_empty("test_robot");
        let usda = export_usda(&model);
        assert!(usda.starts_with("#usda 1.0"));
        assert!(usda.contains("defaultPrim = \"World\""));
        assert!(usda.contains("upAxis = \"Z\""));
        assert!(usda.contains("def Xform \"test_robot\""));
        assert!(usda.contains("PhysicsArticulationRootAPI"));
        assert!(usda.contains("def Xform \"base_link\""));
        assert!(usda.contains("PhysicsRigidBodyAPI"));
        assert!(usda.contains("PhysicsMassAPI"));
        assert!(usda.contains("def Cube \"visual_0\""));
        assert!(usda.contains("double size = 2.0"));
        assert!(usda.contains("def Material \"material_0\""));
        assert!(usda.contains("UsdPreviewSurface"));
    }
}
