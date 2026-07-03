//! USD ASCII (.usda) import for robot models — articara boundary layer.
//!
//! Parsing lives in `misarta_formats::usd` (A5); this layer feeds the
//! resulting [`misarta::native::MisaFile`] through the standard `.misa`
//! load path and re-attaches inline mesh payloads (USD embeds mesh
//! geometry in the file; the schema only carries references, so the
//! importer returns them side-by-side).
//!
//! See [`crate::usd`] for the matching exporter.

use std::path::Path;
use std::sync::Arc;

use crate::robot::*;
use misarta_formats::usd::GeomSlot;

/// Import a `.usda` file and return a `RobotModel`.
pub fn import_usda(path: &Path) -> Result<RobotModel, String> {
    let text = std::fs::read_to_string(path).map_err(|e| format!("Read USDA: {e}"))?;
    import_usda_from_str(&text, Some(path))
}

/// Import from a USDA string. `source_path` is optional metadata.
pub fn import_usda_from_str(
    text: &str,
    source_path: Option<&Path>,
) -> Result<RobotModel, String> {
    let out = misarta_formats::usd::import_str(text)?;
    for w in &out.warnings {
        log::warn!("USD import: {w}");
    }

    // Inline meshes have a sentinel `file` the mesh loader can't open.
    // Swap them for a tiny placeholder primitive before the `.misa`
    // build (avoiding spurious load warnings), then attach the actual
    // vertex data below.
    let mut file = out.file;
    for im in &out.inline_meshes {
        if let Some(geom) = misa_geom_mut(&mut file, &im.link, im.slot) {
            *geom = misarta::native::Geom::Sphere { radius: 1e-6 };
        }
    }

    let base = source_path.unwrap_or_else(|| Path::new(""));
    let mut model = RobotModel::from_misa_file(&file, base)?;
    model.source_path = source_path.map(|p| p.to_path_buf());

    for im in out.inline_meshes {
        let Some(&li) = model.link_map.get(&im.link) else {
            continue;
        };
        let link = &mut model.links[li];
        let geometry = match im.slot {
            GeomSlot::Visual(i) => link.visuals.get_mut(i).map(|v| &mut v.geometry),
            GeomSlot::Collision(i) => link.collisions.get_mut(i).map(|c| &mut c.geometry),
        };
        if let Some(geometry) = geometry {
            *geometry = GeomData::Mesh {
                mesh: Arc::new(im.mesh),
                filename: None,
                scale: None,
            };
        }
    }
    model.rebuild_misarta_model();
    Ok(model)
}

/// Mutable access to the schema geom at (link, slot).
fn misa_geom_mut<'a>(
    file: &'a mut misarta::native::MisaFile,
    link: &str,
    slot: GeomSlot,
) -> Option<&'a mut misarta::native::Geom> {
    let l = file.link.iter_mut().find(|l| l.name == link)?;
    match slot {
        GeomSlot::Visual(i) => l.visual.get_mut(i).map(|v| &mut v.geom),
        GeomSlot::Collision(i) => l.collision.get_mut(i).map(|c| &mut c.geom),
    }
}

// =========================================================================
//  Tests
// =========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra as na;

    #[test]
    fn roundtrip_empty_model() {
        let model = RobotModel::new_empty("test_robot");
        let usda = crate::usd::export_usda(&model);
        let imported = import_usda_from_str(&usda, None).unwrap();

        assert_eq!(imported.name, "test_robot");
        assert_eq!(imported.links.len(), 1);
        assert_eq!(imported.links[0].name, "base_link");
        assert_eq!(imported.joints.len(), 0);
        assert_eq!(imported.root_link, "base_link");

        // Visual
        assert_eq!(imported.links[0].visuals.len(), 1);
        match &imported.links[0].visuals[0].geometry {
            GeomData::Box { hx, hy, hz } => {
                assert!((hx - 0.05).abs() < 1e-3);
                assert!((hy - 0.05).abs() < 1e-3);
                assert!((hz - 0.025).abs() < 1e-3);
            }
            _ => panic!("Expected Box geometry"),
        }
    }

    #[test]
    fn roundtrip_model_with_joints() {
        let mut model = RobotModel::new_empty("jointbot");
        // Add a child link + revolute joint
        model.add_child(
            "base_link",
            "arm_link",
            "arm_joint",
            "revolute",
            na::Isometry3::from_parts(
                na::Translation3::new(0.0, 0.0, 0.1),
                na::UnitQuaternion::identity(),
            ),
            na::Vector3::z(),
            GeomData::Box { hx: 0.05, hy: 0.05, hz: 0.05 },
            [0.7, 0.7, 0.7, 1.0],
            -1.57,
            1.57,
        ).unwrap();

        let usda = crate::usd::export_usda(&model);
        let imported = import_usda_from_str(&usda, None).unwrap();

        assert_eq!(imported.links.len(), 2);
        assert_eq!(imported.joints.len(), 1);
        assert_eq!(imported.joints[0].name, "arm_joint");
        assert_eq!(imported.joints[0].joint_type, "revolute");
        assert_eq!(imported.joints[0].parent_link, "base_link");
        assert_eq!(imported.joints[0].child_link, "arm_link");
        assert_eq!(imported.root_link, "base_link");

        // Check axis reconstruction (should be ~Z)
        let axis = imported.joints[0].axis.normalize();
        assert!(
            (axis.z.abs() - 1.0).abs() < 0.1,
            "Expected Z axis, got {:?}",
            axis
        );

        // Check limits (exported as degrees, re-imported as radians)
        assert!(
            (imported.joints[0].lower - (-1.57)).abs() < 0.02,
            "lower = {}",
            imported.joints[0].lower
        );
        assert!(
            (imported.joints[0].upper - 1.57).abs() < 0.02,
            "upper = {}",
            imported.joints[0].upper
        );

        // Check origin translation
        let t = imported.joints[0].origin.translation;
        assert!((t.z - 0.1).abs() < 1e-3, "origin z = {}", t.z);
    }

    #[test]
    fn roundtrip_fixed_joint() {
        let mut model = RobotModel::new_empty("fixbot");
        model.add_child(
            "base_link", "sensor_link", "sensor_joint", "fixed",
            na::Isometry3::from_parts(
                na::Translation3::new(0.05, 0.0, 0.02),
                na::UnitQuaternion::from_euler_angles(0.0, 0.0, std::f32::consts::FRAC_PI_4),
            ),
            na::Vector3::z(),
            GeomData::Box { hx: 0.02, hy: 0.02, hz: 0.02 },
            [0.5, 0.5, 0.5, 1.0],
            0.0, 0.0,
        ).unwrap();

        let usda = crate::usd::export_usda(&model);
        let imported = import_usda_from_str(&usda, None).unwrap();

        assert_eq!(imported.joints[0].joint_type, "fixed");
        let t = imported.joints[0].origin.translation;
        assert!((t.x - 0.05).abs() < 1e-3);
        assert!((t.z - 0.02).abs() < 1e-3);
    }

    #[test]
    fn roundtrip_prismatic_joint() {
        let mut model = RobotModel::new_empty("slidebot");
        model.add_child(
            "base_link", "slider", "slide_joint", "prismatic",
            na::Isometry3::identity(),
            na::Vector3::x(),
            GeomData::Box { hx: 0.03, hy: 0.03, hz: 0.03 },
            [0.7, 0.7, 0.7, 1.0],
            -0.5, 0.5,
        ).unwrap();

        let usda = crate::usd::export_usda(&model);
        let imported = import_usda_from_str(&usda, None).unwrap();

        assert_eq!(imported.joints[0].joint_type, "prismatic");
        assert!((imported.joints[0].lower - (-0.5)).abs() < 0.02);
        assert!((imported.joints[0].upper - 0.5).abs() < 0.02);

        // Axis should be ~X
        let axis = imported.joints[0].axis.normalize();
        assert!(
            (axis.x.abs() - 1.0).abs() < 0.1,
            "Expected X axis, got {:?}",
            axis
        );
    }

    #[test]
    fn roundtrip_material_colors() {
        let mut model = RobotModel::new_empty("colorbot");
        model.links[0].visuals[0].color = [1.0, 0.0, 0.0, 0.8];

        let usda = crate::usd::export_usda(&model);
        let imported = import_usda_from_str(&usda, None).unwrap();

        let color = imported.links[0].visuals[0].color;
        assert!((color[0] - 1.0).abs() < 0.01, "r = {}", color[0]);
        assert!((color[1]).abs() < 0.01, "g = {}", color[1]);
        assert!((color[2]).abs() < 0.01, "b = {}", color[2]);
        assert!((color[3] - 0.8).abs() < 0.01, "a = {}", color[3]);
    }

    #[test]
    fn roundtrip_cylinder_geometry() {
        let mut model = RobotModel::new_empty("cylbot");
        model.links[0].visuals[0].geometry = GeomData::Cylinder {
            radius: 0.03,
            half_length: 0.15,
        };

        let usda = crate::usd::export_usda(&model);
        let imported = import_usda_from_str(&usda, None).unwrap();

        match &imported.links[0].visuals[0].geometry {
            GeomData::Cylinder { radius, half_length } => {
                assert!(
                    (radius - 0.03).abs() < 1e-3,
                    "radius = {}",
                    radius
                );
                assert!(
                    (half_length - 0.15).abs() < 1e-3,
                    "half_length = {}",
                    half_length
                );
            }
            _ => panic!("Expected Cylinder geometry"),
        }
    }

    #[test]
    fn roundtrip_sphere_geometry() {
        let mut model = RobotModel::new_empty("sphbot");
        model.links[0].visuals[0].geometry = GeomData::Sphere { radius: 0.08 };

        let usda = crate::usd::export_usda(&model);
        let imported = import_usda_from_str(&usda, None).unwrap();

        match &imported.links[0].visuals[0].geometry {
            GeomData::Sphere { radius } => {
                assert!((radius - 0.08).abs() < 1e-3, "radius = {}", radius);
            }
            _ => panic!("Expected Sphere geometry"),
        }
    }

    #[test]
    fn roundtrip_inline_mesh() {
        let mut model = RobotModel::new_empty("meshbot");
        // One triangle, flat [x, y, z, nx, ny, nz] layout.
        let tri: Vec<f32> = vec![
            0.0, 0.0, 0.0, 0.0, 0.0, 1.0, //
            1.0, 0.0, 0.0, 0.0, 0.0, 1.0, //
            0.0, 1.0, 0.0, 0.0, 0.0, 1.0,
        ];
        model.links[0].visuals[0].geometry = GeomData::Mesh {
            mesh: Arc::new(misarta::mesh::MeshData::from_flat_vertices_f32(&tri)),
            filename: None,
            scale: None,
        };

        let usda = crate::usd::export_usda(&model);
        let imported = import_usda_from_str(&usda, None).unwrap();

        match &imported.links[0].visuals[0].geometry {
            GeomData::Mesh { mesh, filename, .. } => {
                assert_eq!(mesh.num_triangles(), 1);
                assert!(filename.is_none());
            }
            _ => panic!("Expected Mesh geometry"),
        }
    }

    #[test]
    fn roundtrip_inertial() {
        let mut model = RobotModel::new_empty("massbot");
        model.links[0].inertial.mass = 2.5;
        model.links[0].inertial.ixx = 0.01;
        model.links[0].inertial.iyy = 0.02;
        model.links[0].inertial.izz = 0.03;
        model.links[0].inertial.origin = na::Isometry3::from_parts(
            na::Translation3::new(0.01, 0.02, 0.03),
            na::UnitQuaternion::identity(),
        );

        let usda = crate::usd::export_usda(&model);
        let imported = import_usda_from_str(&usda, None).unwrap();

        let inertial = &imported.links[0].inertial;
        assert!((inertial.mass - 2.5).abs() < 0.01);
        assert!((inertial.ixx - 0.01).abs() < 1e-3);
        assert!((inertial.iyy - 0.02).abs() < 1e-3);
        assert!((inertial.izz - 0.03).abs() < 1e-3);
        assert!(
            (inertial.origin.translation.x - 0.01).abs() < 1e-3,
        );
    }

    #[test]
    fn roundtrip_multi_link_tree() {
        let mut model = RobotModel::new_empty("treebot");
        model.add_child(
            "base_link", "link_a", "joint_a", "revolute",
            na::Isometry3::identity(), na::Vector3::z(),
            GeomData::Box { hx: 0.05, hy: 0.05, hz: 0.05 },
            [0.7, 0.7, 0.7, 1.0], -1.57, 1.57,
        ).unwrap();
        model.add_child(
            "link_a", "link_b", "joint_b", "revolute",
            na::Isometry3::identity(), na::Vector3::z(),
            GeomData::Box { hx: 0.05, hy: 0.05, hz: 0.05 },
            [0.7, 0.7, 0.7, 1.0], -1.57, 1.57,
        ).unwrap();
        model.add_child(
            "base_link", "link_c", "joint_c", "fixed",
            na::Isometry3::identity(), na::Vector3::z(),
            GeomData::Box { hx: 0.05, hy: 0.05, hz: 0.05 },
            [0.7, 0.7, 0.7, 1.0], 0.0, 0.0,
        ).unwrap();

        let usda = crate::usd::export_usda(&model);
        let imported = import_usda_from_str(&usda, None).unwrap();

        assert_eq!(imported.links.len(), 4);
        assert_eq!(imported.joints.len(), 3);
        assert_eq!(imported.root_link, "base_link");

        // Check tree topology
        let names: Vec<_> = imported.links.iter().map(|l| l.name.as_str()).collect();
        assert!(names.contains(&"base_link"));
        assert!(names.contains(&"link_a"));
        assert!(names.contains(&"link_b"));
        assert!(names.contains(&"link_c"));
    }
}
