//! Mesh decomposition / decimation — articara-side adapters.
//!
//! The algorithm dispatch lives in [`misarta::decompose::decompose_mesh`]
//! (uniform [`misarta::decompose::FitPart`] results); this module converts
//! those parts into articara [`GeomData`] composed onto the source visual /
//! collision origin, and provides the shared decimate helper. Used by the
//! GUI properties panel and the Rhai scripting engine.

use crate::robot::GeomData;
use nalgebra as na;
use std::sync::Arc;

pub use misarta::decompose::DecomposeOptions;

/// Decompose a mesh into simpler shapes.
///
/// Returns one `(origin, geometry)` pair per produced shape, with `origin`
/// already composed onto `base_origin` (the source visual / collision
/// origin). The caller wraps the pairs into `VisualData` / `CollisionData`.
pub fn decompose_mesh(
    mesh: &misarta::mesh::MeshData,
    base_origin: na::Isometry3<f32>,
    method: misarta::decompose::DecompositionMethod,
    opts: DecomposeOptions<'_>,
) -> Vec<(na::Isometry3<f32>, GeomData)> {
    use misarta::decompose::{FitShape, PrimitiveKind};

    misarta::decompose::decompose_mesh(mesh, method, opts)
        .into_iter()
        .map(|part| {
            let t = na::Translation3::new(
                part.translation.x as f32,
                part.translation.y as f32,
                part.translation.z as f32,
            );
            let r = na::UnitQuaternion::new_normalize(na::Quaternion::new(
                part.rotation.w as f32,
                part.rotation.i as f32,
                part.rotation.j as f32,
                part.rotation.k as f32,
            ));
            let geometry = match part.shape {
                FitShape::Hull(h) => GeomData::Mesh {
                    mesh: Arc::new(h),
                    filename: None,
                    scale: None,
                },
                FitShape::Primitive(PrimitiveKind::Box { hx, hy, hz }) => GeomData::Box {
                    hx: hx as f32,
                    hy: hy as f32,
                    hz: hz as f32,
                },
                FitShape::Primitive(PrimitiveKind::Cylinder {
                    radius,
                    half_length,
                }) => GeomData::Cylinder {
                    radius: radius as f32,
                    half_length: half_length as f32,
                },
                FitShape::Primitive(PrimitiveKind::Sphere { radius }) => GeomData::Sphere {
                    radius: radius as f32,
                },
            };
            (base_origin * na::Isometry3::from_parts(t, r), geometry)
        })
        .collect()
}

/// Decimate a shared mesh in place (replaces the `Arc`).
/// Returns `(triangles_before, triangles_after)`.
pub fn decimate_mesh(
    mesh: &mut Arc<misarta::mesh::MeshData>,
    ratio: f64,
    method: misarta::decimate::DecimationMethod,
) -> (usize, usize) {
    let before = mesh.num_triangles();
    let reduced = mesh.decimate_with(ratio, method);
    let after = reduced.num_triangles();
    *mesh = Arc::new(reduced);
    (before, after)
}
