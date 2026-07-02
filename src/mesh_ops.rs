//! Shared mesh decomposition / decimation operations.
//!
//! Single implementation of the misarta decompose / decimate dispatch and
//! the conversion from misarta fit results (hulls / spheres / primitives)
//! into articara geometry. Shared by the GUI properties panel and the Rhai
//! scripting engine so the two paths cannot drift apart.

use crate::robot::GeomData;
use nalgebra as na;
use std::sync::Arc;
use std::sync::atomic::AtomicU8;

/// Options for [`decompose_mesh`].
#[derive(Default)]
pub struct DecomposeOptions<'a> {
    /// Cap on the number of hulls (V-HACD) or spheres (sphere tree).
    /// `None` keeps misarta's defaults. Primitive fitting ignores it.
    pub max_count: Option<usize>,
    /// Coarse phase indicator (`misarta::decompose::PHASE_*`), polled by
    /// the GUI while the decomposition runs on a worker thread.
    pub progress: Option<&'a Arc<AtomicU8>>,
    /// Fine-grained 0–100 progress within the current phase.
    pub sub_progress: Option<&'a Arc<AtomicU8>>,
}

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
    use misarta::decompose as dec;
    match method {
        dec::DecompositionMethod::Vhacd => {
            let mut params = dec::VhacdParams::default();
            if let Some(c) = opts.max_count {
                params.max_hulls = c as u32;
            }
            dec::vhacd_with_progress(mesh, &params, opts.progress, opts.sub_progress)
                .into_iter()
                .map(|h| {
                    (
                        base_origin,
                        GeomData::Mesh {
                            mesh: Arc::new(h),
                            filename: None,
                            scale: None,
                        },
                    )
                })
                .collect()
        }
        dec::DecompositionMethod::SphereTree => {
            let mut params = dec::SphereTreeParams::default();
            if let Some(c) = opts.max_count {
                params.max_spheres = c;
            }
            dec::sphere_tree_with_progress(mesh, &params, opts.progress, opts.sub_progress)
                .iter()
                .map(|s| {
                    let t = na::Translation3::new(
                        s.center.x as f32,
                        s.center.y as f32,
                        s.center.z as f32,
                    );
                    (
                        base_origin * na::Isometry3::from_parts(t, na::UnitQuaternion::identity()),
                        GeomData::Sphere {
                            radius: s.radius as f32,
                        },
                    )
                })
                .collect()
        }
        dec::DecompositionMethod::PrimitiveFit => {
            let params = dec::VhacdParams::default();
            dec::primitive_fit_with_progress(mesh, &params, opts.progress, opts.sub_progress)
                .iter()
                .map(|p| primitive_to_part(p, &base_origin))
                .collect()
        }
        dec::DecompositionMethod::PrimitiveFitDirect => {
            let p =
                dec::primitive_fit_direct_with_progress(mesh, opts.progress, opts.sub_progress);
            vec![primitive_to_part(&p, &base_origin)]
        }
    }
}

/// Convert one misarta fit primitive into an articara geometry part.
fn primitive_to_part(
    p: &misarta::decompose::FitPrimitive,
    base_origin: &na::Isometry3<f32>,
) -> (na::Isometry3<f32>, GeomData) {
    let t = na::Translation3::new(p.center.x as f32, p.center.y as f32, p.center.z as f32);
    let r = na::UnitQuaternion::new_normalize(na::Quaternion::new(
        p.rotation.w as f32,
        p.rotation.i as f32,
        p.rotation.j as f32,
        p.rotation.k as f32,
    ));
    let geometry = match p.kind {
        misarta::decompose::PrimitiveKind::Box { hx, hy, hz } => GeomData::Box {
            hx: hx as f32,
            hy: hy as f32,
            hz: hz as f32,
        },
        misarta::decompose::PrimitiveKind::Cylinder {
            radius,
            half_length,
        } => GeomData::Cylinder {
            radius: radius as f32,
            half_length: half_length as f32,
        },
        misarta::decompose::PrimitiveKind::Sphere { radius } => GeomData::Sphere {
            radius: radius as f32,
        },
    };
    (base_origin * na::Isometry3::from_parts(t, r), geometry)
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
