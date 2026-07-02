//! Ray picking against [`RobotModel`] geometry: viewport click → link
//! selection plus the per-primitive ray intersection tests.

use nalgebra as na;

use crate::rbd::model::*;

impl RobotModel {
    /// Pick: find the closest link hit by a ray, given current world transforms.
    /// Uses two-pass: bounding sphere (coarse) → triangle/analytic intersection (precise).
    /// Returns (link_index, distance) or None.
    pub fn pick_link(
        &self,
        ray_origin: &na::Point3<f32>,
        ray_dir: &na::Vector3<f32>,
        transforms: &std::collections::HashMap<String, na::Isometry3<f32>>,
    ) -> Option<(usize, f32)> {
        let mut best: Option<(usize, f32)> = None;

        for (li, link) in self.links.iter().enumerate() {
            let world_tf = transforms
                .get(&link.name)
                .copied()
                .unwrap_or(na::Isometry3::identity());

            // Coarse pass: bounding sphere
            let (local_center, radius) = self.link_bounding_sphere(li);
            if radius < 1e-6 {
                continue; // Skip links with no visual geometry
            }
            let world_center = world_tf * local_center;
            if ray_sphere_intersect(ray_origin, ray_dir, &world_center, radius).is_none() {
                continue; // Ray misses bounding sphere
            }

            // Precise pass: test against actual geometry of each visual
            let mut link_best_dist: Option<f32> = None;
            for vis in &link.visuals {
                let full_tf = world_tf * vis.origin;
                let dist = precise_geometry_intersect(
                    ray_origin, ray_dir, &full_tf, &vis.geometry,
                );
                if let Some(d) = dist {
                    if d > 0.0 && (link_best_dist.is_none() || d < link_best_dist.unwrap()) {
                        link_best_dist = Some(d);
                    }
                }
            }

            if let Some(d) = link_best_dist {
                if best.is_none() || d < best.unwrap().1 {
                    best = Some((li, d));
                }
            }
        }
        best
    }
}

// ========== Ray Intersection Tests ==========

/// Precise geometry intersection: transforms ray into geometry-local space and tests.
pub fn precise_geometry_intersect(
    ray_origin: &na::Point3<f32>,
    ray_dir: &na::Vector3<f32>,
    geom_tf: &na::Isometry3<f32>,
    geom: &GeomData,
) -> Option<f32> {
    // Transform ray into geometry's local frame
    let inv_tf = geom_tf.inverse();
    let local_origin = inv_tf * ray_origin;
    let local_dir = inv_tf * ray_dir;

    match geom {
        GeomData::Box { hx, hy, hz } => ray_box_intersect(&local_origin, &local_dir, *hx, *hy, *hz),
        GeomData::Cylinder { radius, half_length } => {
            ray_cylinder_intersect(&local_origin, &local_dir, *radius, *half_length)
        }
        GeomData::Sphere { radius } => {
            ray_sphere_intersect(&local_origin, &local_dir, &na::Point3::origin(), *radius)
        }
        GeomData::Capsule { radius, half_length } => {
            // Test cylinder body + two hemisphere caps
            let t_cyl = ray_cylinder_intersect(&local_origin, &local_dir, *radius, *half_length);
            let top_center = na::Point3::new(0.0, 0.0, *half_length);
            let bot_center = na::Point3::new(0.0, 0.0, -*half_length);
            let t_top = ray_sphere_intersect(&local_origin, &local_dir, &top_center, *radius);
            let t_bot = ray_sphere_intersect(&local_origin, &local_dir, &bot_center, *radius);
            [t_cyl, t_top, t_bot].iter().filter_map(|t| *t).reduce(f32::min)
        }
        GeomData::Mesh { mesh, .. } => {
            ray_mesh_intersect(&local_origin, &local_dir, &mesh.to_flat_vertices_f32())
        }
    }
}

/// Ray-sphere intersection. Returns the nearest positive distance or None.
pub fn ray_sphere_intersect(
    origin: &na::Point3<f32>,
    dir: &na::Vector3<f32>,
    center: &na::Point3<f32>,
    radius: f32,
) -> Option<f32> {
    let oc = origin - center;
    let a = dir.dot(dir);
    let b = 2.0 * oc.dot(dir);
    let c = oc.dot(&oc) - radius * radius;
    let disc = b * b - 4.0 * a * c;
    if disc < 0.0 {
        return None;
    }
    let sqrt_disc = disc.sqrt();
    let t1 = (-b - sqrt_disc) / (2.0 * a);
    let t2 = (-b + sqrt_disc) / (2.0 * a);
    if t1 > 0.0 {
        Some(t1)
    } else if t2 > 0.0 {
        Some(t2)
    } else {
        None
    }
}

/// Ray-AABB (box) intersection using slab method.
pub fn ray_box_intersect(
    origin: &na::Point3<f32>,
    dir: &na::Vector3<f32>,
    hx: f32,
    hy: f32,
    hz: f32,
) -> Option<f32> {
    let mut tmin = f32::NEG_INFINITY;
    let mut tmax = f32::INFINITY;
    let halves = [hx, hy, hz];

    for i in 0..3 {
        if dir[i].abs() < 1e-10 {
            if origin[i] < -halves[i] || origin[i] > halves[i] {
                return None;
            }
        } else {
            let inv_d = 1.0 / dir[i];
            let mut t1 = (-halves[i] - origin[i]) * inv_d;
            let mut t2 = (halves[i] - origin[i]) * inv_d;
            if t1 > t2 {
                std::mem::swap(&mut t1, &mut t2);
            }
            tmin = tmin.max(t1);
            tmax = tmax.min(t2);
            if tmin > tmax {
                return None;
            }
        }
    }
    if tmax < 0.0 {
        None
    } else if tmin > 0.0 {
        Some(tmin)
    } else {
        Some(tmax)
    }
}

/// Ray-cylinder intersection (Z-axis aligned, centered at origin).
pub fn ray_cylinder_intersect(
    origin: &na::Point3<f32>,
    dir: &na::Vector3<f32>,
    radius: f32,
    half_length: f32,
) -> Option<f32> {
    // Infinite cylinder in XY
    let a = dir.x * dir.x + dir.y * dir.y;
    let b = 2.0 * (origin.x * dir.x + origin.y * dir.y);
    let c = origin.x * origin.x + origin.y * origin.y - radius * radius;
    let disc = b * b - 4.0 * a * c;

    let mut best: Option<f32> = None;

    if disc >= 0.0 && a > 1e-10 {
        let sqrt_disc = disc.sqrt();
        for &t in &[(-b - sqrt_disc) / (2.0 * a), (-b + sqrt_disc) / (2.0 * a)] {
            if t > 0.0 {
                let z = origin.z + t * dir.z;
                if z.abs() <= half_length {
                    if best.is_none() || t < best.unwrap() {
                        best = Some(t);
                    }
                }
            }
        }
    }

    // Cap discs (top and bottom)
    if dir.z.abs() > 1e-10 {
        for &cap_z in &[half_length, -half_length] {
            let t = (cap_z - origin.z) / dir.z;
            if t > 0.0 {
                let px = origin.x + t * dir.x;
                let py = origin.y + t * dir.y;
                if px * px + py * py <= radius * radius {
                    if best.is_none() || t < best.unwrap() {
                        best = Some(t);
                    }
                }
            }
        }
    }

    best
}

/// Ray-mesh (triangle soup) intersection using Möller–Trumbore algorithm.
/// Vertices are in flat format: [x, y, z, nx, ny, nz, x, y, z, nx, ny, nz, ...].
/// Every 3 vertices (18 floats) form one triangle.
pub fn ray_mesh_intersect(
    origin: &na::Point3<f32>,
    dir: &na::Vector3<f32>,
    vertices: &[f32],
) -> Option<f32> {
    let mut best: Option<f32> = None;
    let stride = 6; // x,y,z,nx,ny,nz per vertex
    let tri_stride = stride * 3; // 18 floats per triangle

    let mut i = 0;
    while i + tri_stride <= vertices.len() {
        let v0 = na::Point3::new(vertices[i], vertices[i + 1], vertices[i + 2]);
        let v1 = na::Point3::new(vertices[i + stride], vertices[i + stride + 1], vertices[i + stride + 2]);
        let v2 = na::Point3::new(vertices[i + stride * 2], vertices[i + stride * 2 + 1], vertices[i + stride * 2 + 2]);

        if let Some(t) = ray_triangle_intersect(origin, dir, &v0, &v1, &v2) {
            if t > 0.0 && (best.is_none() || t < best.unwrap()) {
                best = Some(t);
            }
        }
        i += tri_stride;
    }
    best
}

/// Möller–Trumbore ray-triangle intersection.
pub fn ray_triangle_intersect(
    origin: &na::Point3<f32>,
    dir: &na::Vector3<f32>,
    v0: &na::Point3<f32>,
    v1: &na::Point3<f32>,
    v2: &na::Point3<f32>,
) -> Option<f32> {
    let edge1 = v1 - v0;
    let edge2 = v2 - v0;
    let h = dir.cross(&edge2);
    let a = edge1.dot(&h);
    if a.abs() < 1e-8 {
        return None; // Ray parallel to triangle
    }
    let f = 1.0 / a;
    let s = origin - v0;
    let u = f * s.dot(&h);
    if !(0.0..=1.0).contains(&u) {
        return None;
    }
    let q = s.cross(&edge1);
    let v = f * dir.dot(&q);
    if v < 0.0 || u + v > 1.0 {
        return None;
    }
    let t = f * edge2.dot(&q);
    if t > 1e-6 {
        Some(t)
    } else {
        None
    }
}

/// Find closest approach of a ray to an axis line (infinite line through origin in given direction).
/// Returns `(t_line, distance)` where `t_line` is the parameter along the axis
/// (point on axis = `axis_origin + axis_dir * t_line`) and `distance` is the
/// closest distance between the ray and the axis line.
pub fn ray_axis_closest(
    ro: &na::Point3<f32>,
    rd: &na::Vector3<f32>,
    axis_origin: &na::Point3<f32>,
    axis_dir: &na::Vector3<f32>,
) -> (f32, f32) {
    let w = ro - axis_origin;
    let a = rd.dot(rd);
    let b = rd.dot(axis_dir);
    let c = axis_dir.dot(axis_dir);
    let d = rd.dot(&w);
    let e = axis_dir.dot(&w);
    let denom = a * c - b * b;

    if denom.abs() < 1e-10 {
        // Ray parallel to axis
        let t_line = e / c;
        let closest_on_line = axis_origin + axis_dir * t_line;
        let dist = (ro - closest_on_line).norm();
        (t_line, dist)
    } else {
        let t_ray = (b * e - c * d) / denom;
        let t_line = (a * e - b * d) / denom;
        let p_ray = ro + rd * t_ray;
        let p_line = axis_origin + axis_dir * t_line;
        let dist = (p_ray - p_line).norm();
        (t_line, dist)
    }
}

