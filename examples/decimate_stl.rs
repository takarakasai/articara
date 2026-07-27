//! Shrink a display STL with quadric-error decimation.
//!
//! These are visual meshes only -- nothing in the pipeline collides against
//! them (articara's loader strips mesh geoms from `link.collisions`, never
//! from visuals), so the only quality bar is "does it still read as the same
//! part on screen". A torso shell shipped at 24,612 triangles is drawn maybe
//! 250 px tall; the renderer already discards any facet under 1 px^2.
//!
//!     cargo run --release --example decimate_stl -- in.stl out.stl 0.10
//!
//! Writes binary STL (84-byte header + 50 bytes/triangle), which is what the
//! input format already is -- the saving is triangles, not encoding.

use std::io::Write;
use std::path::Path;

use nalgebra as na;

use misarta::decimate::DecimationMethod;
use misarta::mesh::MeshData;

fn write_binary_stl(path: &Path, m: &MeshData) -> std::io::Result<()> {
    let mut f = std::io::BufWriter::new(std::fs::File::create(path)?);
    let mut header = [0u8; 80];
    let tag = b"binary STL, decimated by articara/decimate_stl";
    header[..tag.len()].copy_from_slice(tag);
    f.write_all(&header)?;
    f.write_all(&(m.indices.len() as u32).to_le_bytes())?;
    for (t, tri) in m.indices.iter().enumerate() {
        // Recompute rather than trust face_normals: decimation moves vertices,
        // and a stale normal is what makes a shrunk mesh shade wrong.
        let p: Vec<_> = tri.iter().map(|&i| m.vertices[i as usize]).collect();
        let n = (p[1] - p[0]).cross(&(p[2] - p[0]));
        let n = if n.norm() > 1e-20 {
            n / n.norm()
        } else {
            m.face_normals.get(t).copied().unwrap_or_else(na::Vector3::zeros)
        };
        for v in [n.x, n.y, n.z] {
            f.write_all(&(v as f32).to_le_bytes())?;
        }
        for q in &p {
            for v in [q.x, q.y, q.z] {
                f.write_all(&(v as f32).to_le_bytes())?;
            }
        }
        f.write_all(&0u16.to_le_bytes())?;
    }
    Ok(())
}

/// One-sided Hausdorff distance: the largest distance from an original vertex
/// to the decimated SURFACE.
///
/// Measuring to the nearest surviving VERTEX instead is the obvious shortcut
/// and it is wrong -- deleting the interior vertices of a flat panel changes
/// the surface not at all, but leaves those points far from any remaining
/// vertex. That version reported 14.6 mm of error on a torso the eye cannot
/// tell apart from the original.
fn point_tri(p: &na::Point3<f64>, a: &na::Point3<f64>, b: &na::Point3<f64>, c: &na::Point3<f64>) -> f64 {
    let (ab, ac, ap) = (b - a, c - a, p - a);
    let (d1, d2) = (ab.dot(&ap), ac.dot(&ap));
    if d1 <= 0.0 && d2 <= 0.0 {
        return ap.norm();
    }
    let bp = p - b;
    let (d3, d4) = (ab.dot(&bp), ac.dot(&bp));
    if d3 >= 0.0 && d4 <= d3 {
        return bp.norm();
    }
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        return (ap - ab * (d1 / (d1 - d3))).norm();
    }
    let cp = p - c;
    let (d5, d6) = (ab.dot(&cp), ac.dot(&cp));
    if d6 >= 0.0 && d5 <= d6 {
        return cp.norm();
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        return (ap - ac * (d2 / (d2 - d6))).norm();
    }
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        return (bp - (c - b) * ((d4 - d3) / ((d4 - d3) + (d5 - d6)))).norm();
    }
    let denom = 1.0 / (va + vb + vc);
    (ap - (ab * (vb * denom) + ac * (vc * denom))).norm()
}

/// Returns (p50, p99, max) of the one-sided distance, in metres. The max
/// alone cannot tell "one thin boss collapsed" from "the whole surface
/// moved", and those call for different ratios.
fn deviation(before: &MeshData, after: &MeshData, stride: usize) -> (f64, f64, f64) {
    let tris: Vec<_> = after
        .indices
        .iter()
        .map(|t| {
            (
                after.vertices[t[0] as usize],
                after.vertices[t[1] as usize],
                after.vertices[t[2] as usize],
            )
        })
        .collect();
    let mut d: Vec<f64> = before
        .vertices
        .iter()
        .step_by(stride)
        .map(|p| {
            tris.iter()
                .map(|(a, b, c)| point_tri(p, a, b, c))
                .fold(f64::INFINITY, f64::min)
        })
        .collect();
    d.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let at = |q: f64| d[((d.len() - 1) as f64 * q) as usize];
    (at(0.50), at(0.99), *d.last().unwrap())
}

fn main() {
    let a: Vec<String> = std::env::args().collect();
    if a.len() < 4 {
        eprintln!("usage: decimate_stl <in.stl> <out.stl> <ratio 0..1>");
        std::process::exit(2);
    }
    let (src, dst, ratio) = (Path::new(&a[1]), Path::new(&a[2]), a[3].parse::<f64>().unwrap());

    let before = MeshData::from_stl(src).expect("read stl");
    let after = before.decimate_with(ratio, DecimationMethod::Qem);
    write_binary_stl(dst, &after).expect("write stl");

    let dev = deviation(&before, &after, 4);
    let sz = |p: &Path| std::fs::metadata(p).map(|m| m.len()).unwrap_or(0);
    println!(
        "{:<18} {:6} -> {:6} tris ({:4.1}%)  {:7.1} -> {:6.1} kB  \
dev p50 {:.3}  p99 {:.3}  max {:.3} mm",
        src.file_name().unwrap().to_string_lossy(),
        before.num_triangles(),
        after.num_triangles(),
        100.0 * after.num_triangles() as f64 / before.num_triangles() as f64,
        sz(src) as f64 / 1024.0,
        sz(dst) as f64 / 1024.0,
        1000.0 * dev.0,
        1000.0 * dev.1,
        1000.0 * dev.2,
    );
}
