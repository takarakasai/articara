/// Generate vertex data for a box. Format: [pos.x, pos.y, pos.z, norm.x, norm.y, norm.z, ...]
pub fn generate_box(hx: f32, hy: f32, hz: f32) -> Vec<f32> {
    let mut v = Vec::with_capacity(216);
    let faces: [([f32; 3], [[f32; 3]; 4]); 6] = [
        (
            [1.0, 0.0, 0.0],
            [
                [hx, -hy, -hz],
                [hx, hy, -hz],
                [hx, hy, hz],
                [hx, -hy, hz],
            ],
        ),
        (
            [-1.0, 0.0, 0.0],
            [
                [-hx, hy, -hz],
                [-hx, -hy, -hz],
                [-hx, -hy, hz],
                [-hx, hy, hz],
            ],
        ),
        (
            [0.0, 1.0, 0.0],
            [
                [-hx, hy, -hz],
                [-hx, hy, hz],
                [hx, hy, hz],
                [hx, hy, -hz],
            ],
        ),
        (
            [0.0, -1.0, 0.0],
            [
                [-hx, -hy, hz],
                [-hx, -hy, -hz],
                [hx, -hy, -hz],
                [hx, -hy, hz],
            ],
        ),
        (
            [0.0, 0.0, 1.0],
            [
                [-hx, -hy, hz],
                [hx, -hy, hz],
                [hx, hy, hz],
                [-hx, hy, hz],
            ],
        ),
        (
            [0.0, 0.0, -1.0],
            [
                [hx, -hy, -hz],
                [-hx, -hy, -hz],
                [-hx, hy, -hz],
                [hx, hy, -hz],
            ],
        ),
    ];
    for (normal, corners) in &faces {
        for &idx in &[0, 1, 2, 0, 2, 3] {
            push_vert(&mut v, corners[idx], *normal);
        }
    }
    v
}

/// Generate vertex data for a cylinder (axis = Z, centered at origin).
pub fn generate_cylinder(radius: f32, half_length: f32, segments: u32) -> Vec<f32> {
    let mut v = Vec::new();
    let seg = segments as f32;
    for i in 0..segments {
        let a0 = (i as f32 / seg) * std::f32::consts::TAU;
        let a1 = ((i + 1) as f32 / seg) * std::f32::consts::TAU;
        let (c0, s0) = (a0.cos(), a0.sin());
        let (c1, s1) = (a1.cos(), a1.sin());

        // Side
        let n0 = [c0, s0, 0.0];
        let n1 = [c1, s1, 0.0];
        let p0 = [radius * c0, radius * s0, half_length];
        let p1 = [radius * c1, radius * s1, half_length];
        let p2 = [radius * c1, radius * s1, -half_length];
        let p3 = [radius * c0, radius * s0, -half_length];
        push_vert(&mut v, p0, n0);
        push_vert(&mut v, p1, n1);
        push_vert(&mut v, p2, n1);
        push_vert(&mut v, p0, n0);
        push_vert(&mut v, p2, n1);
        push_vert(&mut v, p3, n0);

        // Top cap
        push_vert(&mut v, [0.0, 0.0, half_length], [0.0, 0.0, 1.0]);
        push_vert(&mut v, p0, [0.0, 0.0, 1.0]);
        push_vert(&mut v, p1, [0.0, 0.0, 1.0]);

        // Bottom cap
        push_vert(&mut v, [0.0, 0.0, -half_length], [0.0, 0.0, -1.0]);
        push_vert(
            &mut v,
            [radius * c1, radius * s1, -half_length],
            [0.0, 0.0, -1.0],
        );
        push_vert(
            &mut v,
            [radius * c0, radius * s0, -half_length],
            [0.0, 0.0, -1.0],
        );
    }
    v
}

/// Generate vertex data for a UV sphere.
pub fn generate_sphere(radius: f32, slices: u32, stacks: u32) -> Vec<f32> {
    let mut v = Vec::new();
    for i in 0..stacks {
        let phi0 = (i as f32 / stacks as f32) * std::f32::consts::PI;
        let phi1 = ((i + 1) as f32 / stacks as f32) * std::f32::consts::PI;
        for j in 0..slices {
            let th0 = (j as f32 / slices as f32) * std::f32::consts::TAU;
            let th1 = ((j + 1) as f32 / slices as f32) * std::f32::consts::TAU;
            let p00 = sph(radius, phi0, th0);
            let p10 = sph(radius, phi1, th0);
            let p01 = sph(radius, phi0, th1);
            let p11 = sph(radius, phi1, th1);
            let n00 = norm(p00);
            let n10 = norm(p10);
            let n01 = norm(p01);
            let n11 = norm(p11);
            push_vert(&mut v, p00, n00);
            push_vert(&mut v, p10, n10);
            push_vert(&mut v, p11, n11);
            push_vert(&mut v, p00, n00);
            push_vert(&mut v, p11, n11);
            push_vert(&mut v, p01, n01);
        }
    }
    v
}

/// Generate vertex data for a grid on the XY plane.
pub fn generate_grid(size: f32, divisions: u32) -> Vec<f32> {
    let mut v = Vec::new();
    let step = 2.0 * size / divisions as f32;
    let n = [0.0, 0.0, 1.0];
    for i in 0..=divisions {
        let t = -size + i as f32 * step;
        push_vert(&mut v, [t, -size, 0.0], n);
        push_vert(&mut v, [t, size, 0.0], n);
        push_vert(&mut v, [-size, t, 0.0], n);
        push_vert(&mut v, [size, t, 0.0], n);
    }
    v
}

/// Generate axis lines (X=red, Y=green, Z=blue) at origin. Returns (vertices, colors per vertex).
pub fn generate_axes(length: f32) -> Vec<f32> {
    let mut v = Vec::new();
    // X axis
    push_vert(&mut v, [0.0, 0.0, 0.0], [1.0, 0.0, 0.0]);
    push_vert(&mut v, [length, 0.0, 0.0], [1.0, 0.0, 0.0]);
    // Y axis
    push_vert(&mut v, [0.0, 0.0, 0.0], [0.0, 1.0, 0.0]);
    push_vert(&mut v, [0.0, length, 0.0], [0.0, 1.0, 0.0]);
    // Z axis
    push_vert(&mut v, [0.0, 0.0, 0.0], [0.0, 0.0, 1.0]);
    push_vert(&mut v, [0.0, 0.0, length], [0.0, 0.0, 1.0]);
    v
}

/// Generate vertex data for a 3D arrow along +Z axis.
/// Shaft: cylinder from z=0 to z=shaft_length.
/// Head: cone from z=shaft_length to z=shaft_length+head_length.
pub fn generate_arrow(
    shaft_radius: f32,
    shaft_length: f32,
    head_radius: f32,
    head_length: f32,
    segments: u32,
) -> Vec<f32> {
    let mut v = Vec::new();
    let seg = segments as f32;
    let z1 = shaft_length;
    let z2 = shaft_length + head_length;

    for i in 0..segments {
        let a0 = (i as f32 / seg) * std::f32::consts::TAU;
        let a1 = ((i + 1) as f32 / seg) * std::f32::consts::TAU;
        let (c0, s0) = (a0.cos(), a0.sin());
        let (c1, s1) = (a1.cos(), a1.sin());

        // Shaft side
        let n0 = [c0, s0, 0.0];
        let n1 = [c1, s1, 0.0];
        push_vert(&mut v, [shaft_radius * c0, shaft_radius * s0, 0.0], n0);
        push_vert(&mut v, [shaft_radius * c1, shaft_radius * s1, 0.0], n1);
        push_vert(&mut v, [shaft_radius * c1, shaft_radius * s1, z1], n1);
        push_vert(&mut v, [shaft_radius * c0, shaft_radius * s0, 0.0], n0);
        push_vert(&mut v, [shaft_radius * c1, shaft_radius * s1, z1], n1);
        push_vert(&mut v, [shaft_radius * c0, shaft_radius * s0, z1], n0);

        // Shaft bottom cap
        push_vert(&mut v, [0.0, 0.0, 0.0], [0.0, 0.0, -1.0]);
        push_vert(
            &mut v,
            [shaft_radius * c1, shaft_radius * s1, 0.0],
            [0.0, 0.0, -1.0],
        );
        push_vert(
            &mut v,
            [shaft_radius * c0, shaft_radius * s0, 0.0],
            [0.0, 0.0, -1.0],
        );

        // Cone side
        let slope = (head_radius / head_length).atan();
        let nz = slope.sin();
        let nr = slope.cos();
        let cn0 = [nr * c0, nr * s0, nz];
        let cn1 = [nr * c1, nr * s1, nz];
        push_vert(&mut v, [head_radius * c0, head_radius * s0, z1], cn0);
        push_vert(&mut v, [head_radius * c1, head_radius * s1, z1], cn1);
        push_vert(&mut v, [0.0, 0.0, z2], [0.0, 0.0, 1.0]);

        // Cone base cap
        push_vert(&mut v, [0.0, 0.0, z1], [0.0, 0.0, -1.0]);
        push_vert(
            &mut v,
            [head_radius * c1, head_radius * s1, z1],
            [0.0, 0.0, -1.0],
        );
        push_vert(
            &mut v,
            [head_radius * c0, head_radius * s0, z1],
            [0.0, 0.0, -1.0],
        );
    }
    v
}

fn sph(r: f32, phi: f32, theta: f32) -> [f32; 3] {
    [
        r * phi.sin() * theta.cos(),
        r * phi.sin() * theta.sin(),
        r * phi.cos(),
    ]
}

fn norm(v: [f32; 3]) -> [f32; 3] {
    let len = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    if len > 1e-10 {
        [v[0] / len, v[1] / len, v[2] / len]
    } else {
        [0.0, 0.0, 1.0]
    }
}

fn push_vert(buf: &mut Vec<f32>, pos: [f32; 3], normal: [f32; 3]) {
    buf.extend_from_slice(&pos);
    buf.extend_from_slice(&normal);
}

/// Generate a torus (ring) around the Z axis with arrowheads indicating
/// the positive rotation direction (right-hand rule: counterclockwise
/// when viewed from +Z).
///
/// `ring_radius` — distance from the Z axis to the center of the tube.
/// `tube_radius` — radius of the tube cross-section.
/// `ring_segments` — number of segments around the ring.
/// `tube_segments` — number of segments around the tube.
///
/// Returns vertex data in the same [pos, normal] format as other primitives.
pub fn generate_ring(
    ring_radius: f32,
    tube_radius: f32,
    ring_segments: u32,
    tube_segments: u32,
) -> Vec<f32> {
    let mut v = Vec::new();
    let pi2 = std::f32::consts::TAU;

    for i in 0..ring_segments {
        let theta0 = pi2 * i as f32 / ring_segments as f32;
        let theta1 = pi2 * (i + 1) as f32 / ring_segments as f32;

        for j in 0..tube_segments {
            let phi0 = pi2 * j as f32 / tube_segments as f32;
            let phi1 = pi2 * (j + 1) as f32 / tube_segments as f32;

            // Positions on the torus
            let p = |theta: f32, phi: f32| -> ([f32; 3], [f32; 3]) {
                let cx = (ring_radius + tube_radius * phi.cos()) * theta.cos();
                let cy = (ring_radius + tube_radius * phi.cos()) * theta.sin();
                let cz = tube_radius * phi.sin();
                let nx = phi.cos() * theta.cos();
                let ny = phi.cos() * theta.sin();
                let nz = phi.sin();
                ([cx, cy, cz], [nx, ny, nz])
            };

            let (p00, n00) = p(theta0, phi0);
            let (p10, n10) = p(theta1, phi0);
            let (p11, n11) = p(theta1, phi1);
            let (p01, n01) = p(theta0, phi1);

            // Two triangles per quad
            push_vert(&mut v, p00, n00);
            push_vert(&mut v, p10, n10);
            push_vert(&mut v, p11, n11);

            push_vert(&mut v, p00, n00);
            push_vert(&mut v, p11, n11);
            push_vert(&mut v, p01, n01);
        }
    }

    // --- Arrowheads ---
    // Place two arrowheads at 0° and 180° on the ring, pointing in the
    // positive tangent direction (counterclockwise around +Z).
    let arrow_len = ring_radius * 0.28;
    let arrow_r = tube_radius * 2.8;
    let cone_segs: u32 = 12;

    for &theta_a in &[0.0_f32, std::f32::consts::PI] {
        let ct = theta_a.cos();
        let st = theta_a.sin();
        // Center of the ring tube at this angle
        let base = [ring_radius * ct, ring_radius * st, 0.0];
        // Tangent direction (positive rotation = counterclockwise)
        let tang = [-st, ct, 0.0];
        // Tip of the cone
        let tip = [
            base[0] + tang[0] * arrow_len,
            base[1] + tang[1] * arrow_len,
            base[2],
        ];
        // Normal of the tip points along the tangent
        let tip_n = tang;

        // Two vectors perpendicular to the tangent for the cone base circle.
        // radial = outward from Z axis, z_up = (0,0,1)
        let radial = [ct, st, 0.0];
        let z_up = [0.0, 0.0, 1.0];

        for k in 0..cone_segs {
            let a0 = pi2 * k as f32 / cone_segs as f32;
            let a1 = pi2 * (k + 1) as f32 / cone_segs as f32;

            let base_pt = |a: f32| -> [f32; 3] {
                let r_comp = arrow_r * a.cos();
                let z_comp = arrow_r * a.sin();
                [
                    base[0] + radial[0] * r_comp + z_up[0] * z_comp,
                    base[1] + radial[1] * r_comp + z_up[1] * z_comp,
                    base[2] + radial[2] * r_comp + z_up[2] * z_comp,
                ]
            };

            let bp0 = base_pt(a0);
            let bp1 = base_pt(a1);

            // Side normal: cross(tip - bp0, bp1 - bp0) (outward facing)
            let e1 = [tip[0] - bp0[0], tip[1] - bp0[1], tip[2] - bp0[2]];
            let e2 = [bp1[0] - bp0[0], bp1[1] - bp0[1], bp1[2] - bp0[2]];
            let sn = [
                e1[1] * e2[2] - e1[2] * e2[1],
                e1[2] * e2[0] - e1[0] * e2[2],
                e1[0] * e2[1] - e1[1] * e2[0],
            ];
            let sn_len = (sn[0] * sn[0] + sn[1] * sn[1] + sn[2] * sn[2]).sqrt();
            let sn_norm = if sn_len > 1e-12 {
                [sn[0] / sn_len, sn[1] / sn_len, sn[2] / sn_len]
            } else {
                tip_n
            };

            // Cone side triangle: bp0, bp1, tip
            push_vert(&mut v, bp0, sn_norm);
            push_vert(&mut v, bp1, sn_norm);
            push_vert(&mut v, tip, tip_n);

            // Base cap triangle: center, bp1, bp0 (reversed winding for inward-facing normal)
            let neg_tang = [-tang[0], -tang[1], -tang[2]];
            push_vert(&mut v, base, neg_tang);
            push_vert(&mut v, bp1, neg_tang);
            push_vert(&mut v, bp0, neg_tang);
        }
    }

    v
}
