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
