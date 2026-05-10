//! D3.1 — misarta dynamics primitive profiling on namiashi.
//!
//! Measures per-call cost of every primitive that a 24-state full-
//! centroidal NMPC would evaluate at each shooting node, so we know
//! whether re-solving at 33 Hz with horizon=10 + sqp_iter=3 is feasible
//! before committing to D3.2 implementation.
//!
//! Run:
//!   cargo run --release --example bench_misarta_dynamics
//!
//! Output: markdown table to stdout + per-MPC-solve cost estimate.
//
// No external bench framework — just `Instant` over a fixed iteration
// count. Releases-mode loops with black-box-style nudging are good
// enough for ms-scale measurements.

use std::path::PathBuf;
use std::time::Instant;

use misarta::aba::compute_minv;
use misarta::centroidal::{
    compute_centroidal_inertia, compute_centroidal_momentum_matrix,
    compute_centroidal_momentum_matrix_time_derivative, compute_com, compute_com_jacobian,
};
use misarta::coriolis::compute_coriolis_matrix;
use misarta::crba::crba;
use misarta::rnea::{compute_gravity, rnea};
use misarta::urdf::load_urdf;

const ITERS: usize = 200;

fn namiashi_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests")
        .join("fixtures")
        .join("namiashi")
        .join("urdf")
        .join("namiashi.urdf")
}

fn time_us<F: FnMut()>(label: &str, mut f: F) -> f64 {
    // Warm-up to factor out first-call alloc.
    for _ in 0..5 {
        f();
    }
    let t0 = Instant::now();
    for _ in 0..ITERS {
        f();
    }
    let elapsed_us = t0.elapsed().as_secs_f64() * 1e6 / ITERS as f64;
    println!("| {label:<48} | {elapsed_us:>10.2} |");
    elapsed_us
}

fn main() {
    let path = namiashi_path();
    let model = load_urdf(&path).expect("namiashi URDF must load");
    let nq = model.nq;
    let nv = model.nv;
    println!("# misarta dynamics profile — namiashi (nq={nq}, nv={nv})");
    println!();

    // Generate a non-trivial pose (small offsets on every DOF).
    let q: Vec<f64> = (0..nq).map(|i| 0.05 * (i as f64 + 1.0)).collect();
    let v: Vec<f64> = (0..nv).map(|i| 0.10 * (i as f64 + 1.0)).collect();
    let a: Vec<f64> = (0..nv).map(|i| 0.02 * (i as f64 + 1.0)).collect();

    println!("| Primitive                                        | μs / call  |");
    println!("|--------------------------------------------------|------------|");

    let _ = time_us("compute_com (FK + mass-weighted sum)", || {
        std::hint::black_box(compute_com(&model, std::hint::black_box(&q)));
    });
    let _ = time_us("compute_com_jacobian (3 × nv)", || {
        std::hint::black_box(compute_com_jacobian(&model, std::hint::black_box(&q)));
    });
    let t_cmm = time_us("compute_centroidal_momentum_matrix (CMM, 6 × nv)", || {
        std::hint::black_box(compute_centroidal_momentum_matrix(
            &model,
            std::hint::black_box(&q),
        ));
    });
    let t_cmm_dot = time_us(
        "compute_centroidal_momentum_matrix_time_derivative (FD!)",
        || {
            std::hint::black_box(compute_centroidal_momentum_matrix_time_derivative(
                &model,
                std::hint::black_box(&q),
                std::hint::black_box(&v),
            ));
        },
    );
    let _ = time_us("compute_centroidal_inertia (6×6)", || {
        std::hint::black_box(compute_centroidal_inertia(
            &model,
            std::hint::black_box(&q),
        ));
    });
    let t_crba = time_us("crba (mass matrix M, nv × nv)", || {
        std::hint::black_box(crba(&model, std::hint::black_box(&q)));
    });
    let t_rnea = time_us("rnea (inverse dynamics τ)", || {
        std::hint::black_box(rnea(
            &model,
            std::hint::black_box(&q),
            std::hint::black_box(&v),
            std::hint::black_box(&a),
        ));
    });
    let _ = time_us("compute_gravity (g(q))", || {
        std::hint::black_box(compute_gravity(&model, std::hint::black_box(&q)));
    });
    let _ = time_us("compute_coriolis_matrix (C, nv × nv)", || {
        std::hint::black_box(compute_coriolis_matrix(
            &model,
            std::hint::black_box(&q),
            std::hint::black_box(&v),
        ));
    });
    let _ = time_us("compute_minv (M⁻¹ via ABA)", || {
        std::hint::black_box(compute_minv(&model, std::hint::black_box(&q)));
    });

    println!();
    println!("# MPC node-cost projection");
    println!();
    // legged_control's full-centroidal NMPC needs at each node:
    //   - CMM A(q)                     for centroidal dynamics
    //   - Ȧ(q,q̇)                       for ḣ rate
    //   - CRBA M(q) (or compute_minv)  for floating-base coupling
    //   - RNEA / Coriolis              for the joint-side ID balance
    let per_node_us = t_cmm + t_cmm_dot + t_crba + t_rnea;
    let horizons = [10usize, 12, 16, 20];
    let sqp_iters = [1usize, 3, 5];
    println!("Per-node cost (CMM + Ȧ + CRBA + RNEA): **{per_node_us:.1} μs**");
    println!();
    println!("| horizon | sqp=1   | sqp=3   | sqp=5   |");
    println!("|---------|---------|---------|---------|");
    for h in horizons {
        let mut row = format!("| N={h:<6} ");
        for s in sqp_iters {
            let total_ms = per_node_us * (h * s) as f64 / 1000.0;
            row.push_str(&format!("| {total_ms:>5.1} ms "));
        }
        row.push('|');
        println!("{row}");
    }
    println!();
    println!("Target: ≤ 30 ms / solve (33 Hz re-plan). Anything above that");
    println!("means D3 needs structural optimization (cache shared FK, replace");
    println!("FD Ȧ with analytical, or shrink horizon).");
    println!();
    println!("# Notable observations");
    println!();
    println!("- `compute_centroidal_momentum_matrix_time_derivative` ({t_cmm_dot:.1} μs) is");
    println!("  ~{:.1}× the cost of a single CMM ({t_cmm:.1} μs) — finite-difference",
        t_cmm_dot / t_cmm.max(1e-3));
    println!("  via 2·nnz(v) CMM calls ({} expected). Replace with analytical Ȧ", 2 * nv);
    println!("  (Pinocchio dccrba) for ~{}× speedup if D3 hits node-budget.", 2 * nv);
    println!("- CMM + CRBA share the same propagated FK + spatial inertia composite;");
    println!("  a fused evaluator could amortize ~{:.0}% by reusing intermediates.",
        100.0 * (t_cmm.min(t_crba) / (t_cmm + t_crba)));
}
