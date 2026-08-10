//! Search for a recovery trajectory that actually gets namiashi back on its
//! feet, over the 20 parameters of `articara::self_righting::RecoveryParams`.
//!
//! Worth doing because `namiashi_self_righting_probe` established that the
//! ceiling is a scheduling problem, not a physical one: hand-written plans
//! stall on the robot's side at about 45 degrees, but widening hip roll from
//! 1.05 to 2.40 rad does not move that number at all. So the pose is
//! reachable and what is missing is the timing to reach it.
//!
//! Scored through `Drive::TorqueAhrs` -- the torque PD and IMU-filtered
//! attitude `run_wbc_sim` uses -- not the .misa's Position actuators.
//!
//! METHOD: cross-entropy method, matching `go2_wbc_bound_cmaes_mode_h` in
//! `tests/wbc_walk_go2.rs` -- a diagonal Gaussian refit to the elite
//! fraction each generation, no new dependency, deterministic LCG so a run
//! reproduces. `evaluate` is deterministic (MuJoCo forward dynamics and an
//! open-loop trajectory, no RNG on the path), so one evaluation per
//! candidate per condition suffices.
//!
//! OBJECTIVE: every candidate is scored on TEN conditions at once -- all
//! five sites on the ring crossed with friction 0.30 and 1.00 -- and the
//! fitness is `mean + 0.5 * worst`. This is the whole point of the setup.
//! The hip-bias-gate search earlier in this project optimised a single
//! condition and produced a parameter set that collapsed under any
//! perturbation (`namiashi_staircase_5cm_hip_gate_robustness`, 6/6
//! failures); a fitness that a one-condition spike cannot maximise is the
//! structural fix, not more careful interpretation of the result.
//!
//! Friction 0.70 is held out of training entirely, so the validation grid
//! still contains five conditions the search never saw.
//!
//! Run: `cargo run --release --no-default-features --features mujoco
//!       --example namiashi_self_righting_search`
//!
//! Roughly 0.4 s per evaluation, parallel across the population.

#[cfg(feature = "mujoco")]
fn main() {
    use articara::mjcf::KawasakiRingCfg;
    use articara::self_righting::{
        evaluate_with, Drive, RecoveryParams, BOUNDS, DIM, SEARCHED_V4,
    };

    // Search against the control path the teleop actually runs, not the
    // .misa's Position actuators. Round 2 optimised against those and
    // `namiashi_self_righting_check` then measured 15/15 under them and
    // 8/15 through the torque PD and IMU-filtered attitude that
    // `run_wbc_sim` uses. A trajectory that only works under the evaluator
    // it was fitted to is the same failure as fitting one condition.
    const TELEOP: Drive = Drive::TorqueAhrs { kp: 100.0, kd: 1.2, imu_beta: 0.1 };

    const POP: usize = 40;
    const ELITE: usize = 10;
    const GENS: usize = 25;
    /// Search and validation horizons, deliberately equal. They were 6 and 8
    /// and round 4 scored 2.69 out of a possible 2.70 in training while
    /// righting only 4 of 15 in validation -- ten of those fifteen ARE the
    /// training conditions, so the contradiction was entirely the two extra
    /// seconds. The trajectory stood up inside six and toppled inside eight.
    /// A shorter search horizon does not select for faster recoveries, it
    /// selects for ones that have not fallen over yet.
    const SEARCH_S: f64 = 8.0;
    const VALIDATE_S: f64 = 8.0;

    let ring = KawasakiRingCfg::default();
    let (pw, pd) = ring.red_platform_m;
    let misa = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests/fixtures/namiashi/namiashi_3p3_prop.misa");

    // The flat venue floor is the honest baseline (nothing to slide off, no
    // edge to help); the open ring surface adds the plate seams and the 5 mm
    // heightfield relief; the rest are the ring's own features.
    const FLAT: (f64, f64) = (1.60, 1.60);
    const OPEN_RING: (f64, f64) = (0.35, 0.35);
    let sites: [(&str, (f64, f64)); 5] = [
        ("flat venue floor", FLAT),
        ("open ring surface", OPEN_RING),
        ("ring centre (bowl)", (0.0, 0.0)),
        ("on a round plate", ring.round_plate_centres[0]),
        (
            "red platform (edge)",
            (-(ring.ring_m / 2.0 + pd / 2.0), -(ring.ring_m / 2.0 - pw / 2.0)),
        ),
    ];
    // Round 1 trained on two sites and failed on three of the other three,
    // so all five sites are in the training set now. Friction 0.70 is held
    // out entirely instead: it keeps a real generalisation check (five
    // conditions the search never sees) while no longer asking the
    // trajectory to generalise across terrain it was never shown.
    let train: Vec<((f64, f64), f64)> = sites
        .iter()
        .flat_map(|&(_, xy)| [0.30, 1.00].map(move |mu| (xy, mu)))
        .collect();

    // Fitness across the training conditions. `mean + 0.5 * worst`: the mean
    // gives the search something to climb while everything still fails, the
    // worst-case term stops it trading a condition away once things start
    // working.
    let fitness = |p: &RecoveryParams, conds: &[((f64, f64), f64)], secs: f64| -> f64 {
        let mut sum = 0.0;
        let mut worst = f64::INFINITY;
        for &(xy, mu) in conds {
            let s = evaluate_with(&misa, &ring, xy, mu, p, secs, TELEOP).score();
            sum += s;
            worst = worst.min(s);
        }
        sum / conds.len() as f64 + 0.5 * worst
    };

    // Deterministic LCG; std has no RNG and reproducibility is a feature.
    let mut seed: u64 = 0x5EED_1234_ABCD_0001;
    let mut next_unit = move || {
        seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        ((seed >> 11) as f64) / ((1u64 << 53) as f64)
    };
    let gauss = |m: f64, s: f64, u1: f64, u2: f64| {
        let r = (-2.0 * (u1.max(1e-12)).ln()).sqrt();
        m + s * r * (2.0 * std::f64::consts::PI * u2).cos()
    };

    let mut mean = SEARCHED_V4.to_vec();
    let mut sigma = [0.0; DIM];
    for i in 0..DIM {
        // Wider again than a pure refinement: the seed works under a
        // different evaluator, so its optimum is not necessarily near this
        // one's.
        sigma[i] = 0.18 * (BOUNDS[i].1 - BOUNDS[i].0);
    }

    let seed_fit = fitness(&SEARCHED_V4, &train, SEARCH_S);
    println!("seed (previous winner) fitness = {seed_fit:.4}");
    println!("{:>4} {:>10} {:>10} {:>10}", "gen", "best", "elite mean", "sigma sum");

    let mut best = (seed_fit, SEARCHED_V4);
    for g in 0..GENS {
        // Sample the population up front so the RNG draw order does not
        // depend on thread scheduling.
        let mut cands: Vec<RecoveryParams> = Vec::with_capacity(POP);
        cands.push(RecoveryParams::from_vec(&mean));
        while cands.len() < POP {
            let mut v = [0.0; DIM];
            for i in 0..DIM {
                v[i] = gauss(mean[i], sigma[i], next_unit(), next_unit());
            }
            cands.push(RecoveryParams::from_vec(&v));
        }

        let workers = std::thread::available_parallelism().map(|n| n.get()).unwrap_or(4);
        let chunk = cands.len().div_ceil(workers);
        let scored: Vec<(f64, RecoveryParams)> = std::thread::scope(|s| {
            let handles: Vec<_> = cands
                .chunks(chunk)
                .map(|part| s.spawn(|| part.iter().map(|c| (fitness(c, &train, SEARCH_S), *c)).collect::<Vec<_>>()))
                .collect();
            handles.into_iter().flat_map(|h| h.join().unwrap()).collect()
        });

        let mut ranked = scored;
        ranked.sort_by(|a, b| b.0.total_cmp(&a.0));
        if ranked[0].0 > best.0 {
            best = ranked[0];
        }

        // Refit the diagonal Gaussian to the elites.
        let elites = &ranked[..ELITE];
        let mut new_mean = [0.0; DIM];
        for (_, p) in elites {
            let v = p.to_vec();
            for i in 0..DIM {
                new_mean[i] += v[i] / ELITE as f64;
            }
        }
        let mut new_sigma = [0.0; DIM];
        for (_, p) in elites {
            let v = p.to_vec();
            for i in 0..DIM {
                new_sigma[i] += (v[i] - new_mean[i]).powi(2) / ELITE as f64;
            }
        }
        for i in 0..DIM {
            // Floor at 2% of the range: a diagonal CEM collapses its own
            // variance and stops exploring long before it has to.
            new_sigma[i] = new_sigma[i].sqrt().max(0.02 * (BOUNDS[i].1 - BOUNDS[i].0));
        }
        mean = new_mean;
        sigma = new_sigma;

        let elite_mean: f64 = elites.iter().map(|e| e.0).sum::<f64>() / ELITE as f64;
        println!(
            "{g:>4} {:>10.4} {elite_mean:>10.4} {:>10.4}",
            ranked[0].0,
            sigma.iter().sum::<f64>()
        );
    }

    // ---- validation on all fifteen conditions -------------------------
    println!("\nbest fitness {:.4}\n{:#?}\n", best.0, best.1);
    println!(
        "validation at {VALIDATE_S} s ('*' = a condition the search never saw)\n{:<22} {:>5} {:>8} {:>8} {:>8} {:>7}",
        "site", "mu", "peak up", "final", "d_xy m", "t_right"
    );
    let (mut ok, mut total) = (0, 0);
    for (name, xy) in sites {
        for mu in [0.30_f64, 0.70, 1.00] {
            let o = evaluate_with(&misa, &ring, xy, mu, &best.1, VALIDATE_S, TELEOP);
            let unseen = if train.contains(&(xy, mu)) { ' ' } else { '*' };
            total += 1;
            if o.righted() {
                ok += 1;
            }
            println!(
                "{unseen}{name:<21} {mu:>5.2} {:>8.3} {:>8.3} {:>8.3} {:>7.2}  {}",
                o.peak_up,
                o.final_up,
                o.travel_m,
                o.t_right_s,
                if o.righted() { "RIGHTED" } else { "no" }
            );
        }
    }
    println!("\n{ok}/{total} righted");
}

#[cfg(not(feature = "mujoco"))]
fn main() {
    eprintln!("needs --features mujoco");
    std::process::exit(2);
}
