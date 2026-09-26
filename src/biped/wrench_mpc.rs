//! Convex receding-horizon centroidal MPC for two sole contact wrenches.
//!
//! The prediction freezes each contact moment arm for one QP solve, which
//! makes the SRBD dynamics linear. Wrenches are expressed at the sole centre
//! in the sole frame as `[mx, my, mz, fx, fy, fz]`.

use misa_wbc::qp::{QpConfig, QpSolution, QpStatus, QpWorkspace, solve_qp, solve_qp_warm};
use nalgebra as na;

pub type Wrench = na::SVector<f64, 6>;

#[derive(Clone, Debug)]
pub struct Contact {
    pub position: na::Vector3<f64>,
    pub rotation: na::Matrix3<f64>,
    /// Available rectangular CoP region for this contact. This may be a
    /// narrow toe/heel patch while the sole is rolling onto a tread.
    pub cop_half: (f64, f64),
}

#[derive(Clone, Debug)]
pub struct Input {
    pub com: na::Vector3<f64>,
    pub velocity: na::Vector3<f64>,
    pub angular_momentum: na::Vector3<f64>,
    pub target_com: na::Vector3<f64>,
    pub target_velocity: na::Vector3<f64>,
    pub target_acceleration: na::Vector3<f64>,
    pub contacts: [Contact; 2],
    /// Fixed contact availability for every prediction stage. An unavailable
    /// contact has all six wrench components constrained to zero. Keeping the
    /// schedule in the optimisation input prevents the SRBD model from using
    /// a reaction wrench that the plant cannot realise.
    pub contact_schedule: Vec<[bool; 2]>,
    /// Continuous contact authority per prediction stage. Zero removes the
    /// wrench and one permits `Config::max_fz`; intermediate values cap the
    /// full friction/CoP cone through its normal-force bound.
    pub activation_schedule: Vec<[f64; 2]>,
    /// Per-stage desired vertical force in each sole frame.
    pub reference_fz_schedule: Vec<[f64; 2]>,
    /// Where the NET pressure of both feet should sit, per node, in world
    /// [m]. Only x and y; the height is taken from each contact.
    ///
    /// The first version of this asked for a CoP on each sole separately and
    /// made things WORSE (the left/right force gap grew 43 -> 63 N, measured
    /// 2026-09-25): the walk's plan is a NET pressure point, and in double
    /// support each sole's own CoP need not sit anywhere near it. What the
    /// gait actually constrains is the net, which is the force-weighted mean
    /// of the two -- and that is still linear in the wrench:
    ///
    ///     sum_i [ m_i + (p_i - c) x f_i ]_z-free  =>  two rows
    ///
    /// written out, with `p_i` the contact origin and `c` the target:
    ///     sum_i ( m_x,i + (p_iy - c_y) f_z,i - (p_iz - c_z) f_y,i ) = 0
    ///     sum_i ( m_y,i - (p_ix - c_x) f_z,i + (p_iz - c_z) f_x,i ) = 0
    pub reference_net_cop_schedule: Option<Vec<na::Vector2<f64>>>,
    /// Superseded, kept so the field名 does not silently change meaning.
    /// Where the pressure should sit on each sole, per node, in that sole's
    /// own frame [m]. `None` (the default) adds nothing, which is the
    /// behaviour every recipe measured before 2026-09-25 was tuned under.
    ///
    /// Without this the MPC has no opinion about the CoP, so in double
    /// support -- where the split of a given total wrench is underdetermined
    /// -- it picks whatever its wrench and smoothness terms like. Measured
    /// 2026-09-25: the totals it asks for are right (median 1.008 body
    /// weights) but the left/right split differs from the walk's by 40-55 N,
    /// and since that split IS the CoP, applying the MPC's answer moved the
    /// pressure a few centimetres and no blend fraction, replacement or
    /// horizon change could walk with it. The gait's intent lived only in the
    /// DCM plan; this is how the MPC is told about it.
    ///
    /// The CoP is a ratio of moment to normal force, but the condition
    /// "pressure at `c`" is linear in the wrench:
    ///     m_y + c_x f_z = 0,   m_x - c_y f_z = 0
    /// so it costs the problem nothing in structure.
    pub reference_cop_schedule: Option<Vec<[na::Vector2<f64>; 2]>>,
    /// Where the divergent component of motion should be, per node, in
    /// world [m]; horizontal only.
    ///
    /// The MPC's own feedback law is whatever its cost implies. With the CoM
    /// position and velocity penalised SEPARATELY that law is not a DCM law
    /// and cannot be made into one by choosing the two weights, because a
    /// diagonal cost in (c, c_dot) has no cross term and the DCM cost
    ///
    ///     |c + c_dot/omega - xi_ref|^2
    ///
    /// is exactly a cross term. Meanwhile the WBC's commanded pressure point
    /// IS a DCM law. So the two ran different feedback on the same plant, and
    /// measured 2026-09-25 their double-support force split would not come
    /// closer than 33 N however the other weights were moved -- the floor was
    /// the mismatch itself, not a tuning failure.
    ///
    /// Only x and y: the vertical has no divergent LIPM mode.
    pub reference_dcm_schedule: Option<Vec<na::Vector2<f64>>>,
    /// Per-stage lower bound on vertical force. This lets touchdown load be
    /// admitted continuously instead of imposing a full support load on the
    /// first sample that reports contact.
    pub min_fz_schedule: Vec<[f64; 2]>,
    pub previous: [Wrench; 2],
}

#[derive(Clone, Debug)]
pub struct Config {
    pub horizon: usize,
    pub dt: f64,
    pub mass: f64,
    pub gravity: f64,
    pub friction_mu: f64,
    pub cop_half: (f64, f64),
    pub torsion_mu: f64,
    pub max_fz: f64,
    pub position_weight: f64,
    pub velocity_weight: f64,
    pub momentum_weight: f64,
    pub acceleration_weight: f64,
    pub wrench_weight: f64,
    /// Weight on the CoP residual (see `Input::reference_cop_schedule`).
    /// 0 = off = the behaviour before 2026-09-25.
    pub cop_weight: f64,
    /// Weight on the NET CoP residual (see `Input::reference_net_cop_schedule`).
    pub net_cop_weight: f64,
    /// Make the net CoP an EQUALITY instead of a cost.
    ///
    /// As a cost it is only a wish: measured 2026-09-25, raising the weight
    /// saturated at a left/right force gap of 57 N against the walk's own
    /// split, because the MPC still trades the CoP off against its wrench and
    /// smoothness terms. The CoP has a centimetre-scale tolerance, so a wish
    /// is not enough -- and two things wishing for different CoPs is exactly
    /// the authority conflict that stopped the MPC being usable at all. As an
    /// equality the MPC instead picks the best force split AMONG those that
    /// put the pressure where the walk asked.
    ///
    /// Only where both feet are scheduled down: in single support the net CoP
    /// is that one sole's own CoP, already bounded by its box, and an equality
    /// there can only make the problem infeasible.
    pub net_cop_equality: bool,
    /// Weight on the DCM residual (see `Input::reference_dcm_schedule`).
    /// 0 = off = the behaviour before 2026-09-25.
    pub dcm_weight: f64,
    /// LIPM frequency sqrt(g/z) [1/s] for the DCM term. The term is skipped
    /// unless this and `dcm_weight` are both positive.
    pub omega: f64,
    pub smooth_weight: f64,
    /// Independent MPC backend configuration; defaults preserve ActiveSet.
    pub qp: QpConfig,
    /// Opt-in common formal-status/KKT gate, including the default backend.
    /// Closed-loop comparisons enable this for baseline and candidates alike.
    pub validate_solution: bool,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            horizon: 8,
            dt: 0.05,
            mass: 1.0,
            gravity: 9.81,
            friction_mu: 0.6,
            cop_half: (0.05, 0.025),
            torsion_mu: 0.02,
            max_fz: 1000.0,
            position_weight: 20.0,
            velocity_weight: 4.0,
            momentum_weight: 2.0,
            acceleration_weight: 6.0,
            wrench_weight: 2e-4,
            cop_weight: 0.0,
            net_cop_weight: 0.0,
            net_cop_equality: false,
            dcm_weight: 0.0,
            omega: 0.0,
            smooth_weight: 1e-3,
            qp: QpConfig::default(),
            validate_solution: false,
        }
    }
}

#[derive(Clone, Debug)]
pub struct Solution {
    pub sole_wrenches: [Wrench; 2],
    pub acceleration: na::Vector3<f64>,
    pub terminal_com: na::Vector3<f64>,
    pub terminal_momentum: na::Vector3<f64>,
}

fn skew(v: &na::Vector3<f64>) -> na::Matrix3<f64> {
    na::Matrix3::new(0.0, -v.z, v.y, v.z, 0.0, -v.x, -v.y, v.x, 0.0)
}

/// Dense original QP, exposed for identical-input solver comparisons.
#[derive(Clone, Debug)]
pub struct DenseProblem {
    pub h: na::DMatrix<f64>,
    pub linear: na::DVector<f64>,
    pub d: na::DMatrix<f64>,
    pub rhs: na::DVector<f64>,
    pub initial: na::DVector<f64>,
    /// Constant omitted from the solver objective (for OCP equivalence).
    pub objective_constant: f64,
    pub terminal_map: na::DMatrix<f64>,
    pub terminal_bias: na::DVector<f64>,
    /// Equality rows, when something asks for one (see
    /// `Config::net_cop_equality`). `None` keeps the shipped problem, which
    /// has inequalities only.
    pub a_eq: Option<na::DMatrix<f64>>,
    pub b_eq: Option<na::DVector<f64>>,
}

/// Assemble without running a solver; allocation time is separate from solve time.
pub fn assemble(input: &Input, cfg: &Config) -> Option<DenseProblem> {
    let n = cfg.horizon;
    let nu = 12 * n;
    if n == 0
        || [
            cfg.dt,
            cfg.mass,
            cfg.gravity,
            cfg.friction_mu,
            cfg.cop_half.0,
            cfg.cop_half.1,
            cfg.torsion_mu,
            cfg.max_fz,
            cfg.position_weight,
            cfg.velocity_weight,
            cfg.momentum_weight,
            cfg.acceleration_weight,
            cfg.wrench_weight,
            cfg.cop_weight,
            cfg.net_cop_weight,
            cfg.dcm_weight,
            cfg.omega,
            cfg.smooth_weight,
        ]
        .iter()
        .any(|v| !v.is_finite())
        || [
            cfg.position_weight,
            cfg.velocity_weight,
            cfg.momentum_weight,
            cfg.acceleration_weight,
            cfg.wrench_weight,
            cfg.cop_weight,
            cfg.net_cop_weight,
            cfg.dcm_weight,
            cfg.omega,
            cfg.smooth_weight,
            cfg.torsion_mu,
        ]
        .iter()
        .any(|v| *v < 0.0)
        || cfg.dt <= 0.0
        || cfg.mass <= 0.0
        || cfg.gravity <= 0.0
        || cfg.friction_mu < 0.0
        || cfg.cop_half.0 <= 0.0
        || cfg.cop_half.1 <= 0.0
        || cfg.max_fz <= 0.0
        || input.contact_schedule.len() != n
        || input.activation_schedule.len() != n
        || input.reference_fz_schedule.len() != n
        || input
            .reference_cop_schedule
            .as_ref()
            .is_some_and(|c| c.len() != n)
        || input
            .reference_net_cop_schedule
            .as_ref()
            .is_some_and(|c| c.len() != n)
        || input.min_fz_schedule.len() != n
        || input.contacts.iter().any(|c| {
            !c.cop_half.0.is_finite()
                || !c.cop_half.1.is_finite()
                || c.cop_half.0 <= 0.0
                || c.cop_half.1 <= 0.0
                || c.rotation
                    .iter()
                    .chain(c.position.iter())
                    .any(|x| !x.is_finite())
        })
        || input
            .reference_fz_schedule
            .iter()
            .zip(&input.min_fz_schedule)
            .zip(&input.activation_schedule)
            .any(|((reference, minimum), activation)| {
                (0..2).any(|foot| {
                    !reference[foot].is_finite()
                        || !minimum[foot].is_finite()
                        || !activation[foot].is_finite()
                        || activation[foot] < 0.0
                        || activation[foot] > 1.0
                        || minimum[foot] < 0.0
                        || reference[foot] < minimum[foot]
                        || minimum[foot] > cfg.max_fz * activation[foot]
                        || reference[foot] > cfg.max_fz
                })
            })
    {
        return None;
    }
    if input
        .com
        .iter()
        .chain(input.velocity.iter())
        .chain(input.angular_momentum.iter())
        .chain(input.target_com.iter())
        .chain(input.target_velocity.iter())
        .chain(input.target_acceleration.iter())
        .chain(input.previous.iter().flat_map(|w| w.iter()))
        .any(|x| !x.is_finite())
    {
        return None;
    }

    // x = [com, velocity, angular momentum] = map * U + bias.
    let mut map = na::DMatrix::<f64>::zeros(9, nu);
    let mut bias = na::DVector::from_iterator(
        9,
        input
            .com
            .iter()
            .chain(input.velocity.iter())
            .chain(input.angular_momentum.iter())
            .copied(),
    );
    let mut rows: Vec<(na::DVector<f64>, f64)> = Vec::new();
    let mut eq_rows: Vec<na::DVector<f64>> = Vec::new();
    let add_residual = |rows: &mut Vec<_>, coeff: na::DVector<f64>, offset: f64, weight: f64| {
        if weight > 0.0 {
            rows.push((coeff * weight.sqrt(), offset * weight.sqrt()));
        }
    };
    let gravity = na::Vector3::new(0.0, 0.0, -cfg.gravity);

    for k in 0..n {
        for axis in 0..3 {
            let old_c = map.row(axis).into_owned();
            let old_v = map.row(3 + axis).into_owned();
            map.row_mut(axis).copy_from(&(old_c + old_v.scale(cfg.dt)));
            bias[axis] += cfg.dt * bias[3 + axis] + 0.5 * cfg.dt * cfg.dt * gravity[axis];
            bias[3 + axis] += cfg.dt * gravity[axis];
        }
        for foot in 0..2 {
            let base = 12 * k + 6 * foot;
            let contact = &input.contacts[foot];
            let force_gain = contact.rotation / cfg.mass;
            let torque_gain = skew(&(contact.position - input.target_com)) * contact.rotation;
            for r in 0..3 {
                for j in 0..3 {
                    map[(r, base + 3 + j)] += 0.5 * cfg.dt * cfg.dt * force_gain[(r, j)];
                    map[(3 + r, base + 3 + j)] += cfg.dt * force_gain[(r, j)];
                    map[(6 + r, base + j)] += cfg.dt * contact.rotation[(r, j)];
                    map[(6 + r, base + 3 + j)] += cfg.dt * torque_gain[(r, j)];
                }
            }
        }

        let frac = (k + 1) as f64 / n as f64;
        let ref_c = input.com + (input.target_com - input.com) * frac;
        let ref_v = input.velocity + (input.target_velocity - input.velocity) * frac;
        let terminal = if k + 1 == n { 4.0 } else { 1.0 };
        for axis in 0..3 {
            add_residual(
                &mut rows,
                map.row(axis).transpose(),
                bias[axis] - ref_c[axis],
                cfg.position_weight * terminal,
            );
            add_residual(
                &mut rows,
                map.row(3 + axis).transpose(),
                bias[3 + axis] - ref_v[axis],
                cfg.velocity_weight * terminal,
            );
            add_residual(
                &mut rows,
                map.row(6 + axis).transpose(),
                bias[6 + axis],
                cfg.momentum_weight * terminal,
            );
        }

        // Penalise the DIVERGENT component, xi = c + c_dot/omega, which is
        // the only thing the pressure point can steer. Built from the rows
        // already assembled above, so it costs one axpy per axis.
        if cfg.dcm_weight > 0.0 && cfg.omega > 0.0 {
            for axis in 0..2 {
                let reference = input
                    .reference_dcm_schedule
                    .as_ref()
                    .and_then(|s| s.get(k))
                    .map(|xi| xi[axis])
                    .unwrap_or(ref_c[axis] + ref_v[axis] / cfg.omega);
                let coeff =
                    map.row(axis).transpose() + map.row(3 + axis).transpose() / cfg.omega;
                add_residual(
                    &mut rows,
                    coeff,
                    bias[axis] + bias[3 + axis] / cfg.omega - reference,
                    cfg.dcm_weight * terminal,
                );
            }
        }

        for axis in 0..3 {
            let mut coeff = na::DVector::zeros(nu);
            for foot in 0..2 {
                let base = 12 * k + 6 * foot;
                for j in 0..3 {
                    coeff[base + 3 + j] = input.contacts[foot].rotation[(axis, j)] / cfg.mass;
                }
            }
            add_residual(
                &mut rows,
                coeff,
                gravity[axis] - input.target_acceleration[axis],
                cfg.acceleration_weight,
            );
        }

        for foot in 0..2 {
            for j in 0..6 {
                let idx = 12 * k + 6 * foot + j;
                let mut coeff = na::DVector::zeros(nu);
                coeff[idx] = 1.0;
                let reference = if j == 5 && input.contact_schedule[k][foot] {
                    input.reference_fz_schedule[k][foot]
                } else {
                    0.0
                };
                add_residual(&mut rows, coeff.clone(), -reference, cfg.wrench_weight);
                // The wrench is (m_x, m_y, m_z, f_x, f_y, f_z) -- the CoP box
                // constraints below are written on components 0 and 1, and
                // friction on 3 and 4. A first draft of this term used 3 and 4
                // for the moments and constrained the tangential forces
                // instead; the unit test made the same mistake and passed.
                if cfg.cop_weight > 0.0
                    && input.contact_schedule[k][foot]
                    && let Some(cops) = input.reference_cop_schedule.as_ref()
                    && (j == 0 || j == 1)
                {
                    let c = cops[k][foot];
                    let mut row = na::DVector::zeros(nu);
                    row[idx] = 1.0;
                    // m_x - c_y f_z = 0 ;  m_y + c_x f_z = 0
                    row[12 * k + 6 * foot + 5] = if j == 0 { -c.y } else { c.x };
                    add_residual(&mut rows, row, 0.0, cfg.cop_weight);
                }
                if k == 0 {
                    add_residual(
                        &mut rows,
                        coeff,
                        -input.previous[foot][j],
                        cfg.smooth_weight,
                    );
                } else {
                    coeff[idx - 12] = -1.0;
                    add_residual(&mut rows, coeff, 0.0, cfg.smooth_weight);
                }
            }
        }
        // Net CoP over both feet (see `reference_net_cop_schedule`). Two rows
        // per node, spanning both feet, so it constrains the SPLIT rather than
        // each foot's own pressure point.
        if (cfg.net_cop_weight > 0.0 || cfg.net_cop_equality)
            && let Some(nets) = input.reference_net_cop_schedule.as_ref()
        {
            let c = nets[k];
            let mut rx = na::DVector::zeros(nu);
            let mut ry = na::DVector::zeros(nu);
            let mut any = false;
            for foot in 0..2 {
                if !input.contact_schedule[k][foot] {
                    continue;
                }
                any = true;
                let contact = &input.contacts[foot];
                let p = contact.position;
                let rot = contact.rotation;
                let b = 12 * k + 6 * foot;
                // The wrench is in the SOLE's frame: the dynamics multiplies
                // it by `contact.rotation` before use, and the friction and
                // CoP boxes are written on its local components. A first
                // version of these rows added the LOCAL moment to a world
                // lever arm; with flat feet at zero yaw that is nearly right,
                // which is how it half-worked (43 -> 32 N) and hid the error.
                //
                // World moment about `c`:  R m + (p - c) x (R f).
                let d = na::Vector3::new(p.x - c.x, p.y - c.y, p.z);
                let cross = skew(&d) * rot;
                for j in 0..3 {
                    rx[b + j] += rot[(0, j)];
                    ry[b + j] += rot[(1, j)];
                    rx[b + 3 + j] += cross[(0, j)];
                    ry[b + 3 + j] += cross[(1, j)];
                }
            }
            let both = input.contact_schedule[k][0] && input.contact_schedule[k][1];
            if any && cfg.net_cop_equality && both {
                eq_rows.push(rx.clone());
                eq_rows.push(ry.clone());
            } else if any && cfg.net_cop_weight > 0.0 {
                add_residual(&mut rows, rx, 0.0, cfg.net_cop_weight);
                add_residual(&mut rows, ry, 0.0, cfg.net_cop_weight);
            }
        }
    }

    let (a_eq, b_eq) = if eq_rows.is_empty() {
        (None, None)
    } else {
        let mut a = na::DMatrix::zeros(eq_rows.len(), nu);
        for (i, r) in eq_rows.iter().enumerate() {
            a.row_mut(i).copy_from(&r.transpose());
        }
        (Some(a), Some(na::DVector::zeros(eq_rows.len())))
    };
    let mut objective = na::DMatrix::zeros(rows.len(), nu);
    let mut offset = na::DVector::zeros(rows.len());
    for (r, (a, b)) in rows.into_iter().enumerate() {
        objective.row_mut(r).copy_from(&a.transpose());
        offset[r] = b;
    }
    let h = objective.transpose() * &objective + na::DMatrix::identity(nu, nu) * 1e-10;
    let objective_constant = 0.5 * offset.norm_squared();
    let linear = objective.transpose() * offset;

    // Sole-frame friction, CoP, torsion, and normal-force constraints.
    let mut d = na::DMatrix::zeros(24 * n, nu);
    let mut rhs = na::DVector::zeros(24 * n);
    let mut row = 0;
    for k in 0..n {
        for foot in 0..2 {
            let base = 12 * k + 6 * foot;
            let fz = base + 5;
            let bounds = [
                (base + 3, 1.0, cfg.friction_mu),
                (base + 3, -1.0, cfg.friction_mu),
                (base + 4, 1.0, cfg.friction_mu),
                (base + 4, -1.0, cfg.friction_mu),
                (base, 1.0, input.contacts[foot].cop_half.1),
                (base, -1.0, input.contacts[foot].cop_half.1),
                (base + 1, 1.0, input.contacts[foot].cop_half.0),
                (base + 1, -1.0, input.contacts[foot].cop_half.0),
                (base + 2, 1.0, cfg.torsion_mu),
                (base + 2, -1.0, cfg.torsion_mu),
            ];
            for &(idx, sign, scale) in &bounds {
                d[(row, idx)] = sign;
                d[(row, fz)] = -scale;
                row += 1;
            }
            let active = input.contact_schedule[k][foot];
            let activation = if active {
                input.activation_schedule[k][foot]
            } else {
                0.0
            };
            d[(row, fz)] = 1.0;
            rhs[row] = cfg.max_fz * activation;
            row += 1;
            d[(row, fz)] = -1.0;
            rhs[row] = if active {
                -input.min_fz_schedule[k][foot]
            } else {
                0.0
            };
            row += 1;
        }
    }

    let mut initial = na::DVector::zeros(nu);
    for k in 0..n {
        for foot in 0..2 {
            if input.contact_schedule[k][foot] {
                initial[12 * k + 6 * foot + 5] = input.reference_fz_schedule[k][foot]
                    .clamp(input.min_fz_schedule[k][foot], cfg.max_fz);
            }
        }
    }
    Some(DenseProblem {
        h,
        linear,
        d,
        rhs,
        initial,
        objective_constant,
        terminal_map: map,
        terminal_bias: bias,
        a_eq,
        b_eq,
    })
}

/// State is invalidated when contact availability or constraint structure changes.
#[derive(Default)]
pub struct Workspace {
    pub qp: QpWorkspace,
    signature: Vec<u64>,
    last_report: Option<SolveReport>,
}
/// Diagnostic snapshot for the most recent valid assembled MPC problem.
#[derive(Clone, Debug)]
pub struct SolveReport {
    pub status: QpStatus,
    pub iterations: usize,
    pub native: Option<misa_wbc::qp::QpDiagnostics>,
    pub residuals: Residuals,
    pub assembly_ns: u128,
    pub solver_call_ns: u128,
    pub validation_ns: u128,
}

impl Workspace {
    pub fn clear(&mut self) {
        self.qp.clear();
        self.signature.clear();
        self.last_report = None;
    }
    pub fn last_report(&self) -> Option<&SolveReport> {
        self.last_report.as_ref()
    }
    pub fn prepare(&mut self, input: &Input, problem: &DenseProblem) {
        let mut signature = vec![
            problem.h.nrows() as u64,
            problem.d.nrows() as u64,
            problem.a_eq.as_ref().map_or(0, |a| a.nrows()) as u64,
        ];
        signature.extend(
            input
                .contact_schedule
                .iter()
                .flat_map(|v| v.iter().map(|b| *b as u64)),
        );
        signature.extend(
            input
                .activation_schedule
                .iter()
                .flat_map(|v| v.iter().map(|a| (*a > 0.0) as u64)),
        );
        signature.extend(problem.d.iter().map(|v| (*v != 0.0) as u64));
        if signature != self.signature {
            self.qp.clear();
            self.signature = signature;
        }
    }
}

pub fn solve_dense(
    problem: &DenseProblem,
    cfg: &QpConfig,
    workspace: Option<&mut QpWorkspace>,
) -> QpSolution {
    let mut cfg = cfg.clone();
    cfg.prox_weight = 0.0;
    // The warm start is built from the vertical-force schedule and satisfies
    // no equality we might add. The active-set backend answers `Infeasible`
    // for a feasible problem when `x0` violates an equality (misa-wbc
    // `equality_qp_is_solved_even_from_a_violating_warm_start`), so drop it
    // when there are equality rows.
    let initial = problem.a_eq.is_none().then_some(&problem.initial);
    match workspace {
        Some(ws) => solve_qp_warm(
            &problem.h,
            &problem.linear,
            problem.a_eq.as_ref(),
            problem.b_eq.as_ref(),
            Some(&problem.d),
            Some(&problem.rhs),
            initial,
            &cfg,
            ws,
        ),
        None => solve_qp(
            &problem.h,
            &problem.linear,
            problem.a_eq.as_ref(),
            problem.b_eq.as_ref(),
            Some(&problem.d),
            Some(&problem.rhs),
            initial,
            &cfg,
        ),
    }
}

/// Solve the linearised SRBD problem and return the first horizon command.
pub fn solve(input: &Input, cfg: &Config) -> Option<Solution> {
    solve_with_workspace(input, cfg, None)
}

pub fn solve_with_workspace(
    input: &Input,
    cfg: &Config,
    mut workspace: Option<&mut Workspace>,
) -> Option<Solution> {
    if let Some(ws) = workspace.as_deref_mut() {
        ws.last_report = None;
    }
    let assembly_started = std::time::Instant::now();
    let problem = assemble(input, cfg)?;
    #[cfg(feature = "qp-hpipm")]
    let use_ocp = cfg.qp.solver == misa_wbc::qp::QpSolver::Hpipm;
    #[cfg(not(feature = "qp-hpipm"))]
    let use_ocp = false;
    #[cfg(feature = "qp-hpipm")]
    let ocp = if use_ocp {
        Some(assemble_ocp(input, cfg)?)
    } else {
        None
    };
    let assembly_ns = assembly_started.elapsed().as_nanos();
    let solve_started = std::time::Instant::now();
    let result = if use_ocp {
        #[cfg(feature = "qp-hpipm")]
        {
            misa_wbc::ocp::solve_ocp(ocp.as_ref().unwrap(), &cfg.qp)
        }
        #[cfg(not(feature = "qp-hpipm"))]
        {
            unreachable!()
        }
    } else if let Some(ws) = workspace.as_deref_mut() {
        ws.prepare(input, &problem);
        solve_dense(&problem, &cfg.qp, Some(&mut ws.qp))
    } else {
        solve_dense(&problem, &cfg.qp, None)
    };
    let solver_call_ns = solve_started.elapsed().as_nanos();
    // A finite Optimal status alone is insufficient for newly selected backends.
    let validation_started = std::time::Instant::now();
    let residual = residuals(&problem, &result);
    let validation_ns = validation_started.elapsed().as_nanos();
    if let Some(ws) = workspace {
        ws.last_report = Some(SolveReport {
            status: result.status,
            iterations: result.iterations,
            native: result.diagnostics.clone(),
            residuals: residual.clone(),
            assembly_ns,
            solver_call_ns,
            validation_ns,
        });
    }
    if (cfg.validate_solution || cfg.qp.solver != misa_wbc::qp::QpSolver::ActiveSet)
        && (!result.formally_solved() || !residual.accepted(1e-6))
    {
        return None;
    }
    let map = &problem.terminal_map;
    let bias = &problem.terminal_bias;
    let gravity = na::Vector3::new(0.0, 0.0, -cfg.gravity);
    if result.status != QpStatus::Optimal || result.x.iter().any(|x| !x.is_finite()) {
        return None;
    }
    let wrenches = [
        Wrench::from_column_slice(&result.x.as_slice()[0..6]),
        Wrench::from_column_slice(&result.x.as_slice()[6..12]),
    ];
    let net_force = input.contacts[0].rotation * wrenches[0].fixed_rows::<3>(3)
        + input.contacts[1].rotation * wrenches[1].fixed_rows::<3>(3);
    Some(Solution {
        sole_wrenches: wrenches,
        acceleration: net_force / cfg.mass + gravity,
        terminal_com: na::Vector3::new(bias[0], bias[1], bias[2])
            + na::Vector3::new(
                (&map.row(0) * &result.x)[0],
                (&map.row(1) * &result.x)[0],
                (&map.row(2) * &result.x)[0],
            ),
        terminal_momentum: na::Vector3::new(bias[6], bias[7], bias[8])
            + na::Vector3::new(
                (&map.row(6) * &result.x)[0],
                (&map.row(7) * &result.x)[0],
                (&map.row(8) * &result.x)[0],
            ),
    })
}

/// Independent residuals in physical units and row/term-relative normalization.
#[derive(Clone, Debug)]
pub struct Residuals {
    pub feasibility: f64,
    pub normalized_feasibility: f64,
    pub stationarity: f64,
    pub normalized_stationarity: f64,
    pub complementarity: f64,
    pub dual_violation: f64,
}
impl Residuals {
    pub fn accepted(&self, tolerance: f64) -> bool {
        [
            self.normalized_feasibility,
            self.normalized_stationarity,
            self.complementarity,
            self.dual_violation,
        ]
        .iter()
        .all(|v| v.is_finite() && *v <= tolerance)
    }
}
pub fn residuals(p: &DenseProblem, result: &QpSolution) -> Residuals {
    let r = misa_wbc::qp_check::check(&p.h, &p.linear, None, Some((&p.d, &p.rhs)), result);
    let invalid = if r.finite { 0.0 } else { f64::INFINITY };
    Residuals {
        feasibility: r.primal_raw.max(invalid),
        normalized_feasibility: r.primal_scaled.max(invalid),
        stationarity: r.stationarity_raw.max(invalid),
        normalized_stationarity: r.stationarity_scaled.max(invalid),
        complementarity: r.complementarity_scaled.max(invalid),
        dual_violation: r.dual_sign.max(invalid),
    }
}

/// Equivalent OCP with 21 states: physical SRBD state plus previous 12 wrenches.
/// Augmentation expresses the wrench-rate cost without cross-stage couplings.
pub fn assemble_ocp(input: &Input, cfg: &Config) -> Option<misa_wbc::ocp::OcpProblem> {
    use misa_wbc::ocp::{OcpProblem, OcpStage};
    let dense = assemble(input, cfg)?;
    let mut initial = na::DVector::zeros(21);
    initial.rows_mut(0, 3).copy_from(&input.com);
    initial.rows_mut(3, 3).copy_from(&input.velocity);
    initial.rows_mut(6, 3).copy_from(&input.angular_momentum);
    for foot in 0..2 {
        initial
            .rows_mut(9 + 6 * foot, 6)
            .copy_from(&input.previous[foot]);
    }
    let mut a = na::DMatrix::zeros(21, 21);
    a.view_mut((0, 0), (9, 9))
        .copy_from(&na::DMatrix::identity(9, 9));
    for j in 0..3 {
        a[(j, 3 + j)] = cfg.dt;
    }
    let mut b = na::DMatrix::zeros(21, 12);
    b.view_mut((9, 0), (12, 12))
        .copy_from(&na::DMatrix::identity(12, 12));
    let mut bias = na::DVector::zeros(21);
    bias[2] = -0.5 * cfg.dt * cfg.dt * cfg.gravity;
    bias[5] = -cfg.dt * cfg.gravity;
    for foot in 0..2 {
        let c = &input.contacts[foot];
        let torque = skew(&(c.position - input.target_com)) * c.rotation;
        for i in 0..3 {
            for j in 0..3 {
                b[(i, 6 * foot + 3 + j)] = 0.5 * cfg.dt * cfg.dt * c.rotation[(i, j)] / cfg.mass;
                b[(3 + i, 6 * foot + 3 + j)] = cfg.dt * c.rotation[(i, j)] / cfg.mass;
                b[(6 + i, 6 * foot + j)] = cfg.dt * c.rotation[(i, j)];
                b[(6 + i, 6 * foot + 3 + j)] = cfg.dt * torque[(i, j)];
            }
        }
    }
    let mut stages = Vec::new();
    for k in 0..cfg.horizon {
        let mut lx = na::DMatrix::zeros(36, 21);
        let mut lu = na::DMatrix::zeros(36, 12);
        let mut offset = na::DVector::zeros(36);
        let terminal = if k + 1 == cfg.horizon { 4.0 } else { 1.0 };
        let frac = (k + 1) as f64 / cfg.horizon as f64;
        let rc = input.com + (input.target_com - input.com) * frac;
        let rv = input.velocity + (input.target_velocity - input.velocity) * frac;
        for axis in 0..9 {
            let weight = (terminal
                * match axis {
                    0..=2 => cfg.position_weight,
                    3..=5 => cfg.velocity_weight,
                    _ => cfg.momentum_weight,
                })
            .sqrt();
            lx.row_mut(axis).copy_from(&(a.row(axis) * weight));
            lu.row_mut(axis).copy_from(&(b.row(axis) * weight));
            offset[axis] = weight
                * (bias[axis]
                    - match axis {
                        0..=2 => rc[axis],
                        3..=5 => rv[axis - 3],
                        _ => 0.0,
                    });
        }
        for axis in 0..3 {
            for foot in 0..2 {
                for j in 0..3 {
                    lu[(9 + axis, 6 * foot + 3 + j)] = cfg.acceleration_weight.sqrt()
                        * input.contacts[foot].rotation[(axis, j)]
                        / cfg.mass;
                }
            }
            offset[9 + axis] = cfg.acceleration_weight.sqrt()
                * ((if axis == 2 { -cfg.gravity } else { 0.0 }) - input.target_acceleration[axis]);
        }
        for j in 0..12 {
            lu[(12 + j, j)] = cfg.wrench_weight.sqrt();
            let foot = j / 6;
            let reference = if j % 6 == 5 && input.contact_schedule[k][foot] {
                input.reference_fz_schedule[k][foot]
            } else {
                0.0
            };
            offset[12 + j] = -cfg.wrench_weight.sqrt() * reference;
            lu[(24 + j, j)] = cfg.smooth_weight.sqrt();
            lx[(24 + j, 9 + j)] = -cfg.smooth_weight.sqrt();
        }
        stages.push(OcpStage {
            a: a.clone(),
            b: b.clone(),
            bias: bias.clone(),
            q_matrix: lx.transpose() * &lx,
            r_matrix: lu.transpose() * &lu + na::DMatrix::identity(12, 12) * 1e-10,
            s_matrix: lu.transpose() * &lx,
            q: lx.transpose() * &offset,
            r: lu.transpose() * &offset,
            c: na::DMatrix::zeros(24, 21),
            d: dense.d.view((24 * k, 12 * k), (24, 12)).into_owned(),
            lower: na::DVector::from_element(24, f64::NEG_INFINITY),
            upper: dense.rhs.rows(24 * k, 24).into_owned(),
            constant: 0.5 * offset.norm_squared(),
        });
    }
    Some(OcpProblem { initial, stages })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The CoP term has to actually move the pressure, and do nothing when
    /// the reference is where the solution already puts it.
    #[test]
    fn the_cop_reference_moves_the_pressure() {
        let (input, cfg) = standing([0.0, 0.0]);
        let plain = solve(&input, &cfg).expect("baseline solves");
        // Ask for the pressure 3 cm forward on the LEFT sole only.
        let want = na::Vector2::new(0.03, 0.0);
        let mut asked = input.clone();
        asked.reference_cop_schedule = Some(vec![[want, na::Vector2::zeros()]; cfg.horizon]);
        let cfg2 = Config { cop_weight: 1.0, ..cfg.clone() };
        let moved = solve(&asked, &cfg2).expect("with a CoP reference it still solves");
        // CoP on a sole is (-m_y / f_z, m_x / f_z).
        let cop_x = |w: &Wrench| -w[1] / w[5].max(1e-9);
        assert!(
            cop_x(&moved.sole_wrenches[0]) > cop_x(&plain.sole_wrenches[0]) + 0.005,
            "left CoP {} -> {}",
            cop_x(&plain.sole_wrenches[0]),
            cop_x(&moved.sole_wrenches[0])
        );
        // The right foot was asked for its centre and should not run away.
        assert!(cop_x(&moved.sole_wrenches[1]).abs() < 0.02);
    }

    /// Net CoP where the physics allows it.
    ///
    /// Asking for a CoP offset while also asking the CoM to stay still is a
    /// contradiction -- the pressure point IS what accelerates the CoM -- and
    /// the first version of this test did exactly that, then read the
    /// resulting infeasibility as a bug in the rows. The lateral acceleration
    /// that a CoP offset `d` implies is `g d / z`, so the target has to move
    /// with it.
    fn leaning(offset_y: f64) -> (Input, Config) {
        let (mut input, cfg) = standing([0.0, 0.0]);
        let z = input.com.z;
        let a = cfg.gravity * offset_y / z;
        input.target_acceleration = na::Vector3::new(0.0, a, 0.0);
        input.reference_net_cop_schedule =
            Some(vec![na::Vector2::new(0.0, offset_y); cfg.horizon]);
        (input, cfg)
    }

    fn net_cop_y(s: &Solution) -> f64 {
        let (wl, wr) = (&s.sole_wrenches[0], &s.sole_wrenches[1]);
        let (fl, fr) = (wl[5], wr[5]);
        ((wl[0] + 0.08 * fl) + (wr[0] - 0.08 * fr)) / (fl + fr).max(1e-9)
    }

    /// The net CoP term must move the force SPLIT, which is the thing the
    /// per-sole version could not do.
    #[test]
    fn the_net_cop_reference_moves_the_split() {
        let (input, cfg) = leaning(0.04);
        let plain = solve(&input, &cfg).expect("baseline solves");
        let cfg2 = Config { net_cop_weight: 10.0, ..cfg.clone() };
        let moved = solve(&input, &cfg2).expect("solves");
        assert!(
            net_cop_y(&moved) > net_cop_y(&plain) + 0.005,
            "net CoP {:.4} -> {:.4}",
            net_cop_y(&plain),
            net_cop_y(&moved)
        );
    }

    /// As an EQUALITY the net CoP is met, not merely preferred.
    #[test]
    fn the_net_cop_equality_is_met() {
        let (input, cfg) = leaning(0.015);
        let soft = Config { net_cop_weight: 10.0, ..cfg.clone() };
        let hard = Config { net_cop_equality: true, ..cfg.clone() };
        let a = solve(&input, &soft).expect("soft solves");
        let b = solve(&input, &hard).expect("hard solves");
        assert!(
            (net_cop_y(&b) - 0.015).abs() < (net_cop_y(&a) - 0.015).abs(),
            "soft {:.4} hard {:.4}",
            net_cop_y(&a),
            net_cop_y(&b)
        );
        assert!((net_cop_y(&b) - 0.015).abs() < 2e-3, "hard {:.4}", net_cop_y(&b));
    }

    /// The equality must not appear in single support, where it could only
    /// make the problem infeasible.
    #[test]
    fn the_equality_is_skipped_in_single_support() {
        let (mut input, cfg) = standing([0.0, 0.0]);
        input.contact_schedule = vec![[true, false]; cfg.horizon];
        input.reference_fz_schedule = vec![[98.1, 0.0]; cfg.horizon];
        input.reference_net_cop_schedule =
            Some(vec![na::Vector2::new(0.0, 0.5); cfg.horizon]);
        let hard = Config { net_cop_equality: true, ..cfg.clone() };
        let problem = assemble(&input, &hard).expect("assembles");
        assert!(problem.a_eq.is_none(), "no equality rows in single support");
        assert!(solve(&input, &hard).is_some(), "and it still solves");
    }

    /// The net CoP rows must be right for a YAWED foot, which is where the
    /// frame mistake shows: with zero yaw a local moment and a world moment
    /// coincide and a wrong row still looks plausible.
    #[test]
    fn the_net_cop_rows_are_in_world_for_a_yawed_foot() {
        let (mut input, cfg) = standing([0.0, 0.0]);
        let yaw = 0.6_f64;
        let (sy, cy) = yaw.sin_cos();
        let rot = na::Matrix3::new(cy, -sy, 0.0, sy, cy, 0.0, 0.0, 0.0, 1.0);
        input.contacts[0].rotation = rot;
        input.contacts[1].rotation = rot;
        input.reference_net_cop_schedule = Some(vec![na::Vector2::zeros(); cfg.horizon]);
        let hard = Config { net_cop_equality: true, ..cfg.clone() };
        let pr = assemble(&input, &hard).expect("assembles");
        let a = pr.a_eq.as_ref().expect("equality rows");
        // Build a wrench by hand whose WORLD net moment about the origin is
        // zero, and check the rows agree: equal vertical loads at +-0.08 with
        // local moments that cancel the lever arms.
        let mut x = na::DVector::zeros(pr.h.ncols());
        for (foot, sign) in [(0usize, 1.0_f64), (1, -1.0)] {
            let b = 6 * foot;
            let fz = 49.05;
            x[b + 5] = fz;
            // world moment from the lever arm is (p x f)_x = p_y * fz
            // so the local moment must produce -p_y*fz in world x:
            // R m = (-p_y fz, 0, 0)  =>  m = R^T (-p_y fz, 0, 0)
            let want = na::Vector3::new(-sign * 0.08 * fz, 0.0, 0.0);
            let m = rot.transpose() * want;
            for j in 0..3 {
                x[b + j] = m[j];
            }
        }
        let r = a * &x;
        assert!(r[0].abs() < 1e-9 && r[1].abs() < 1e-9, "rows give {:?}", (r[0], r[1]));
    }

    #[test]
    fn a_zero_net_cop_weight_changes_nothing() {
        let (input, cfg) = standing([0.0, 0.0]);
        let plain = solve(&input, &cfg).expect("solves");
        let mut asked = input.clone();
        asked.reference_net_cop_schedule =
            Some(vec![na::Vector2::new(0.05, 0.05); cfg.horizon]);
        let same = solve(&asked, &cfg).expect("solves");
        for foot in 0..2 {
            for j in 0..6 {
                assert_eq!(plain.sole_wrenches[foot][j], same.sole_wrenches[foot][j]);
            }
        }
    }

    /// Weight 0 must reproduce the shipped problem exactly, whatever the
    /// schedule says.
    #[test]
    fn a_zero_cop_weight_changes_nothing() {
        let (input, cfg) = standing([0.0, 0.0]);
        let plain = solve(&input, &cfg).expect("solves");
        let mut asked = input.clone();
        asked.reference_cop_schedule =
            Some(vec![[na::Vector2::new(0.04, 0.02); 2]; cfg.horizon]);
        let same = solve(&asked, &cfg).expect("solves");
        for foot in 0..2 {
            for j in 0..6 {
                assert_eq!(plain.sole_wrenches[foot][j], same.sole_wrenches[foot][j]);
            }
        }
    }

    fn standing(min: [f64; 2]) -> (Input, Config) {
        let cfg = Config {
            mass: 10.0,
            max_fz: 150.0,
            horizon: 5,
            ..Config::default()
        };
        let contact = |y: f64, _i: usize| Contact {
            position: na::Vector3::new(0.0, y, 0.0),
            rotation: na::Matrix3::identity(),
            cop_half: cfg.cop_half,
        };
        let com = na::Vector3::new(0.0, 0.0, 0.6);
        (
            Input {
                com,
                velocity: na::Vector3::zeros(),
                angular_momentum: na::Vector3::zeros(),
                target_com: com,
                target_velocity: na::Vector3::zeros(),
                target_acceleration: na::Vector3::zeros(),
                contacts: [contact(0.08, 0), contact(-0.08, 1)],
                contact_schedule: vec![[true; 2]; cfg.horizon],
                activation_schedule: vec![[1.0; 2]; cfg.horizon],
                reference_fz_schedule: vec![[49.05; 2]; cfg.horizon],
                min_fz_schedule: vec![min; cfg.horizon],
                reference_cop_schedule: None,
                reference_net_cop_schedule: None,
                reference_dcm_schedule: None,
                previous: [Wrench::zeros(), Wrench::zeros()],
            },
            cfg,
        )
    }

    /// The stabilising pressure point for a LIPM must lie BEYOND the DCM,
    /// because xi_dot = omega (xi - p): only p > xi brings a forward-running
    /// xi back. The expected value here comes from that identity, not from
    /// the assembled rows -- the last time a test here was written the same
    /// way as the code, it passed while both were wrong.
    #[test]
    fn the_dcm_term_puts_the_pressure_beyond_the_divergent_component() {
        let (mut input, mut cfg) = standing([0.0, 0.0]);
        let omega = (cfg.gravity / input.com.z).sqrt();
        // Running forward with the CoM still at the origin: xi = v/omega.
        input.velocity.x = 0.2;
        let xi = input.velocity.x / omega;
        // Ask to come to rest at the origin, and give the CoM tracking terms
        // nothing to argue with by leaving the targets where the CoM is.
        input.reference_dcm_schedule = Some(vec![na::Vector2::zeros(); cfg.horizon]);

        let without = solve(&input, &cfg).unwrap();
        cfg.dcm_weight = 200.0;
        cfg.omega = omega;
        let with = solve(&input, &cfg).unwrap();

        let net_cop_x = |w: &Solution| {
            let (mut m, mut f) = (0.0, 0.0);
            for foot in 0..2 {
                let c = &input.contacts[foot];
                let wr = w.sole_wrenches[foot];
                let moment = c.rotation * na::Vector3::new(wr[0], wr[1], wr[2]);
                let force = c.rotation * na::Vector3::new(wr[3], wr[4], wr[5]);
                m += moment.y + (c.position.x - input.com.x) * force.z;
                f += force.z;
            }
            // CoP_x = -M_y/F_z about the CoM, plus the CoM's own x.
            input.com.x - m / f
        };
        cfg.dcm_weight = 2000.0;
        let harder = solve(&input, &cfg).unwrap();

        let p_without = net_cop_x(&without);
        let p_with = net_cop_x(&with);
        let p_harder = net_cop_x(&harder);
        assert!(
            p_with > p_without,
            "the DCM term must move the pressure forward ({p_without:.4} -> {p_with:.4})"
        );
        assert!(
            p_harder > p_with,
            "more weight must push further ({p_with:.4} -> {p_harder:.4})"
        );
        // The shipped cost barely reacts: 2.7 mm of pressure against a 49 mm
        // divergence. That is the gap this term exists to close, and it is
        // why the MPC and the WBC -- which runs a real DCM law -- could not
        // be made to agree on the double-support split by any reweighting.
        assert!(
            p_without < 0.2 * xi && p_harder > 0.8 * xi,
            "xi={xi:.4} without={p_without:.4} with={p_with:.4} harder={p_harder:.4}"
        );
    }

    #[test]
    fn dense_and_structured_ocp_are_equivalent() {
        for horizon in [1, 8, 16, 32] {
            let (mut input, mut cfg) = standing([0.0, 0.0]);
            cfg.horizon = horizon;
            input.contact_schedule = vec![[true, false]; horizon];
            input.activation_schedule = vec![[0.7, 0.0]; horizon];
            input.reference_fz_schedule = vec![[70.0, 0.0]; horizon];
            input.min_fz_schedule = vec![[0.0, 0.0]; horizon];
            input.previous[0][5] = 49.0;
            input.target_com.x = 0.1;
            input.contacts[0].rotation =
                na::Rotation3::from_euler_angles(0.1, -0.2, 0.3).into_inner();
            input.contacts[1].cop_half = (0.035, 0.018);
            let dense = assemble(&input, &cfg).unwrap();
            let ocp = assemble_ocp(&input, &cfg).unwrap();
            let u = na::DVector::from_iterator(
                12 * horizon,
                (0..12 * horizon).map(|j| ((j * 17 % 71) as f64 - 35.0) * 0.1),
            );
            let mut x = ocp.initial.clone();
            let mut value = 0.0;
            for (k, stage) in ocp.stages.iter().enumerate() {
                let uk = u.rows(12 * k, 12).into_owned();
                value += 0.5 * x.dot(&(&stage.q_matrix * &x))
                    + 0.5 * uk.dot(&(&stage.r_matrix * &uk))
                    + uk.dot(&(&stage.s_matrix * &x))
                    + stage.q.dot(&x)
                    + stage.r.dot(&uk)
                    + stage.constant;
                let violation = &stage.c * &x + &stage.d * &uk - &stage.upper;
                let dense_violation = dense.d.rows(24 * k, 24) * &u - dense.rhs.rows(24 * k, 24);
                assert!((violation - dense_violation).amax() < 1e-10);
                x = &stage.a * &x + &stage.b * &uk + &stage.bias;
            }
            let expected =
                0.5 * u.dot(&(&dense.h * &u)) + dense.linear.dot(&u) + dense.objective_constant;
            assert!(
                (value - expected).abs() < 1e-8 * (1.0 + expected.abs()),
                "{value} != {expected}"
            );
            let terminal = &dense.terminal_map * &u + &dense.terminal_bias;
            assert!((x.rows(0, 9) - terminal).amax() < 1e-10);
        }
    }

    #[test]
    fn rejects_nonfinite_weights_and_failed_residuals() {
        let (input, mut cfg) = standing([0.0, 0.0]);
        cfg.position_weight = f64::NAN;
        assert!(assemble(&input, &cfg).is_none());
        cfg.position_weight = 20.0;
        let p = assemble(&input, &cfg).unwrap();
        let mut result = solve_dense(&p, &cfg.qp, None);
        result.x[5] = -1.0;
        assert!(!residuals(&p, &result).accepted(1e-6));
    }

    #[cfg(feature = "qp-hpipm")]
    #[test]
    fn hpipm_ocp_matches_dense_original_qp() {
        for horizon in [1, 8, 16, 32] {
            let (mut input, mut cfg) = standing([0.0, 0.0]);
            cfg.horizon = horizon;
            input.contact_schedule = vec![[true, true]; horizon];
            input.activation_schedule = vec![[1.0, 1.0]; horizon];
            input.reference_fz_schedule = vec![[49.05, 49.05]; horizon];
            input.min_fz_schedule = vec![[0.0, 0.0]; horizon];
            let p = assemble(&input, &cfg).unwrap();
            let ocp = assemble_ocp(&input, &cfg).unwrap();
            let mut qp = cfg.qp.clone();
            qp.solver = misa_wbc::qp::QpSolver::Hpipm;
            let structured = misa_wbc::ocp::solve_ocp(&ocp, &qp);
            let dense = solve_dense(&p, &qp, None);
            assert_eq!(
                structured.status,
                QpStatus::Optimal,
                "N={horizon} {:?}",
                structured.diagnostics
            );
            assert_eq!(dense.status, QpStatus::Optimal, "N={horizon}");
            let r = residuals(&p, &structured);
            assert!(r.accepted(1e-6), "N={horizon} {r:?}");
            let original =
                0.5 * structured.x.dot(&(&p.h * &structured.x)) + p.linear.dot(&structured.x);
            assert!(
                (original - dense.objective).abs() < 1e-6 * (1.0 + dense.objective.abs()),
                "N={horizon}: {original} != {}",
                dense.objective
            );
            assert!(solve(&input, &Config { qp, ..cfg }).is_some());
        }
    }

    #[test]
    fn common_validation_flag_rejects_loose_default_backend_solution() {
        let (input, mut cfg) = standing([0.0, 0.0]);
        // A deliberately loose stopping tolerance labels the initial feasible
        // reference Optimal although smoothing leaves a stationarity residual.
        cfg.qp.optimality_tol = 1e6;
        let mut workspace = Workspace::default();
        assert!(solve_with_workspace(&input, &cfg, Some(&mut workspace)).is_some());
        assert!(!workspace.last_report().unwrap().residuals.accepted(1e-6));
        cfg.validate_solution = true;
        assert!(solve_with_workspace(&input, &cfg, Some(&mut workspace)).is_none());
        assert!(!workspace.last_report().unwrap().residuals.accepted(1e-6));
    }

    #[test]
    fn static_balance_shares_weight() {
        let (input, cfg) = standing([0.0, 0.0]);
        let sol = solve(&input, &cfg).unwrap();
        let fz = sol.sole_wrenches[0][5] + sol.sole_wrenches[1][5];
        assert!((fz - cfg.mass * cfg.gravity).abs() < 2.0, "fz={fz}");
        assert!(sol.acceleration.norm() < 0.2, "a={:?}", sol.acceleration);
    }

    #[test]
    fn preserves_low_foot_minimum_load() {
        let (mut input, cfg) = standing([25.0, 0.0]);
        input.reference_fz_schedule.fill([25.0, 73.1]);
        let sol = solve(&input, &cfg).unwrap();
        assert!(sol.sole_wrenches[0][5] >= 25.0 - 1e-6);
    }

    #[test]
    fn returned_wrench_respects_patch_constraints() {
        let (mut input, cfg) = standing([5.0, 5.0]);
        input.target_acceleration = na::Vector3::new(20.0, -20.0, 0.0);
        let sol = solve(&input, &cfg).unwrap();
        for wrench in sol.sole_wrenches {
            let fz = wrench[5];
            assert!(wrench[3].abs() <= cfg.friction_mu * fz + 1e-6);
            assert!(wrench[4].abs() <= cfg.friction_mu * fz + 1e-6);
            assert!(wrench[0].abs() <= cfg.cop_half.1 * fz + 1e-6);
            assert!(wrench[1].abs() <= cfg.cop_half.0 * fz + 1e-6);
            assert!(wrench[2].abs() <= cfg.torsion_mu * fz + 1e-6);
        }
    }

    #[test]
    fn rejects_impossible_minimum() {
        let (input, mut cfg) = standing([160.0, 0.0]);
        cfg.max_fz = 150.0;
        assert!(solve(&input, &cfg).is_none());
    }

    #[test]
    fn unavailable_contact_has_zero_wrench() {
        let (mut input, cfg) = standing([25.0, 5.0]);
        input.contact_schedule = vec![[false, true]; cfg.horizon];
        input
            .reference_fz_schedule
            .fill([25.0, cfg.mass * cfg.gravity]);
        let sol = solve(&input, &cfg).unwrap();
        assert!(
            sol.sole_wrenches[0].norm() < 1e-7,
            "w={:?}",
            sol.sole_wrenches[0]
        );
        assert!(sol.sole_wrenches[1][5] > 0.0);
    }

    #[test]
    fn fractional_activation_caps_normal_force() {
        let (mut input, cfg) = standing([0.0, 0.0]);
        input.activation_schedule.fill([0.4, 1.0]);
        let sol = solve(&input, &cfg).unwrap();
        assert!(sol.sole_wrenches[0][5] <= 0.4 * cfg.max_fz + 1e-6);
    }

    #[test]
    fn rejects_wrong_schedule_length() {
        let (mut input, cfg) = standing([0.0, 0.0]);
        input.contact_schedule.pop();
        assert!(solve(&input, &cfg).is_none());
    }

    #[test]
    fn stage_force_schedule_sets_first_stage_minimum() {
        let (mut input, cfg) = standing([0.0, 0.0]);
        input.reference_fz_schedule = (0..cfg.horizon)
            .map(|k| [10.0 + 8.0 * k as f64, 88.1 - 8.0 * k as f64])
            .collect();
        input.min_fz_schedule = (0..cfg.horizon)
            .map(|k| [5.0 + 5.0 * k as f64, 0.0])
            .collect();
        let sol = solve(&input, &cfg).unwrap();
        assert!(sol.sole_wrenches[0][5] >= 5.0 - 1e-6);
    }

    #[test]
    fn rejects_wrong_force_schedule_length() {
        let (mut input, cfg) = standing([0.0, 0.0]);
        input.reference_fz_schedule.pop();
        assert!(solve(&input, &cfg).is_none());
    }
}
