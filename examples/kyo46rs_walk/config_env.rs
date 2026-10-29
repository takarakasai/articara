//! Environment -> [`Config`]: the walker's knobs as this harness reads them.
//!
//! `misa_biped::config` holds the knobs and their code defaults; this file is
//! the table from key to field, with each read's own post-processing
//! (`.max`, `.clamp`, `as usize`) next to it. The same table reads a JSON of
//! the same keys (`get` is any lookup), which is how the presets are checked
//! against the controls they came from (crates/misa-biped/tests/presets.rs).
//! Keys the walker does not own (the rig, the scenario, the command, the
//! diagnostics) are read in `main.rs`.
//!
//! The lookups match the harness's `env_f64` / `flag`: a number that is unset
//! or does not parse is the default, and a switch is on when set to anything
//! but "0".
use misa_biped::config::*;
use misa_biped::profile::Profile;

/// Every walker knob from `get` (`|k| std::env::var(k).ok()` for the
/// environment). Panics on an unsupported `WBC_STRATEGY` / `WBC_QP_BACKEND`.
pub fn config_from_kv(prof: &Profile, get: &dyn Fn(&str) -> Option<String>) -> Config {
    let num = |k: &str, d: f64| -> f64 { get(k).and_then(|s| s.parse().ok()).unwrap_or(d) };
    let flag = |k: &str, d: bool| -> bool { get(k).map(|v| v != "0").unwrap_or(d) };
    let opt = |k: &str| -> Option<f64> { get(k).and_then(|v| v.parse().ok()) };
    let d = Config::code_defaults(prof);

    let wbc = {
        let d = &d.wbc;
        let mut level_weights = d.level_weights.clone();
        for name in LevelWeights::NAMES {
            level_weights.set(name, num(&LevelWeights::key(name), LevelWeights::default_for(name)));
        }
        WbcConfig {
            feas_tol: num("WBC_FEAS_TOL", d.feas_tol),
            strategy: match get("WBC_STRATEGY").as_deref() {
                // Per-task weights are applied to the levels themselves
                // (WBCW_*); the ratio defaults to 1 so they are not scaled
                // again by position.
                Some("weighted") => misa_wbc::HqpStrategy::WeightedQp {
                    level_ratio: num("WBC_WEIGHT_RATIO", 1.0).max(1.0) as u32,
                },
                Some("force_budget") => misa_wbc::HqpStrategy::ForceBudgetCascade,
                Some("null_space") | None => d.strategy,
                Some(other) => panic!("unsupported WBC_STRATEGY: {other}"),
            },
            backend: get("WBC_QP_BACKEND").map(|name| match name.as_str() {
                "clarabel" => misa_wbc::QpSolver::Clarabel,
                "active_set" => misa_wbc::QpSolver::ActiveSet,
                "proxqp" => misa_wbc::QpSolver::Proxqp,
                "eiquadprog" => misa_wbc::QpSolver::Eiquadprog,
                "qpoases" => misa_wbc::QpSolver::Qpoases,
                _ => panic!("unsupported WBC_QP_BACKEND: {name}"),
            }),
            bench_gate: flag("QP_BENCH_GATE", d.bench_gate),
            friction_mu: opt("friction_mu"),
            f_max_scale: num("F_MAX_SCALE", d.f_max_scale),
            f_max: opt("F_MAX"),
            eom_hard: flag("EOM_HARD", d.eom_hard),
            hold_kp: num("HOLD_KP", d.hold_kp),
            hold_kd: num("HOLD_KD", d.hold_kd),
            hold_bridge: num("HOLD_BRIDGE", d.hold_bridge as f64) as u32,
            fallback_level: num("FALLBACK_LEVEL", d.fallback_level as f64) as usize,
            level_weights,
            ctrl_dt: num("CTRL_DT", d.ctrl_dt),
        }
    };

    let joint_limit = {
        let d = &d.joint_limit;
        JointLimitConfig {
            enabled: flag("JLIM", d.enabled),
            alpha: [
                num("JLIM_ALPHA1", d.alpha[0]),
                num("JLIM_ALPHA2", d.alpha[1]),
                num("JLIM_ALPHA3", d.alpha[2]),
            ],
            a_max: num("JLIM_AMAX", d.a_max),
            v_frac: num("JLIM_V_FRAC", d.v_frac).clamp(0.05, 1.0),
        }
    };

    let gains = {
        let d = &d.gains;
        BalanceGains {
            kp_com: num("KP_COM", d.kp_com),
            kd_com: num("KD_COM", d.kd_com),
            kp_trunk: num("KP_TRUNK", d.kp_trunk),
            kd_trunk: num("KD_TRUNK", d.kd_trunk),
            kp_post: num("KP_POST", d.kp_post),
            kd_post: num("KD_POST", d.kd_post),
            posture: flag("POST", d.posture),
            trunk: flag("TRUNK", d.trunk),
            trunk_sign: num("TRUNK_SIGN", d.trunk_sign),
            trunk_yaw: flag("TRUNK_YAW", d.trunk_yaw),
            kp_yaw: num("KP_YAW", d.kp_yaw),
            kd_yaw: num("KD_YAW", d.kd_yaw),
            diag_vz_bias_mpc: num("DIAG_VZ_BIAS_MPC", d.diag_vz_bias_mpc),
            diag_vz_bias_height: num("DIAG_VZ_BIAS_HEIGHT", d.diag_vz_bias_height),
        }
    };

    let dcm = {
        let d = &d.dcm;
        DcmConfig {
            cop_frac: num("COP_FRAC", d.cop_frac),
            k_dcm: num("K_DCM", d.k_dcm),
            vh_dcm: flag("VH_DCM", d.vh_dcm),
            zmp_margin: num("ZMP_MARGIN", d.zmp_margin),
            clamp_zmp: flag("CLAMP_ZMP", d.clamp_zmp),
            zmp_lat_scale: num("ZMP_LAT_SCALE", d.zmp_lat_scale),
            zmp_roll: opt("ZMP_ROLL"),
            com_xy_jerk_max: num("COM_XY_JERK_MAX", d.com_xy_jerk_max),
        }
    };

    let gait = {
        let d = &d.gait;
        GaitConfig {
            t_start: num("T_START", d.t_start),
            t_ds: num("T_DS", d.t_ds),
            t_ss: num("T_SS", d.t_ss),
            n_steps: num("N_STEPS", d.n_steps as f64) as usize,
            first_swing: num("FIRST_SWING", d.first_swing as f64) as usize,
            no_lift: flag("NO_LIFT", d.no_lift),
            turn_in_ds: flag("TURN_IN_DS", d.turn_in_ds),
            ds_settle: num("DS_SETTLE", d.ds_settle),
            ds_settle_after_step: num("DS_SETTLE_AFTER_STEP", d.ds_settle_after_step as f64).max(1.0) as usize,
            stride: num("STRIDE", d.stride),
            stride_ramp: num("STRIDE_RAMP", d.stride_ramp as f64) as usize,
            alternate: flag("ALTERNATE", d.alternate),
        }
    };

    let stair_plan = {
        let d = &d.stair_plan;
        StairPlanConfig {
            toe_margin: num("STAIR_TOE_MARGIN", d.toe_margin),
            y_out: num("STAIR_Y_OUT", d.y_out),
            gait_pair: get("STAIR_GAIT").map(|v| v == "pair").unwrap_or(d.gait_pair),
            td_bias: num("STAIR_TD_BIAS", d.td_bias),
            td_bias_asc_mid: opt("STAIR_TD_BIAS_ASC_MID"),
            full_sole: flag("STAIR_FULL_SOLE", d.full_sole),
            sole_margin: num("STAIR_SOLE_MARGIN", d.sole_margin),
            landing_frac: num("STAIR_LANDING_FRAC", d.landing_frac).max(1e-3),
            align_stand_line: flag("STAIR_ALIGN_STAND_LINE", d.align_stand_line),
        }
    };

    let descent = {
        let d = &d.descent;
        DescentConfig {
            td_bias_desc: opt("STAIR_TD_BIAS_DESC"),
            stand_margin: num("STAIR_DESC_STAND_MARGIN", d.stand_margin).max(0.0),
            com_height_lead: num("COM_HEIGHT_DESC_LEAD", d.com_height_lead).clamp(0.0, 1.0),
            zone_crouch: num("STAIR_ZONE_CROUCH", d.zone_crouch).clamp(0.0, 0.15),
            land_extend: opt("COM_LAND_EXTEND_DESC").map(|v| v.max(0.0)),
            land_confirm_xy: opt("LAND_CONFIRM_XY_DESC"),
            land_confirm_z: opt("LAND_CONFIRM_Z_DESC"),
            leg_reach_margin: num("LEG_REACH_MARGIN", d.leg_reach_margin).clamp(0.1, 1.0),
        }
    };

    let slope = {
        let d = &d.slope;
        SlopeConfig {
            keep_stride: flag("SLOPE_KEEP_STRIDE", d.keep_stride),
            heel_cut: num("SLOPE_HEEL_CUT", d.heel_cut).max(0.0),
            heel_cut_max: num("SLOPE_HEEL_CUT_MAX", d.heel_cut_max).clamp(0.0, 0.08),
            terrain_pitch: flag("TERRAIN_PITCH", d.terrain_pitch),
            terrain_pitch_yaw: flag("TERRAIN_PITCH_YAW", d.terrain_pitch_yaw),
        }
    };

    Config { wbc, joint_limit, gains, dcm, gait, stair_plan, descent, slope }
}
