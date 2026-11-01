//! Where the run writes besides stdout: the trajectory CSV, the opt-in
//! diagnostic CSVs and snapshot directories, and the walker's log lines.
//!
//! `DiagSinks` holds them (opened in `open`); its `impl WalkerDiag` writes
//! the walker's records, `QpDiagWriter` the task stack's QP snapshots, and
//! the per-tick methods the harness's own rows, at the points of the tick
//! they always ran (`transfer` after the step, `before_plant` before the
//! plant is written, `after_plant` after it steps). Moved out of main.rs in
//! S15 of doc/from_claude/20261029_walker_extraction_plan.md, expression for
//! expression, so the bytes are the same.
use crate::{env_f64, flag};
use articara::biped::gait::Support;
use articara::biped::log::{MeasuredContact, Row, measure_contacts};
use articara::biped::rig::{BipedRig, G};
use articara::biped::wbc_stack::TaskStack;
use misa_biped::wbc::WbcStack as _;
use misa_biped::robot::RobotState;
use misa_biped::walker::{self, TickRecord, WalkerStats};
use nalgebra as na;
use std::io::Write;

/// What the per-tick rows read: the tick, the walker's record and the
/// state it assembled, the measured contacts (vertical forces before the
/// step, the finished measurement), and whether the solve was degraded.
#[derive(Clone, Copy)]
pub struct Tick<'a> {
    pub t: f64,
    pub dt: f64,
    pub rec: &'a TickRecord,
    pub st: &'a RobotState,
    pub measured_pre: [f64; 2],
    pub measured: &'a MeasuredContact,
    pub degraded: bool,
    pub total_mass: f64,
}

/// Taken before the plant is written and stepped: the tick audit's
/// pre-state and the push force live on this tick (for the replay to draw;
/// read from the sim's own pulse list, so the arrow cannot disagree with the
/// force the plant saw).
pub struct PreStep {
    audit_pre: Option<Vec<(f64, f64)>>,
    push_now: [f64; 2],
}

/// Where the run writes, besides stdout: TRAJ_CSV (`traj`), and the opt-in
/// diagnostics -- the `*_CSV` files, the QP / MPC snapshot directories and
/// the physics-rate audit. Harness state: the controller writes into some of
/// them on the way, but none of them is read back by it. Opened together in
/// `open`; a file that is not asked for is `None`. The QP_DIAG_DIR settings
/// and their writer ([`QpDiagWriter`]) are handed to the `TaskStack`, which
/// picks the ticks to snapshot and hands them to the writer (and keeps the
/// physical rows for them, or for QP_RUNTIME_CSV's violation column, without
/// changing the solve).
pub struct DiagSinks {
    pub traj: articara::biped::log::TrajLog,
    /// TICK_AUDIT_CSV: per-joint pre/post state around every plant step.
    pub tick_audit: Option<articara::biped::tick_audit::TickAudit>,
    /// QP_DIAG_DIR / QP_CAPTURE_*: the task-stack snapshots (taken by the
    /// `TaskStack` when it is built; `None` without QP_DIAG_DIR).
    pub qp_dump: Option<articara::biped::wbc_stack::QpDump>,
    /// QP_RUNTIME_CSV: per tick, the WBC solve's timing and status.
    pub qp_runtime_log: Option<Csv>,
    /// MPC_RUNTIME_CSV: per MPC solve. The MPC update period and the 5 ms
    /// WBC tick are separate deadlines.
    pub mpc_runtime_log: Option<Csv>,
    /// MOM_DIAG_CSV: opt-in centroidal diagnostic: the arm columns measure
    /// momentum from arm joint motion, while base-motion inertia remains in
    /// the total-minus-arm term.
    pub mom_diag_log: Option<Csv>,
    /// QP_WRENCH_CSV: six-axis instrumentation is additive; writes/conversions
    /// are outside control_ns.
    pub qp_wrench_log: Option<Csv>,
    /// SUBSTEP_LIMIT_AUDIT_JSON: optional physics-rate audit. The normal
    /// trajectory checks only WBC ticks; the simulator trace already contains
    /// q, qdot and applied torque after every MuJoCo step, so no plant or
    /// controller behavior needs to change.
    pub substep_audit_path: Option<String>,
    pub substep_max_excess: Vec<[f64; 3]>,
    pub substep_extrema: Vec<[f64; 4]>,
    pub substep_samples: usize,
    pub substep_nonfinite: usize,
    /// COM_PREVIEW_RUNTIME_CSV: per CoM-preview solve.
    pub com_preview_runtime: Option<Csv>,
    /// TRANSFER_DIAG_CSV: optional P0 diagnostic, sampled entirely before
    /// stepping the plant. A command computed at t is compared with the
    /// observation at t; its physical response must be evaluated at
    /// subsequent observation times.
    pub transfer_diag: Option<Csv>,
    /// ANKLE_CBF_DIAG_CSV: opt-in acceleration-level audit at the right
    /// ankle-roll stop. The CSV samples the state and QP decision in the same
    /// tick; the plant response appears at later samples, so
    /// command-vs-response causality stays clear.
    pub ankle_cbf_diag: Option<Csv>,
    /// PREVIEW_TRAJ_CSV: every accepted LIPM CoM-preview solve (the
    /// bounded-ZMP receding-horizon planner of COM_REPLAN, src/biped/preview.rs),
    /// one row per horizon sample: the planned ZMP, the predicted CoM at the
    /// sample end and the reference ZMP the plan was given. For the summary
    /// videos (MPC_CSV in examples/render_com_squat_video.py).
    pub preview_traj_file: Option<Csv>,
    /// MPC_TRAJ_CSV: one row per wrench-MPC solve with what the MPC
    /// planned -- the CoM it starts from and aims at, its predicted CoM at the
    /// end of every node, the net CoP of node 0 and the per-node net-CoP
    /// reference, each contact's origin and planned normal force, and the
    /// landing variable (foot, shift). For the summary videos (drawn by
    /// examples/render_com_squat_video.py with MPC_CSV). The controller and
    /// TRAJ_CSV are untouched either way.
    pub mpc_traj_file: Option<Csv>,
    /// MPC_DIAG_DIR: every MPC solve's input and config
    /// (`mpc_NNNNNN.json`, numbered by `mpc_diag_count`).
    pub mpc_diag_dir: Option<String>,
    pub mpc_diag_count: usize,
    /// GAIT_DUMP_T="t0,t1": one line per tick with the gait slice the
    /// clock is on against the contact-driven stance set.
    pub gait_dump_t: Option<(f64, f64)>,
    /// CONTACT_DUMP_T="t0,t1": print every MuJoCo contact (bodies, world
    /// position, world force) on the ticks inside the window.
    pub contact_dump_t: Option<(f64, f64)>,
    /// ANKLE_CBF_DIAG_CSV's joint: (misarta joint, velocity row, harness
    /// joint, MuJoCo dof), found at start-up.
    pub ankle_cbf_indices: Option<(usize, usize, usize, usize)>,
}

/// One opt-in diagnostic CSV.
pub type Csv = std::io::BufWriter<std::fs::File>;

/// QP_DIAG_DIR: writes the task stack's snapshots as
/// `<dir>/tick_NNNNNN.json` (schema 2), for offline numerical diagnosis.
/// The stack (misa-biped) picks the ticks and hands over the contents; the
/// JSON is built here (moved from the stack in S13d, expression for
/// expression, so the bytes are the same).
pub struct QpDiagWriter {
    dir: String,
}

impl articara::biped::wbc_stack::QpSnapshotSink for QpDiagWriter {
    fn snapshot(&mut self, s: &articara::biped::wbc_stack::QpSnapshot<'_>) {
        use nalgebra as na;
        let dir = &self.dir;
        let (sol, cfg) = (s.sol, s.solve);
        let matrix = |m: &na::DMatrix<f64>| serde_json::json!({"rows":m.nrows(),"cols":m.ncols(),"data":m.as_slice()});
        let tasks: Vec<_> = s.levels.iter().zip(s.level_names).map(|(task,name)| serde_json::json!({
            "name":name,"a":matrix(&task.a),"b":task.b.as_slice(),"d":matrix(&task.d),"f":task.f.as_slice(),
            "equality_semantics":"lexicographic_least_squares", "inequality_semantics":"hierarchical_slack",
            "row_units":"weighted_task_units", "stored_scale":1.0})).collect();
        let variables: Vec<_> = s.layout.names().map(|name| {
            let var=s.layout.var(name);
            serde_json::json!({"name":name,"offset":var.offset(),"size":var.size(),
                "components":match name {"qddot"=>"base angular rad/s^2, base linear m/s^2, joints rad/s^2", "f"=>"world link-origin [mx,my,mz,fx,fy,fz] per contact", _=>"joint torque N*m"}})
        }).collect();
        let physical = s.physical.as_ref().map(|p| serde_json::json!({
            "eom":{"a":matrix(&p.eom.a),"b":p.eom.b.as_slice(),"units":"base moment N*m, base force N, joint moment N*m"},
            "inequalities":{"d":matrix(p.d),"f":p.f.as_slice(),"units":p.units,"normalization_scales":p.scales,
                "semantics":"physical_hard_despite_hqp_slack","torque_rows":p.torque_rows,"cbf_rows":p.cbf_rows},
            "body_weight_N":p.body_weight,"reference_length_m":p.reference_length}));
        let snapshot = serde_json::json!({"schema_version":2,"t":s.t,"tick":s.tick,"ctrl_dt":s.ctrl_dt,
            "status":format!("{:?}",sol.status),"backend":format!("{:?}",cfg.backend),
            "layer_statuses":sol.level_statuses.iter().map(|s| format!("{s:?}")).collect::<Vec<_>>(),
            "layer_raw_statuses":sol.level_diagnostics.iter().map(|d| d.as_ref().map(|d| &d.raw_status)).collect::<Vec<_>>(),
            "prox_weight":cfg.prox_weight,"tasks":tasks,"solution":sol.x.as_slice(),"variables":variables,
            "contact_ids":s.contact_ids,"planned_support":format!("{:?}",s.support),"phase":"None",
            "measured_fz_N":s.contact_fz,"physical_groups":physical,"solve_ns":s.solve_ns,
            "check_ns":s.check_ns,"normalized_physical_violation":s.violation});
        std::fs::create_dir_all(dir).expect("QP_DIAG_DIR");
        std::fs::write(
            format!("{dir}/tick_{:06}.json", s.index),
            serde_json::to_vec(&snapshot).unwrap(),
        )
        .expect("write QP snapshot");
    }
}

impl DiagSinks {
    /// `traj` and `tick_audit` are made by the caller (TrajLog prints its
    /// path at that point of the start-up log); the rest is opened here from
    /// the environment, each CSV with its header line. Prints nothing.
    pub fn open(
        traj: articara::biped::log::TrajLog,
        tick_audit: Option<articara::biped::tick_audit::TickAudit>,
        substep_audit_path: Option<String>,
        rig: &BipedRig,
    ) -> Self {
        let n_joints = rig.robot.joints.len();
        use std::io::Write;
        let csv = |key: &str, what: &str, header: &dyn Fn(&mut Csv)| -> Option<Csv> {
            std::env::var(key).ok().map(|path| {
                let mut f = std::io::BufWriter::new(std::fs::File::create(path).expect(what));
                header(&mut f);
                f
            })
        };
        Self {
            traj,
            tick_audit,
            qp_dump: {
                let dir = std::env::var("QP_DIAG_DIR").ok();
                let capture_all = flag("QP_CAPTURE_ALL", false);
                let start = env_f64("QP_CAPTURE_START", 0.0);
                let end = env_f64("QP_CAPTURE_END", f64::INFINITY);
                let stride = env_f64("QP_CAPTURE_STRIDE", 1.0).max(1.0) as usize;
                let max = env_f64("QP_CAPTURE_MAX", 20000.0).max(1.0) as usize;
                dir.map(|dir| articara::biped::wbc_stack::QpDump {
                    count: 0,
                    capture_all,
                    start,
                    end,
                    stride,
                    max,
                    sink: Box::new(QpDiagWriter { dir }),
                })
            },
            qp_runtime_log: csv("QP_RUNTIME_CSV", "QP_RUNTIME_CSV", &|f| {
                writeln!(f, "t,solve_ns,check_ns,status,normalized_physical_violation,reset,deadline_miss,control_ns,mpc_age_s,mpc_solves,mpc_failures,prefix_applied,repair_applied,repair_scaled_delta,repair_ns").unwrap();
            }),
            mpc_runtime_log: csv("MPC_RUNTIME_CSV", "MPC_RUNTIME_CSV", &|f| {
                writeln!(f, "t,phase,contact_mask,backend,horizon,update_period_s,solve_ns,deadline_miss,accepted,previous_age_s").unwrap();
            }),
            mom_diag_log: csv("MOM_DIAG_CSV", "MOM_DIAG_CSV", &|f| {
                writeln!(f, "t,h_total_x,h_total_y,h_total_z,h_arm_joint_x,h_arm_joint_y,h_arm_joint_z,h_other_x,h_other_y,h_other_z,fz_measured_l,fz_measured_r,fz_qp_l,fz_qp_r,n_stance").unwrap();
            }),
            qp_wrench_log: csv("QP_WRENCH_CSV", "QP_WRENCH_CSV", &|f| {
                write!(f, "t,side,planned_support,wbc_contact,mpc_valid,planned_valid,wbc_valid,measured_valid,wbc_degraded,mpc_age_s").unwrap();
                for source in ["planned", "mpc", "wbc", "measured"] {
                    for component in ["mx", "my", "mz", "fx", "fy", "fz"] { write!(f, ",{source}_{component}").unwrap(); }
                }
                writeln!(f).unwrap();
            }),
            substep_audit_path,
            substep_max_excess: vec![[0.0_f64; 3]; n_joints],
            substep_extrema: vec![[f64::INFINITY, f64::NEG_INFINITY, 0.0, 0.0]; n_joints],
            substep_samples: 0,
            substep_nonfinite: 0,
            com_preview_runtime: csv("COM_PREVIEW_RUNTIME_CSV", "COM_PREVIEW_RUNTIME_CSV", &|f| {
                writeln!(f, "t,solve_ns,deadline_miss_50ms,accepted,terminal_error_m,applied_lateral,horizon").unwrap();
            }),
            transfer_diag: csv("TRANSFER_DIAG_CSV", "transfer diagnostic CSV", &|f| {
                writeln!(f, "t,dt,bw,phase,high,alpha,stance_mask,measured_mask,mpc_valid,mpc_age,mpc_failed,mpc_ref_l,mpc_ref_r,mpc_fz_l,mpc_fz_r,nominal_world_fz_l,nominal_world_fz_r,qp_world_fz_l,qp_world_fz_r,measured_world_fz_l,measured_world_fz_r,trunk_pitch,trunk_roll,qp_cop_l_x,qp_cop_r_x,measured_cop_l_x,measured_cop_r_x,degraded,ffdc_dz").unwrap();
            }),
            ankle_cbf_diag: csv("ANKLE_CBF_DIAG_CSV", "ankle CBF diagnostic CSV", &|f| {
                writeln!(f, "t,q,v,q_max,cbf_upper,qp_qdd,measured_fz_r,qp_fx_r,qp_fy_r,qp_fz_r,measured_fx_r,measured_fy_r,post_q,post_v,mj_qacc,mj_qfrc_constraint,mj_qfrc_passive,mj_qfrc_actuator,command_tau").unwrap();
            }),
            preview_traj_file: csv("PREVIEW_TRAJ_CSV", "PREVIEW_TRAJ_CSV", &|f| {
                writeln!(f, "t,k,dur,zmp_x,zmp_y,c_x,c_y,rzmp_x,rzmp_y,lo_x,lo_y,hi_x,hi_y").unwrap();
            }),
            mpc_traj_file: csv("MPC_TRAJ_CSV", "MPC_TRAJ_CSV", &|f| {
                let mut h = String::from("t,com_x,com_y,com_z,tgt_x,tgt_y,tgt_z");
                for k in 0..16 {
                    h += &format!(",c{k}_x,c{k}_y,c{k}_z");
                }
                h += ",cop0_x,cop0_y";
                for k in 0..16 {
                    h += &format!(",rcop{k}_x,rcop{k}_y");
                }
                h += ",pl_x,pl_y,pl_z,pr_x,pr_y,pr_z,fz_l,fz_r,on_l,on_r,land_foot,land_dx,land_dy,n_nodes";
                writeln!(f, "{h}").unwrap();
            }),
            mpc_diag_dir: std::env::var("MPC_DIAG_DIR").ok(),
            mpc_diag_count: 0,
            gait_dump_t: std::env::var("GAIT_DUMP_T").ok().and_then(|v| {
                let mut it = v.split(',').map(|x| x.trim().parse::<f64>().ok());
                Some((it.next()??, it.next()??))
            }),
            contact_dump_t: std::env::var("CONTACT_DUMP_T").ok().and_then(|v| {
                let mut it = v.split(',').map(|x| x.trim().parse::<f64>().ok());
                Some((it.next()??, it.next()??))
            }),
            ankle_cbf_indices: rig.actuated().into_iter().find_map(|(ji, vi)| {
                (rig.robot.joints[ji].name == "right_ankle_roll_joint").then(|| {
                    (
                        rig.a2m[ji].expect("ankle misarta joint"),
                        vi,
                        ji,
                        rig.sim
                            .joint_dof_adr("right_ankle_roll_joint")
                            .expect("ankle MuJoCo joint"),
                    )
                })
            }),
        }
    }
}

/// The walker's diagnostic records (`misa_biped::sink::WalkerDiag`), written
/// to the CSV / JSON files above in the formats the control used to write
/// them itself.
impl misa_biped::sink::WalkerDiag for DiagSinks {
    fn preview_solved(
        &mut self,
        t: f64,
        samples: &[articara::biped::preview::PreviewSample],
        plan: &articara::biped::preview::ComPreview,
    ) {
        if let Some(f) = self.preview_traj_file.as_mut() {
            use std::io::Write as _;
            for (k, smp) in samples.iter().enumerate() {
                let (Some(z), Some(c)) = (plan.zmp.get(k), plan.com.get(k)) else { break };
                writeln!(
                    f,
                    "{t:.4},{k},{:.4},{:.5},{:.5},{:.5},{:.5},{:.5},{:.5},{:.5},{:.5},{:.5},{:.5}",
                    smp.duration, z.x, z.y, c.x, c.y, smp.zmp.x, smp.zmp.y,
                    smp.bounds.lo.x, smp.bounds.lo.y, smp.bounds.hi.x, smp.bounds.hi.y
                )
                .unwrap();
            }
        }
    }

    fn preview_runtime(&mut self, t: f64, preview_solve_ns: u128, preview_accepted: bool, terminal_error: f64, com_preview_n: usize) {
        if let Some(writer) = &mut self.com_preview_runtime {
            use std::io::Write as _;
            writeln!(writer, "{t:.6},{preview_solve_ns},{},{},{:.9},{},{}",
                u8::from(preview_solve_ns > 50_000_000),
                u8::from(preview_accepted),
                terminal_error,
                0, // applied_lateral: COM_REPLAN_LATERAL (removed) was off
                com_preview_n,
            ).unwrap();
        }
    }

    fn wants_mpc_input(&self) -> bool {
        self.mpc_diag_dir.is_some()
    }

    fn mpc_input(
        &mut self,
        t: f64,
        dt: f64,
        stair_wrench_dt: f64,
        input: &articara::biped::wrench_mpc::Input,
        config: &articara::biped::wrench_mpc::Config,
    ) {
        let Some(dir) = &self.mpc_diag_dir else { return };
        let contacts: Vec<_> = input.contacts.iter().map(|c| serde_json::json!({
            "position":c.position.as_slice(),"rotation":c.rotation.as_slice(),"cop_half":c.cop_half})).collect();
        // "phase" was the stair transfer's phase (removed); the walk
        // always wrote its placeholder, `Capture`.
        let dump = serde_json::json!({"schema":1,"t":t,"ctrl_dt":dt,"mpc_update_dt":stair_wrench_dt,
            "phase":"Capture","input":{
                "com":input.com.as_slice(),"velocity":input.velocity.as_slice(),"angular_momentum":input.angular_momentum.as_slice(),
                "target_com":input.target_com.as_slice(),"target_velocity":input.target_velocity.as_slice(),"target_acceleration":input.target_acceleration.as_slice(),
                "contacts":contacts,"contact_schedule":input.contact_schedule,"activation_schedule":input.activation_schedule,
                "reference_fz_schedule":input.reference_fz_schedule,"min_fz_schedule":input.min_fz_schedule,
                "previous":[input.previous[0].as_slice(),input.previous[1].as_slice()]},
            "config":{"horizon":config.horizon,"dt":config.dt,"mass":config.mass,"gravity":config.gravity,
                "friction_mu":config.friction_mu,"cop_half":config.cop_half,"torsion_mu":config.torsion_mu,"max_fz":config.max_fz,
                "position_weight":config.position_weight,"velocity_weight":config.velocity_weight,"momentum_weight":config.momentum_weight,
                "acceleration_weight":config.acceleration_weight,"wrench_weight":config.wrench_weight,"smooth_weight":config.smooth_weight,"validate_solution":config.validate_solution}});
        std::fs::create_dir_all(dir).expect("MPC_DIAG_DIR");
        std::fs::write(
            format!("{dir}/mpc_{:06}.json", self.mpc_diag_count),
            serde_json::to_vec(&dump).unwrap(),
        )
        .expect("MPC snapshot");
        self.mpc_diag_count += 1;
    }

    fn mpc_runtime(
        &mut self,
        t: f64,
        mask: u8,
        config: &articara::biped::wrench_mpc::Config,
        stair_wrench_dt: f64,
        mpc_solve_ns: u128,
        accepted: bool,
        previous_age: f64,
    ) {
        if let Some(file) = &mut self.mpc_runtime_log {
            use std::io::Write;
            // Second column: the removed stair transfer's phase,
            // `Capture` being the walk's placeholder.
            writeln!(file, "{t:.9},Capture,{mask},{:?},{},{stair_wrench_dt:.9},{mpc_solve_ns},{},{},{previous_age:.9}",
                config.qp.solver, config.horizon,
                mpc_solve_ns > (stair_wrench_dt * 1e9) as u128,
                accepted).unwrap();
        }
    }

    fn keeps_mpc_frames(&self) -> bool {
        self.qp_wrench_log.is_some()
    }

    fn mpc_solved(
        &mut self,
        t: f64,
        com: &na::Vector3<f64>,
        input: &articara::biped::wrench_mpc::Input,
        solution: &articara::biped::wrench_mpc::Solution,
        net_cop: [f64; 2],
        landing: Option<&articara::biped::wrench_mpc::Landing>,
        by_side: &[articara::biped::wrench_mpc::Wrench; 2],
    ) {
        if let Some(f) = self.mpc_traj_file.as_mut() {
            use std::io::Write as _;
            let nan = f64::NAN;
            let mut row = format!(
                "{t:.4},{:.5},{:.5},{:.5},{:.5},{:.5},{:.5}",
                com.x, com.y, com.z, input.target_com.x, input.target_com.y, input.target_com.z
            );
            for k in 0..16 {
                match solution.trajectory.get(k) {
                    Some(n) => row += &format!(",{:.5},{:.5},{:.5}", n[0].x, n[0].y, n[0].z),
                    None => row += &format!(",{nan},{nan},{nan}"),
                }
            }
            row += &format!(",{:.5},{:.5}", net_cop[0], net_cop[1]);
            for k in 0..16 {
                match input.reference_net_cop_schedule.as_ref().and_then(|v| v.get(k)) {
                    Some(c) => row += &format!(",{:.5},{:.5}", c.x, c.y),
                    None => row += &format!(",{nan},{nan}"),
                }
            }
            for foot in 0..2 {
                let p = input.contacts[foot].position;
                row += &format!(",{:.5},{:.5},{:.5}", p.x, p.y, p.z);
            }
            let on = input.contact_schedule.first().copied().unwrap_or([false; 2]);
            let (lf, ld) = match (&landing, solution.landing_shift) {
                (Some(l), Some(d)) => (l.foot as i64, d),
                (Some(l), None) => (l.foot as i64, na::Vector2::zeros()),
                _ => (-1, na::Vector2::zeros()),
            };
            row += &format!(
                ",{:.3},{:.3},{},{},{lf},{:.5},{:.5},{}",
                by_side[0][5], by_side[1][5], on[0] as u8, on[1] as u8, ld.x, ld.y, solution.trajectory.len()
            );
            writeln!(f, "{row}").unwrap();
        }
    }
}

/// The walker's log lines to this process's stdout / stderr (the run's log
/// is taken with `2>&1`, so the two streams interleave as written).
pub struct StdSink;

impl misa_biped::sink::LogSink for StdSink {
    fn line(&mut self, s: &str) {
        println!("{s}");
    }
    fn err_line(&mut self, s: &str) {
        eprintln!("{s}");
    }
}

/// The trajectory CSV's columns after the common ones (`TrajLog`).
pub const EXTRA_COLS: [&str; 98] = [
    "xi_x",
    "xi_y",
    "xi_ref_x",
    "xi_ref_y",
    "zmp_ref_x",
    "zmp_ref_y",
    "zmp_cmd_x",
    "zmp_cmd_y",
    "zmp_clamp",
    "dcm_err",
    "support",
    "step",
    "share_l",
    "share_r",
    "swing_tgt_x",
    "swing_tgt_y",
    "swing_tgt_z",
    "ankle_roll_l",
    "ankle_roll_r",
    // The disturbance itself, so a replay can DRAW it instead of the
    // viewer having to be told separately what was applied. Zero except
    // while a pulse is live.
    "push_fx",
    "push_fy",
    // Contact with `SPAR_WALL`'s target: this tick's force, the impulse
    // delivered by the punch in progress, and the run total.
    "hit_f",
    "hit_imp_punch",
    "hit_imp_total",
    // Pre-step FK, time-aligned with measured contact forces and goals.
    "sole_l_x",
    "sole_l_y",
    "sole_l_z",
    "sole_r_x",
    "sole_r_y",
    "sole_r_z",
    "anchor_l_y",
    "anchor_r_y",
    "anchor_err_l_y",
    "anchor_err_r_y",
    "foot_v_l_x",
    "foot_v_l_y",
    "foot_v_r_x",
    "foot_v_r_y",
    "contact_activation_l",
    "contact_activation_r",
    "contact_force_activation_l",
    "contact_force_activation_r",
    "load_chatter_ema",
    "goal_l_x",
    "goal_l_y",
    "goal_l_z",
    "goal_r_x",
    "goal_r_y",
    "goal_r_z",
    "com_replan",
    "preview_x",
    "preview_y",
    "preview_vx",
    "preview_vy",
    "preview_terminal_error",
    "preview_solves",
    "preview_failures",
    "landing_delay",
    "stair_transfer",
    "stair_transfer_phase",
    "stair_transfer_alpha",
    "stair_transfer_high",
    "ffdc_dz",
    "ffdc_err",
    "capture_error",
    "capture_tilt",
    "trunk_roll",
    "trunk_pitch",
    "trunk_cmd_pitch",
    "trunk_acc_pitch",
    "trunk_cmd_roll",
    "trunk_acc_roll",
    "transfer_guard",
    "wrench_mpc",
    "wrench_mpc_fz_l",
    "wrench_mpc_fz_r",
    "mpc_cop_x",
    "mpc_cop_y",
    "mpc_fs_x",
    "mpc_fs_y",
    "wbc_cop_x",
    "wbc_cop_y",
    "wrench_mpc_solves",
    "wrench_mpc_failures",
    "swing_pos_err_xy",
    "swing_pos_err_z",
    "swing_task_residual_rms",
    "swing_res_z",
    "swing_res_rows",
    "swing_cmd_acc_z",
    "swing_achieved_acc_z",
    "tau_use_max",
    "tau_saturated_count",
    // Leg reach (src/biped/reach.rs): |hip - sole| / leg length, per side,
    // and the hip heights the bound is computed from. Zero when the
    // profile has no measured leg. Logged unconditionally because every
    // reach CLAIM about a fall is worthless without it.
    "reach_l",
    "reach_r",
    "hip_l_z",
    "hip_r_z",
    "stride_max",
];

impl DiagSinks {
    /// CONTACT_DUMP_T's lines for this tick (diagnostic only; unset =
    /// nothing).
    pub fn contact_dump(&self, rig: &BipedRig, t: f64) {
        // CONTACT_DUMP_T="t0,t1": print every MuJoCo contact (bodies, world
        // position, world force) on the ticks inside the window. Diagnostic
        // only; unset = nothing.
        if let Some((d0, d1)) = self.contact_dump_t
            && t >= d0
            && t <= d1
        {
            for c in rig.sim.contacts() {
                println!(
                    "  [con] t={t:.3} {} <-> {} pos=({:+.4},{:+.4},{:+.4}) f=({:+.1},{:+.1},{:+.1})",
                    if c.body1.is_empty() { "world" } else { c.body1.as_str() },
                    if c.body2.is_empty() { "world" } else { c.body2.as_str() },
                    c.pos[0], c.pos[1], c.pos[2], c.force_world[0], c.force_world[1], c.force_world[2]
                );
            }
        }
    }

    /// TRANSFER_DIAG_CSV's row, right after the step.
    pub fn transfer(&mut self, k: &Tick, rig: &BipedRig, stats: &WalkerStats) {
        let Tick { t, dt, rec, st, measured_pre, measured: measured_c, degraded, total_mass } = *k;
        let data = &st.data;
        if let Some(file) = self.transfer_diag.as_mut() {
            let mut nominal = [0.0; 2];
            let mut mask = 0u8;
            for (slot, &side) in rec.stance_sides.iter().enumerate() {
                mask |= 1 << side;
                // Native WBC force blocks use world axes (as f_qp_w above).
                nominal[side] = rec.forces_nominal[6 * slot + 5];
            }
            let bw = total_mass * G;
            // Load threshold is evidence of support, not a contact detector.
            let measured_mask = u8::from(measured_pre[0] > 0.05 * bw)
                | (u8::from(measured_pre[1] > 0.05 * bw) << 1);
            let rot = misarta::se3::rotation_matrix(&data.oMi[rig.trunk_mi]);
            let mpc_valid = rec.wrench_mpc_command.is_some();
            let values = [
                t,
                dt,
                bw,
                // phase / high / alpha of the removed stair transfer (idle).
                0.0,
                -1.0,
                0.0,
                mask as f64,
                measured_mask as f64,
                f64::from(mpc_valid),
                if mpc_valid {
                    rec.diag_mpc_time.map_or(f64::NAN, |ts| t - ts)
                } else {
                    f64::NAN
                },
                stats.wrench_mpc_failures as f64,
                if mpc_valid {
                    rec.diag_mpc_reference[0]
                } else {
                    f64::NAN
                },
                if mpc_valid {
                    rec.diag_mpc_reference[1]
                } else {
                    f64::NAN
                },
                rec.wrench_mpc_command.as_ref().map_or(f64::NAN, |w| w[0][5]),
                rec.wrench_mpc_command.as_ref().map_or(f64::NAN, |w| w[1][5]),
                nominal[0],
                nominal[1],
                rec.f_qp_w[0][2],
                rec.f_qp_w[1][2],
                measured_pre[0],
                measured_pre[1],
                (-rot[(2, 0)]).clamp(-1.0, 1.0).asin(),
                rot[(2, 1)].atan2(rot[(2, 2)]),
                rec.cop_qp[0][0],
                rec.cop_qp[1][0],
                measured_c.cop[0][0],
                measured_c.cop[1][0],
                f64::from(degraded),
                0.0, // ffdc_dz: the removed FFDC loops left it at 0
            ];
            for (i, value) in values.iter().enumerate() {
                if i > 0 {
                    write!(file, ",").unwrap();
                }
                write!(file, "{value:.9}").unwrap();
            }
            writeln!(file).unwrap();
        }
    }

    /// Before the plant is written: what the disturbance is doing now (for
    /// the replay), the tick audit's pre-state, and the QP_RUNTIME_CSV /
    /// QP_WRENCH_CSV rows (about the sole as the plant has it now).
    pub fn before_plant(
        &mut self,
        k: &Tick,
        rig: &BipedRig,
        stats: &WalkerStats,
        sol: &misa_wbc::Solution,
        extracted: &misa_wbc::Extracted,
        push_link: &str,
        qp_control_start: std::time::Instant,
    ) -> PreStep {
        let Tick { t, dt, rec, measured: measured_c, degraded, .. } = *k;
        // WBC_REPAIR_ORIGINAL / WBC_PREFIX_MIN_LEVEL (repair the failed QP,
        // or command a checked prefix) were experiments and are removed;
        // QP_RUNTIME_CSV keeps their columns at the off values.
        let prefix_applied = false;
        let repair_applied = false;
        let repair_scaled_delta = 0.0_f64;
        let repair_ns = 0u128;
        let push_now = rig
            .sim
            .external_force_pulses()
            .iter()
            .find(|p| p.link_name == push_link)
            .map(|p| [p.force[0], p.force[1]])
            .unwrap_or([0.0, 0.0]);
        let audit_pre = self.tick_audit.as_ref().map(|audit| audit.sample(rig));
        if let Some(file) = &mut self.qp_runtime_log {
            use std::io::Write;
            let control_ns = qp_control_start
                .elapsed()
                .as_nanos()
                .saturating_sub(rec.qp_capture_ns)
                .saturating_sub(rec.mpc_capture_ns);
            writeln!(file, "{t:.9},{qp_solve_ns},{qp_check_ns},\"{:?}\",{qp_physical_violation:.12e},{qp_reset},{},{control_ns},{},{wrench_mpc_solves},{wrench_mpc_failures},{prefix_applied},{repair_applied},{repair_scaled_delta:.12e},{repair_ns}", sol.status,
                control_ns > (dt * 1e9) as u128, rec.diag_mpc_time.map_or(-1.0, |stamp| t-stamp),
                wrench_mpc_solves = stats.wrench_mpc_solves, wrench_mpc_failures = stats.wrench_mpc_failures,
                qp_solve_ns = rec.qp_solve_ns, qp_check_ns = rec.qp_check_ns,
                qp_physical_violation = rec.qp_physical_violation, qp_reset = rec.qp_reset).unwrap();
        }
        if let Some(file) = &mut self.qp_wrench_log {
            use articara::biped::log::contact_wrench_in_sole;
            use std::io::Write;
            // All four streams are expressed about the same current physical sole
            // centre and actual MuJoCo foot axes, before the plant step. MPC is
            // the held forecast at its solve-time patch, shifted/rotated here.
            for side in 0..2 {
                let link = rig.prof.foot_links[side];
                let pose = rig
                    .sim
                    .body_world_position(link)
                    .zip(rig.sim.body_world_orientation(link));
                let mut planned = na::SVector::<f64, 6>::zeros();
                let mut wbc = na::SVector::<f64, 6>::zeros();
                let mut mpc = na::SVector::<f64, 6>::repeat(f64::NAN);
                let slot = rec.stance_sides.iter().position(|s| *s == side);
                if let Some((position, orientation)) = &pose {
                    let rotation = orientation.to_rotation_matrix().into_inner();
                    let origin = na::Vector3::from_column_slice(position);
                    let sole = origin
                        + rotation
                            * na::Vector3::new(rig.prof.sole_centre_x, 0.0, -rig.prof.sole_below_origin);
                    if let Some(slot) = slot {
                        let force = na::Vector3::from_column_slice(
                            &extracted.forces.as_slice()[6 * slot + 3..6 * slot + 6],
                        );
                        let moment = na::Vector3::from_column_slice(
                            &extracted.forces.as_slice()[6 * slot..6 * slot + 3],
                        );
                        wbc = contact_wrench_in_sole(&origin, &force, &moment, &sole, &rotation);
                        let force = na::Vector3::from_column_slice(
                            &rec.forces_nominal.as_slice()[6 * slot + 3..6 * slot + 6],
                        );
                        let moment = na::Vector3::from_column_slice(
                            &rec.forces_nominal.as_slice()[6 * slot..6 * slot + 3],
                        );
                        planned =
                            contact_wrench_in_sole(&origin, &force, &moment, &sole, &rotation);
                    }
                    if let (Some(command), Some(frames)) = (&rec.wrench_mpc_command, &rec.wrench_mpc_frames)
                    {
                        let source = &frames[side];
                        let force = source.rotation * command[side].fixed_rows::<3>(3);
                        let moment = source.rotation * command[side].fixed_rows::<3>(0);
                        mpc = contact_wrench_in_sole(
                            &source.position,
                            &force,
                            &moment,
                            &sole,
                            &rotation,
                        );
                    }
                }
                let valid =
                    |w: &na::SVector<f64, 6>| pose.is_some() && w.iter().all(|v| v.is_finite());
                let planned_support = match rec.support {
                    Support::Double => true,
                    Support::Single { stance, .. } => stance == side,
                    Support::Flight { .. } => false,
                };
                write!(
                    file,
                    "{t:.9},{side},{planned_support},{},{},{},{},{},{degraded},{}",
                    slot.is_some(),
                    valid(&mpc),
                    valid(&planned),
                    valid(&wbc),
                    measured_c.sole_wrench_valid[side],
                    rec.diag_mpc_time.map_or(-1.0, |stamp| t - stamp)
                )
                .unwrap();
                for wrench in [
                    planned.as_slice(),
                    mpc.as_slice(),
                    wbc.as_slice(),
                    &measured_c.sole_wrench[side],
                ] {
                    for component in wrench {
                        write!(file, ",{component:.12e}").unwrap();
                    }
                }
                writeln!(file).unwrap();
            }
        }
        PreStep { audit_pre, push_now }
    }

    /// After the plant step: the substep audit, the tick audit's row, the
    /// ankle CBF row, and the trajectory CSV's row (with MOM_DIAG_CSV's and
    /// the GAIT_DUMP_T line), in the order they always ran.
    #[allow(clippy::too_many_arguments)]
    pub fn after_plant(
        &mut self,
        k: &Tick,
        rig: &BipedRig,
        walker: &walker::Walker<TaskStack>,
        pre: &PreStep,
        extracted: &misa_wbc::Extracted,
        robot_taus: &[f64],
        mj_substeps: u32,
        jlim_cfg: &misa_wbc::tasks::JointLimitCbf,
        friction_mu: f64,
        ankle_roll: [f64; 2],
        hit: [f64; 3],
    ) {
        let Tick { t, rec, st, measured_pre, measured, degraded, .. } = *k;
        let measured_c = measured;
        let (q, v, v_dvec, data, com) = (&st.q, &st.v, &st.v_dvec, &st.data, st.com);
        let (sole_half_l, sole_half_w) = rig.prof.cop_half;
        let cop_frac = walker.config().dcm.cop_frac;
        let mut tau_use_max = 0.0_f64;
        let mut tau_saturated_count = 0usize;
        for (ji, vi) in rig.actuated() {
            let ratio = robot_taus[ji].abs() / rig.torque_max[vi - 6].max(1e-9);
            tau_use_max = tau_use_max.max(ratio);
            tau_saturated_count += usize::from(ratio >= 0.98);
        }
        let trot = misarta::se3::rotation_matrix(&data.oMi[rig.trunk_mi]);
        let trunk_tilt = trot[(2, 1)]
            .atan2(trot[(2, 2)])
            .abs()
            .max((-trot[(2, 0)]).asin().abs());
        let trunk_tilt_roll = trot[(2, 1)].atan2(trot[(2, 2)]);
        let trunk_tilt_pitch = (-trot[(2, 0)]).asin();
        if self.substep_audit_path.is_some() {
            let tail = rig.sim.trace_len().saturating_sub(mj_substeps as usize);
            for frame in rig.sim.trace().skip(tail) {
                self.substep_samples += 1;
                for (ji, joint) in rig.robot.joints.iter().enumerate() {
                    if joint.joint_type != "revolute" { continue; }
                    let (q, qd, tau) = (frame.q[ji], frame.qvel[ji], frame.tau[ji]);
                    if !q.is_finite() || !qd.is_finite() || !tau.is_finite() {
                        self.substep_nonfinite += 1;
                        continue;
                    }
                    let extrema = &mut self.substep_extrema[ji];
                    extrema[0] = extrema[0].min(q);
                    extrema[1] = extrema[1].max(q);
                    extrema[2] = extrema[2].max(qd.abs());
                    extrema[3] = extrema[3].max(tau.abs());
                    let excess = &mut self.substep_max_excess[ji];
                    excess[0] = excess[0].max((joint.lower - q).max(q - joint.upper).max(0.0));
                    excess[1] = excess[1].max((qd.abs() - joint.velocity).max(0.0));
                    excess[2] = excess[2].max((tau.abs() - joint.effort).max(0.0));
                }
            }
        }
        if let (Some(audit), Some(pre)) = (self.tick_audit.as_mut(), pre.audit_pre.as_ref()) {
            // step() refreshes MuJoCo with forward(): these are post-state
            // forces. Only f_w is used, independent of stale `data` CoP.
            let last = measure_contacts(rig, data, friction_mu);
            audit.write(
                rig,
                t,
                mj_substeps as f64 * rig.sim.timestep(),
                pre,
                &robot_taus,
                measured_pre,
                [rec.f_qp_w[0][2], rec.f_qp_w[1][2]],
                [last.f_w[0][2], last.f_w[1][2]],
            );
        }

        if let (Some(file), Some((mi, vi, ji, mjvi))) = (self.ankle_cbf_diag.as_mut(), self.ankle_cbf_indices)
        {
            let row = vi - 6;
            let position = q[rig.model.q_idx[mi]];
            let velocity = v[vi];
            let a1 = jlim_cfg.alpha1[row];
            let a2 = jlim_cfg.alpha2[row];
            let upper = -(a1 + a2) * velocity + a1 * a2 * (jlim_cfg.q_max[row] - position);
            let (post_q, post_v) = rig
                .sim
                .joint_q_qd("right_ankle_roll_joint")
                .expect("ankle joint state after stepping");
            // MuJoCo force and acceleration entries are from the final physics
            // substep under this tick's held command. Pre/post velocity spans
            // the complete control interval instead.
            writeln!(file,
                "{t:.6},{position:.9},{velocity:.9},{:.9},{upper:.9},{:.9},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{post_q:.9},{post_v:.9},{:.9},{:.9},{:.9},{:.9},{:.9}",
                jlim_cfg.q_max[row], extracted.qddot[vi], measured_pre[1],
                rec.f_qp_w[1][0], rec.f_qp_w[1][1], rec.f_qp_w[1][2],
                measured_c.f_w[1][0], measured_c.f_w[1][1],
                rig.sim.qacc()[mjvi], rig.sim.qfrc_constraint()[mjvi],
                rig.sim.qfrc_passive()[mjvi], rig.sim.qfrc_actuator()[mjvi], robot_taus[ji],
            ).unwrap();
        }

        if self.traj.is_enabled() {
            let foot_z = rig.sim.body_world_position(rig.prof.foot_links[0]).unwrap()[2];
            let foot_vz = rig
                .sim
                .body_world_linear_velocity(rig.prof.foot_links[0])
                .map(|v| v[2])
                .unwrap_or(0.0);
            let swing_z = rig.sim.body_world_position(rig.prof.foot_links[1]).unwrap()[2];
            let support_code = match rec.support {
                Support::Double => 0.0,
                Support::Single { stance, .. } => 1.0 + stance as f64,
                Support::Flight { .. } => 3.0,
            };
            let actual_feet = rec.actual_feet;
            let [al, ar] = actual_feet.sole;
            let [gl, gr] = rec.steps.sole;
            let link_l = misarta::se3::translation(&data.oMi[rig.foot_mi[0]]);
            let link_r = misarta::se3::translation(&data.oMi[rig.foot_mi[1]]);
            let anchor_l_y = walker.stack().anchor(0).map_or(f64::NAN, |(p, _)| p.y);
            let anchor_r_y = walker.stack().anchor(1).map_or(f64::NAN, |(p, _)| p.y);
            let foot_v_l = rig
                .sim
                .body_world_linear_velocity(rig.prof.foot_links[0])
                .unwrap_or([0.0; 3]);
            let foot_v_r = rig
                .sim
                .body_world_linear_velocity(rig.prof.foot_links[1])
                .unwrap_or([0.0; 3]);
            let extra = [
                rec.xi.x,
                rec.xi.y,
                rec.r.xi.x,
                rec.r.xi.y,
                rec.r.p.x,
                rec.r.p.y,
                rec.p_cmd.x,
                rec.p_cmd.y,
                rec.clamped,
                rec.dcm_err,
                support_code,
                rec.step_idx as f64,
                *rec.shares.first().unwrap_or(&0.0),
                *rec.shares.get(1).unwrap_or(&0.0),
                rec.swing_tgt.x,
                rec.swing_tgt.y,
                rec.swing_tgt.z,
                ankle_roll[0],
                ankle_roll[1],
                pre.push_now[0],
                pre.push_now[1],
                hit[0],
                hit[1],
                hit[2],
                al.x,
                al.y,
                al.z,
                ar.x,
                ar.y,
                ar.z,
                anchor_l_y,
                anchor_r_y,
                anchor_l_y - link_l.y,
                anchor_r_y - link_r.y,
                foot_v_l[0],
                foot_v_l[1],
                foot_v_r[0],
                foot_v_r[1],
                // contact_activation_l/r, contact_force_activation_l/r,
                // load_chatter_ema: written only by the removed stair
                // transfer; these are the values it left them at.
                1.0,
                1.0,
                1.0,
                1.0,
                0.0,
                gl.x,
                gl.y,
                gl.z,
                gr.x,
                gr.y,
                gr.z,
                if rec.replan_engaged { 1.0 } else { 0.0 },
                rec.preview_c.x,
                rec.preview_c.y,
                rec.preview_v.x,
                rec.preview_v.y,
                walker.stats().preview_error,
                walker.stats().preview_count as f64,
                walker.stats().preview_failures as f64,
                walker.stats().landing_delay,
                // stair_transfer / _phase / _alpha / _high: the removed
                // three-phase transfer's columns, kept at its idle values so
                // the column layout does not move.
                0.0,
                0.0,
                0.0,
                -1.0,
                0.0, // ffdc_dz (removed DS_FFDC / STAIR_FFDC)
                0.0, // ffdc_err (removed DS_FFDC / STAIR_FFDC)
                -1.0, // capture_error (removed stair transfer)
                0.0,  // capture_tilt (removed stair transfer)
                trunk_tilt_roll,
                trunk_tilt_pitch,
                rec.trunk_cmd_rp[1],
                rec.trunk_acc_rp[1],
                rec.trunk_cmd_rp[0],
                rec.trunk_acc_rp[0],
                -1.0, // transfer_guard (removed stair transfer)
                if rec.wrench_mpc_command.is_some() {
                    1.0
                } else {
                    0.0
                },
                rec.wrench_mpc_command.as_ref().map_or(0.0, |w| w[0][5]),
                rec.wrench_mpc_command.as_ref().map_or(0.0, |w| w[1][5]),
                rec.diag_mpc_net_cop[0],
                rec.diag_mpc_net_cop[1],
                rec.mpc_fs_last.x,
                rec.mpc_fs_last.y,
                rec.diag_wbc_net_cop[0],
                rec.diag_wbc_net_cop[1],
                walker.stats().wrench_mpc_solves as f64,
                walker.stats().wrench_mpc_failures as f64,
                rec.swing_pos_err_xy,
                rec.swing_pos_err_z,
                rec.swing_task_residual_rms,
                rec.swing_res_z,
                rec.swing_res_rows,
                rec.swing_cmd_acc_z,
                rec.swing_achieved_acc_z,
                tau_use_max,
                tau_saturated_count as f64,
                rec.reach_used[0],
                rec.reach_used[1],
                rec.hip_z[0],
                rec.hip_z[1],
                rec.stride_max_now,
            ];
            if let Some(file) = &mut self.mom_diag_log {
                use std::io::Write;
                let ag = walker.robot().cmm(st);
                let h = ag * v_dvec;
                let mut h_arm = na::DVector::zeros(h.len());
                for (ji, vi) in rig.actuated() {
                    let name = &rig.robot.joints[ji].name;
                    if name.contains("shoulder") || name.contains("elbow") || name.contains("wrist")
                    {
                        h_arm += ag.column(vi) * v_dvec[vi];
                    }
                }
                writeln!(file, "{t:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{}",
                    h[0],h[1],h[2],h_arm[0],h_arm[1],h_arm[2],
                    h[0]-h_arm[0],h[1]-h_arm[1],h[2]-h_arm[2],
                    measured.cop[0][2],measured.cop[1][2],rec.cop_qp[0][2],rec.cop_qp[1][2],rec.nc).unwrap();
            }
            // GAIT_DUMP_T="t0,t1": one line per tick with the gait slice the
            // clock is on against the contact-driven stance set.
            if let Some((d0, d1)) = self.gait_dump_t
                && t >= d0
                && t <= d1
            {
                let i = walker.plan().gait().index_at(t);
                let sl = &walker.plan().gait().slices[i];
                println!(
                    "  [gait] t={t:.3} slice {i} {:?} [{:.3},{:.3}] stance={:?} fz=({:.0},{:.0}) xi.x={:.3} ref.x={:.3}",
                    sl.support, sl.t0, sl.t1, rec.stance_sides, rec.fz_meas[0], rec.fz_meas[1], rec.xi.x, rec.r.xi.x
                );
            }
            let row = Row {
                t,
                com,
                com_ref_z: rec.z_ref,
                // The renderer's `com_ref_y` panel is the lateral reference;
                // for a walk that is the DCM target, not a fixed point.
                com_ref_y: rec.r.xi.y,
                tilt: rec.roll.abs().max(rec.pitch.abs()),
                trunk_tilt,
                n_stance: rec.nc,
                foot_z,
                swing_z,
                foot_vz,
                acc_dbg: rec.acc_dbg,
                a_com: rec.a_com,
                rp_ref: rec.rp_ref,
                degraded,
                cop_box: (sole_half_l * cop_frac, sole_half_w * cop_frac),
                cop_qp: rec.cop_qp,
                f_qp_w: rec.f_qp_w,
                measured,
                taus: robot_taus,
                extra: &extra,
            };
            self.traj.write(rig, &row);
        }
    }

    /// SUBSTEP_LIMIT_AUDIT_JSON, after the run.
    pub fn finish(&self, rig: &BipedRig, n_ticks: usize, mj_substeps: u32, mj_dt: f64) {
        if let Some(path) = &self.substep_audit_path {
            let joints: Vec<_> = rig.robot.joints.iter().enumerate()
                .filter(|(_, joint)| joint.joint_type == "revolute")
                .map(|(ji, joint)| serde_json::json!({
                    "name": joint.name, "limits": {"lower_rad": joint.lower, "upper_rad": joint.upper,
                        "velocity_rad_s": joint.velocity, "effort_nm": joint.effort},
                    "observed": {"min_q_rad": self.substep_extrema[ji][0], "max_q_rad": self.substep_extrema[ji][1],
                        "max_abs_qd_rad_s": self.substep_extrema[ji][2], "max_abs_tau_nm": self.substep_extrema[ji][3]},
                    "excess": {"position_rad": self.substep_max_excess[ji][0],
                        "velocity_rad_s": self.substep_max_excess[ji][1], "effort_nm": self.substep_max_excess[ji][2]}
                })).collect();
            let passed = self.substep_nonfinite == 0 && self.substep_samples == n_ticks * mj_substeps as usize
                && rig.robot.joints.iter().enumerate()
                    .filter(|(_, joint)| joint.joint_type == "revolute")
                    .all(|(ji, _)| self.substep_max_excess[ji] == [0.0; 3]);
            let report = serde_json::json!({"schema": 1, "scope": "MuJoCo physics-step trace after each step",
                "physics_dt_s": mj_dt, "expected_samples": n_ticks * mj_substeps as usize,
                "samples": self.substep_samples, "nonfinite_values": self.substep_nonfinite,
                "passed": passed, "joints": joints});
            std::fs::write(path, serde_json::to_vec_pretty(&report).unwrap()).expect("SUBSTEP_LIMIT_AUDIT_JSON");
        }
    }
}
