//! kyo46rs squat/balance on a **centroidal** formulation.
//!
//! Successor to kyo46rs_squat.rs. That file regulates the BASE's
//! roll/pitch/height with separate PD tasks on `qddot[0..6]`, and it
//! plateaus at ~1.1 s before toppling: attitude and height end up
//! competing for the same limited contact authority (raise the attitude
//! gain and tilt is held but z collapses; lower it and z holds but it
//! tips), and nothing at all regulates the horizontal position, so the
//! QP is free to walk all 65 N onto one foot and roll off it.
//!
//! Base attitude is the wrong thing to regulate. What decides whether a
//! biped falls is the CoM (equivalently the ZMP / capture point): the
//! base's 6 DOF are unactuated, so the only handle on the CoM is the
//! contact wrench, bounded by unilaterality, friction and the CoP box.
//! So the task here is the CoM acceleration itself -- and the squat is
//! just a reference on its z component, which makes balance and squat
//! ONE task instead of two that fight.
//!
//! Two conventions worth stating, because both bit the predecessor:
//!
//! - The CoM Jacobian is assembled in the WORLD frame (misarta's
//!   `compute_joint_jacobian` is world-frame with row order
//!   `[angular(3); linear(3)]`), so the CoM task needs no body-frame
//!   conversion at all. That also sidesteps kyo46rs_squat.rs's
//!   unexplained attitude sign inversion, which only ever appeared on
//!   the body-frame `qddot[0..2]` rows.
//! - Trunk orientation likewise uses the trunk's world-frame angular
//!   Jacobian rather than raw `qddot[0..2]`, for the same reason.
//!
//! `J_com` is verified numerically every tick against a finite-difference
//! of the measured CoM (`COMCHK=1`) rather than trusted.
//!
//! Run with: `cargo run --features mujoco --example kyo46rs_com_squat`


/// Everything in this controller that is a property of the ROBOT rather than
/// of the control law. It was all inlined for kyo46rs, which is exactly how
/// `WbcPipeline` ended up unusable for anything but a quadruped -- the leg
/// count was spelled `[String; 4]` in a dozen places. Adding a second machine
/// is the moment to pay that off, not after.
#[cfg(feature = "mujoco")]
struct Profile {
    name: &'static str,
    urdf: &'static str,
    root_link: &'static str,
    foot_links: [&'static str; 2],
    /// hip_pitch / knee / ankle_pitch, per side, for the crouch seed.
    sagittal: [[&'static str; 3]; 2],
    hip_roll: [&'static str; 2],
    /// Sole plane, in the foot link's own frame: how far below the origin it
    /// sits, and where its centre is fore/aft. MUST match the URDF -- the CoP
    /// box is described in this frame and a wrong centre silently describes a
    /// footprint the robot does not have.
    sole_below_origin: f64,
    sole_centre_x: f64,
    /// CoP box half-extents (fore/aft, lateral).
    cop_half: (f64, f64),
    /// Height to drop the model from while measuring where the soles land.
    probe_z: f64,
    /// Crouch seed: knee angle. hip_pitch and ankle_pitch are -knee/2 so the
    /// three sum to zero and the sole stays parallel to the floor.
    knee_seed: f64,
    /// Joints written to the trajectory CSV, in order.
    log_joints: &'static [&'static str],
    /// Burn-in position PD, and the rotor inertia / viscous damping added to
    /// the WBC's M and h. All four scale with the machine: a gain sized for
    /// a 6.6 kg robot with 6 N*m joints does nothing to a 34 kg one with
    /// 139 N*m knees, and armature copied from the wrong motor puts a
    /// systematic error on every actuated row of the mass matrix.
    /// kv must stay under 2*I/dt -- the plant's joint PD is explicit.
    burnin_kp: f64,
    burnin_kv: f64,
    armature: f64,
    joint_damping: f64,
}

#[cfg(feature = "mujoco")]
const KYO46RS: Profile = Profile {
    name: "kyo46rs",
    urdf: "/home/takara/work/dp/humanoid/kyo46rs_description/urdf/kyo46rs.urdf",
    root_link: "torso",
    foot_links: ["left_foot_link", "right_foot_link"],
    sagittal: [
        ["left_hip_pitch_joint", "left_knee_joint", "left_ankle_pitch_joint"],
        ["right_hip_pitch_joint", "right_knee_joint", "right_ankle_pitch_joint"],
    ],
    hip_roll: ["left_hip_roll_joint", "right_hip_roll_joint"],
    sole_below_origin: 0.035,
    sole_centre_x: 0.0,
    cop_half: (0.049, 0.019),
    probe_z: 0.47,
    knee_seed: 0.70,
    burnin_kp: 150.0,
    burnin_kv: 2.0,
    armature: 0.0005,
    joint_damping: 0.15,
    log_joints: &[
        "left_hip_yaw_joint", "left_hip_roll_joint", "left_hip_pitch_joint",
        "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
        "right_hip_yaw_joint", "right_hip_roll_joint", "right_hip_pitch_joint",
        "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
        "left_shoulder_pitch_joint", "left_elbow_joint",
        "right_shoulder_pitch_joint", "right_elbow_joint",
    ],
};

/// Unitree G1, 23-DOF variant. 34.13 kg against kyo46rs's 6.64, and every
/// torque limit is a real per-joint number (knee 139, ankle 35, hip 88)
/// rather than an estimate, which is the point of running it: it separates
/// "the control law is fragile" from "the model is fragile".
///
/// Foot: four 5 mm contact spheres at the corners of a 170 x 60 mm footprint,
/// 35 mm below the ankle_roll origin, and the patch is NOT centred on the
/// ankle -- it runs -50..+120 mm fore/aft, so its centre is 35 mm forward.
#[cfg(feature = "mujoco")]
const G1_23DOF: Profile = Profile {
    name: "g1_23dof",
    urdf: "/home/takara/work/dp/articara/models/unitree_g1_src/robots/g1_description/g1_23dof.urdf",
    root_link: "pelvis",
    foot_links: ["left_ankle_roll_link", "right_ankle_roll_link"],
    sagittal: [
        ["left_hip_pitch_joint", "left_knee_joint", "left_ankle_pitch_joint"],
        ["right_hip_pitch_joint", "right_knee_joint", "right_ankle_pitch_joint"],
    ],
    hip_roll: ["left_hip_roll_joint", "right_hip_roll_joint"],
    sole_below_origin: 0.035,
    sole_centre_x: 0.035,
    cop_half: (0.085, 0.030),
    probe_z: 0.90,
    knee_seed: 0.70,
    // Sized off the torque limits: G1's knee is 139 N*m against kyo46rs's 12,
    // and 34 kg against 6.6. The URDF declares no damping, friction or
    // armature at all, so these are engineering placeholders, not data --
    // flagged here because the same gap on kyo46rs cost a day.
    burnin_kp: 2000.0,
    burnin_kv: 20.0,
    armature: 0.01,
    joint_damping: 1.0,
    log_joints: &[
        "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
        "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
        "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
        "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
        "left_shoulder_pitch_joint", "left_elbow_joint",
        "right_shoulder_pitch_joint", "right_elbow_joint",
    ],
};

#[cfg(feature = "mujoco")]
fn main() {
    use articara::mjcf::{GroundPlaneCfg, MjcfExportOptions};
    use articara::mujoco_sim::MujocoSim;
    use articara::rbd::model::ActuatorMode;
    use articara::robot::RobotModel;
    use articara::wbc_pipeline::build_floating_base_model;
    use misa_wbc::{tasks, AsAffine, Dynamics, Formulation, SolveConfig, Solver};
    use nalgebra as na;
    use std::f64::consts::PI;

    let env_f64 = |k: &str, d: f64| -> f64 {
        std::env::var(k).ok().and_then(|s| s.parse().ok()).unwrap_or(d)
    };
    let flag = |k: &str, d: bool| -> bool {
        std::env::var(k).map(|v| v != "0").unwrap_or(d)
    };

    let prof = match std::env::var("ROBOT").unwrap_or_default().as_str() {
        "" | "kyo46rs" => KYO46RS,
        "g1" | "g1_23dof" => G1_23DOF,
        other => panic!("unknown ROBOT={other:?} (kyo46rs | g1)"),
    };
    println!("robot: {}", prof.name);

    // ── Crouch seed (hip+knee+ankle must sum to 0 for a flat sole) ─────
    let knee_q = env_f64("KNEE", prof.knee_seed);
    let hip_p = env_f64("HIP_PITCH", -knee_q / 2.0);
    let ankle_p = env_f64("ANKLE_PITCH", -knee_q / 2.0);

    let urdf_path = std::path::Path::new(prof.urdf);
    let mut robot = RobotModel::from_urdf(urdf_path)
        .unwrap_or_else(|e| panic!("load {}: {e}", prof.urdf));
    let crouch: Vec<(&str, f64)> = prof
        .sagittal
        .iter()
        .flat_map(|[h, k, a]| [(*h, hip_p), (*k, knee_q), (*a, ankle_p)])
        .collect();
    for (name, q) in crouch.iter().copied() {
        if let Some(&ji) = robot.joint_map.get(name) {
            robot.joint_positions[ji] = q;
        }
    }
    robot.rebuild_misarta_model();
    let q_seed: Vec<f64> = robot.joint_positions.clone();

    // EL05 placeholders + burn-in PD. kv must stay <= ~2 at dt=1 ms: the
    // per-joint PD is explicit velocity feedback, stable only while
    // kv < 2*I/dt, and the distal roll joints have I ~ 6e-4 kg*m^2.
    // Measured threshold (kyo46rs_stand_check.rs, position control only):
    // kv <= 2.0 stands 5 s, kv = 3.0 collapses at 0.65 s.
    let joint_damping: f64 = env_f64("JOINT_DAMPING", prof.joint_damping);
    let armature: f64 = env_f64("ARMATURE", prof.armature);
    // Scale every actuator limit, to separate "the foot is too small" from
    // "the motors are too weak" as the cause of a level-0 infeasibility.
    let torque_scale = env_f64("TORQUE_SCALE", 1.0);
    // Baumgarte stabilisation on the contact constraint. zero_contact_
    // acceleration pins J*qddot + Jdot*v = 0, which is an ACCELERATION
    // constraint: give the foot any angular velocity and its orientation
    // drifts at that rate forever, because nothing feeds the pose error
    // back. Invisible in a symmetric squat (the foot never acquires roll
    // rate) and fatal in a lateral weight shift, where the stance sole
    // rolled to 19 deg while the solver still reported the contact
    // satisfied. Once the sole is on its edge, patch_contact's rectangular
    // CoP box describes a footprint that is no longer touching the floor.
    // Rate (1/s) at which the contact anchor follows the foot's actual pose.
    // 0 freezes it at first touch, which is what produced a 24 N phantom
    // reaction after the foot slid. See the comment at the use site.
    let anchor_leak = env_f64("ANCHOR_LEAK", 0.2);
    let anchor_leak_rot = env_f64("ANCHOR_LEAK_ROT", 0.0);
    let kp_c = env_f64("KP_CONTACT", 1600.0);
    let kd_c = env_f64("KD_CONTACT", 80.0);
    let burnin_kp = env_f64("BURNIN_KP", prof.burnin_kp);
    let burnin_kv = env_f64("BURNIN_KV", prof.burnin_kv);
    let burnin_s = env_f64("BURNIN_S", 1.2);
    for j in robot.joints.iter_mut() {
        j.actuator_mode = ActuatorMode::Position;
        j.actuator_kp = burnin_kp;
        j.actuator_kv = burnin_kv;
        j.joint_damping = joint_damping;
        j.armature = armature;
    }

    // ── Spawn so the soles just touch: measured, not hand-derived ──────
    let sole_below_foot_origin: f64 = prof.sole_below_origin;
    // Fore/aft centre of the sole in the foot link frame. MUST match the
    // URDF's foot collision box origin.
    let sole_centre_x: f64 = prof.sole_centre_x;
    // Sole half-width. MUST match the URDF foot collision box.
    let sole_half_l: f64 = env_f64("SOLE_HALF_L", prof.cop_half.0);
    let sole_half_w: f64 = env_f64("SOLE_HALF_W", prof.cop_half.1);
    const SOLE_CLEARANCE: f64 = 0.001;
    let sim_dt = env_f64("SIM_DT", 0.001);
    let mu_ground = env_f64("MU_GROUND", 0.7);
    let make_opts = |z: f64| MjcfExportOptions {
        base_pos: Some([0.0, 0.0, z]),
        ground_plane: Some(GroundPlaneCfg { z: 0.0, half_size: 2.0, roll: 0.0, pitch: 0.0 }),
        timestep: Some(sim_dt),
        // Plant-side friction, separate from the QP's FRICTION_MU. Raising
        // it is not a fix -- it is how to measure what the stance foot's
        // slip is costing, by removing the slip and nothing else.
        default_friction: [mu_ground, 0.005, 0.0001],
        ..MjcfExportOptions::default()
    };
    let probe_z = env_f64("PROBE_Z", prof.probe_z);
    let spawn_z = {
        let probe = MujocoSim::new(&robot, make_opts(probe_z)).expect("probe sim");
        let f = probe.body_world_position(prof.foot_links[0]).expect("foot")[2];
        probe_z - ((f - sole_below_foot_origin) - SOLE_CLEARANCE)
    };
    {
        // Prove the base is genuinely FREE and the ground is real, rather
        // than trusting MjcfExportOptions::default(). Several sibling
        // examples in this directory deliberately weld the torso
        // (base_locked_axes: [true; 6]) and it would be easy to confuse
        // a suspended rig's result for a standing one.
        let xml = articara::mjcf::export_mjcf_with_options(&robot, make_opts(spawn_z));
        let base_free = xml.contains("<freejoint/>");
        let has_ground = xml.contains(r#"type="plane""#);
        println!("rig check: freejoint={base_free}  ground_plane={has_ground}");
        assert!(base_free, "base is NOT free -- this would be a suspended rig, not standing");
        assert!(has_ground, "no ground plane -- the feet would have nothing to push on");
    }
    let mut sim = MujocoSim::new(&robot, make_opts(spawn_z)).expect("MujocoSim::new");
    let mj_dt = sim.timestep();

    // The forearms used to sit geometrically INSIDE the hip blocks (shoulder
    // at y = +-0.08, 35 mm forearm, hip actuator reaching y = 0.095): 16
    // contacts and 37.2 kN at the spawn pose, 570x body weight, present in
    // every tick of every run. It braced the robot -- single-leg stance
    // "passed" only because of it. Shoulders moved to +-0.115; assert it
    // stays gone, because no number below means anything while the robot is
    // fighting itself.
    {
        let hits: Vec<String> = sim
            .contacts()
            .into_iter()
            .filter(|c| !c.body1.is_empty() && !c.body2.is_empty())
            .map(|c| format!("{} <-> {}", c.body1, c.body2))
            .collect();
        assert!(hits.is_empty(), "self-collision at spawn ({}): {:?}", hits.len(), hits);
    }

    let (model, a2m, link_to_idx) = build_floating_base_model(&robot);
    let nv = model.nv;
    let na_count = nv - 6;
    let left_foot_mi = *link_to_idx
        .get(prof.foot_links[0])
        .unwrap_or_else(|| panic!("no link {}", prof.foot_links[0]));
    let right_foot_mi = *link_to_idx
        .get(prof.foot_links[1])
        .unwrap_or_else(|| panic!("no link {}", prof.foot_links[1]));
    let trunk_mi = 1usize; // the FreeFlyer's own body

    // Links that carry mass, paired with their misarta index and the CoM
    // offset in the link frame -- everything `J_com` needs.
    struct MassLink {
        mi: usize,
        m: f64,
        com_local: na::Vector3<f64>,
    }
    let mass_links: Vec<MassLink> = robot
        .links
        .iter()
        .filter(|l| l.inertial.mass > 0.0)
        .filter_map(|l| {
            link_to_idx.get(&l.name).map(|&mi| {
                let o = l.inertial.origin.translation.vector;
                MassLink {
                    mi,
                    m: l.inertial.mass,
                    com_local: na::Vector3::new(o.x as f64, o.y as f64, o.z as f64),
                }
            })
        })
        .collect();
    let total_mass: f64 = mass_links.iter().map(|l| l.m).sum();
    const G: f64 = 9.81;
    println!(
        "centroidal model: nv={nv} na={na_count} mass_links={} total_mass={total_mass:.3} kg  dt={mj_dt}",
        mass_links.len()
    );

    let mut torque_max = na::DVector::from_element(na_count, 6.0);
    for ji in 0..robot.joints.len() {
        let Some(mi) = a2m[ji] else { continue };
        if model.joints[mi].joint_type.nv() != 1 {
            continue;
        }
        let vi = model.v_idx[mi];
        if vi >= 6 {
            torque_max[vi - 6] = robot.joints[ji].effort.max(1.0) * torque_scale;
        }
    }

    // ── Settle with the base WELDED, then hand a clean pose over ───────
    // A free-standing position-controlled biped is laterally unstable:
    // it has no balance control, so any numerical asymmetry grows and the
    // whole robot slides sideways. Settling in place therefore lands the
    // CoM somewhere arbitrary — measured across burn-in lengths of
    // 0.3/0.6/0.9/1.2/1.6 s it drifted to com_y between -0.024 and
    // -0.090 m, and since the lateral support only spans +-0.089 m the
    // run's survival flipped with it. That makes every downstream
    // comparison a coin toss rather than a measurement.
    //
    // The model is symmetric, so the correct settled state is symmetric.
    // Split the two problems: settle the JOINTS against the floor with
    // the base held (the well-posed half), then start the real free-base
    // run from those angles, centred. Same two-pass idea as the spawn
    // probe above.
    {
        let mut settle_robot = robot.clone();
        let settle_opts = MjcfExportOptions {
            base_locked_axes: [true; 6],
            ..make_opts(spawn_z)
        };
        let mut settle = MujocoSim::new(&settle_robot, settle_opts).expect("settle sim");
        for ji in 0..settle_robot.joints.len() {
            settle.set_position_target(ji, q_seed[ji]);
        }
        settle.step_n_frames(&mut settle_robot, (burnin_s / mj_dt) as u32, true);
        for ji in 0..robot.joints.len() {
            robot.joint_positions[ji] = settle_robot.joint_positions[ji];
        }
        // Name the joint, not just the number. A big number on an arm is
        // cosmetic; the same number on a stance knee means the plant cannot
        // hold the seed pose and nothing measured afterwards is worth much.
        let (wi, worst) = (0..robot.joints.len())
            .map(|ji| (ji, (robot.joint_positions[ji] - q_seed[ji]).abs()))
            .fold((0, 0.0_f64), |acc, x| if x.1 > acc.1 { x } else { acc });
        println!(
            "settled (base welded) for {burnin_s}s: max joint move from seed = {worst:.4} rad  ({})",
            robot.joints[wi].name
        );
        let mut moved: Vec<(f64, &str)> = (0..robot.joints.len())
            .map(|ji| ((robot.joint_positions[ji] - q_seed[ji]).abs(), robot.joints[ji].name.as_str()))
            .filter(|(d, _)| *d > 0.01)
            .collect();
        moved.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap());
        for (d, n) in moved.iter().take(6) {
            println!("    {n:32} {:+.4} rad", d);
        }
    }
    // Re-spawn free-based at the settled pose, centred at the origin.
    let mut sim = MujocoSim::new(&robot, make_opts(spawn_z)).expect("MujocoSim::new (run)");
    for ji in 0..robot.joints.len() {
        sim.set_position_target(ji, robot.joint_positions[ji]);
    }
    // Brief hold so the contacts engage before torque control starts.
    sim.step_n_frames(&mut robot, (0.05 / mj_dt) as u32, true);
    for j in robot.joints.iter_mut() {
        j.actuator_mode = ActuatorMode::Torque;
    }
    {
        let hr = |n: &str| sim.joint_q_qd(n).map(|(q, _)| q).unwrap_or(f64::NAN);
        let lp = sim.body_world_position(prof.foot_links[0]).unwrap();
        let rp = sim.body_world_position(prof.foot_links[1]).unwrap();
        let rpy = sim.body_world_orientation(&robot.root_link).unwrap().euler_angles();
        println!(
            "post-burn-in: rpy=({:+.3},{:+.3},{:+.3}) hip_roll=({:+.3},{:+.3}) foot inner-gap={:+.4}",
            rpy.0, rpy.1, rpy.2,
            hr(prof.hip_roll[0]), hr(prof.hip_roll[1]),
            (lp[1] - 0.019) - (rp[1] + 0.019),
        );
    }

    let mut solver = Solver::new();
    let cfg = SolveConfig::default();
    const FRICTION_MU: f64 = 0.6;
    let kp_com = env_f64("KP_COM", 300.0);
    let kd_com = env_f64("KD_COM", 80.0);
    let kp_trunk = env_f64("KP_TRUNK", 200.0);
    let kd_trunk = env_f64("KD_TRUNK", 40.0);
    let kp_post = env_f64("KP_POST", 100.0);
    let kd_post = env_f64("KD_POST", 20.0);
    let use_post = flag("POST", true);
    let trunk_sign = env_f64("TRUNK_SIGN", 1.0);
    // Shrink the admissible CoP box to keep a margin: riding the exact
    // edge means the next disturbance makes P0 infeasible outright.
    let cop_frac = env_f64("COP_FRAC", 1.0);
    let com_sign = env_f64("COM_SIGN", 1.0);
    let comchk = flag("COMCHK", false);
    let period_s = env_f64("PERIOD_S", 3.0);
    let squat_amp = env_f64("AMP", 0.0); // 0 = hold still; >0 = squat
    // Single-support mode: shift the CoM over the left foot, then release
    // the right foot's contact and lift it. Loads hip_pitch/hip_roll the
    // way a squat never does, which is what the "does hip_pitch really
    // need two motors" question actually needs measuring against.
    let lift_leg = flag("LIFT", false);
    // Static fore/aft CoM offset, both feet down. Loads hip_pitch the way
    // stance does, and stays inside a regime the QP can actually solve.
    let com_dx = env_f64("COM_DX", 0.0);
    let t_shift = env_f64("T_SHIFT", 3.0);   // seconds spent moving the CoM across
    let lift_h = env_f64("LIFT_H", 0.04);    // swing-foot clearance, m
    let lift_ramp = env_f64("LIFT_RAMP", 1.0);
    // Seconds spent unloading the swing foot before it leaves the contact
    // set. 0 restores the old one-tick switch. 0.10 measured best (degraded
    // 223 -> 150, tilt 0.268 -> 0.197 rad); 0.25 and above fall over,
    // because the ramp only moves the cone collapse earlier -- it does not
    // create the CoP margin the stance foot is already out of.
    let unload_ramp = env_f64("UNLOAD_RAMP", 0.0);
    // Degraded-solve fallback gains (torque PD onto the seed posture, on top
    // of gravity compensation). HOLD_LAST=1 restores the old freeze-the-last-
    // good-torque behaviour for comparison.
    let hold_kp = env_f64("HOLD_KP", 15.0);
    let hold_kd = env_f64("HOLD_KD", 2.0);
    // Consecutive degraded ticks bridged with the last good torque before
    // switching to the recomputed one.
    let hold_bridge = env_f64("HOLD_BRIDGE", 8.0) as u32;
    // Target the force regulariser at the load split the CoM reference
    // implies, rather than an equal share.
    //
    // Default OFF, and that is not because it is wrong -- it is the correct
    // formulation and it removes the CoP saturation completely (peak box use
    // over the whole weight shift 0.99 -> 0.17). It is off because switching
    // it on makes the run FALL: it uncovers a lift-off roll transient that
    // the saturated CoP was masking, and until that is fixed, LAT_SHARE=0 is
    // the configuration that actually stands up. Turn it on to work on the
    // real problem; leave it off to reproduce the standing baseline.
    let lat_share = flag("LAT_SHARE", true);
    // Ticks spent crossfading on each fallback <-> QP handover. Default OFF:
    // measured, every non-zero length falls (10/20/40/80 ticks all topple,
    // 20 drives the knee to -25.8 deg). The step at handover is not what it
    // looked like -- the same stance-foot liftoff happens with the fallback
    // disabled entirely, always on the tick a degraded RUN ends, so the
    // discontinuity is between the QP's broken solution and its recovered
    // one, and crossfading into a broken solution cannot help.
    let blend_ticks = if flag("HOLD_LAST", false) { 0 } else { env_f64("BLEND_TICKS", 0.0) as u32 };
    // Constrain only the swing foot's clearance, not its world x,y.
    let swing_z_only = flag("SWING_ZONLY", true);
    let kp_sw = env_f64("KP_SWING", 400.0);
    let kd_sw = env_f64("KD_SWING", 40.0);
    let total_t = env_f64("T", 6.0);

    let mj_substeps = (0.005 / mj_dt).round().max(1.0) as u32;
    let dt = mj_substeps as f64 * mj_dt;
    let n_ticks = (total_t / dt) as usize;

    // Helper: world-frame CoM position from an FK snapshot.
    let com_of = |data: &misarta::data::Data<f64>| -> na::Vector3<f64> {
        let mut c = na::Vector3::zeros();
        for l in &mass_links {
            let r = misarta::se3::rotation_matrix(&data.oMi[l.mi]);
            let o = misarta::se3::translation(&data.oMi[l.mi]);
            c += l.m * (o + r * l.com_local);
        }
        c / total_mass
    };

    // ── Where does the CoM actually sit inside the footprint? ──────────
    // The fore/aft split of the sole about the ankle sets how far the
    // centre of pressure can travel each way, and that only helps if it
    // is matched to which way the robot actually tends to fall. Copying
    // the human 25/75 split assumes a human's stance, where the CoM sits
    // well forward of the ankle; it is the wrong trade if this robot's
    // CoM sits level with or behind its ankles.
    {
        let d0 = misarta::fk::forward_kinematics(&model, &{
            let p = sim.body_world_position(&robot.root_link).unwrap();
            let qq = sim.body_world_orientation(&robot.root_link).unwrap();
            let mut q = model.neutral_q();
            q[0] = p[0]; q[1] = p[1]; q[2] = p[2];
            q[3] = qq.i; q[4] = qq.j; q[5] = qq.k; q[6] = qq.w;
            for ji in 0..robot.joints.len() {
                if let Some(mi) = a2m[ji] {
                    if model.joints[mi].joint_type.nq() == 1 {
                        q[model.q_idx[mi]] = robot.joint_positions[ji];
                    }
                }
            }
            q
        });
        let com0 = com_of(&d0);
        let ankle_x = 0.5
            * (misarta::se3::translation(&d0.oMi[left_foot_mi]).x
                + misarta::se3::translation(&d0.oMi[right_foot_mi]).x);
        let (cx, half) = (sole_centre_x, sole_half_l);
        let (back, front) = (ankle_x + cx - half, ankle_x + cx + half);
        println!(
            "footprint: ankle x={ankle_x:+.4}  sole x=[{back:+.4},{front:+.4}]  CoM x={:+.4}",
            com0.x
        );
        println!(
            "  CoM is {:+.1} mm relative to the ankle;  margin back {:.1} mm / front {:.1} mm",
            (com0.x - ankle_x) * 1000.0,
            (com0.x - back) * 1000.0,
            (front - com0.x) * 1000.0
        );
        let centred_cx = com0.x - ankle_x;
        println!(
            "  sole centre that would put the CoM mid-footprint: x = {centred_cx:+.4} (currently {cx:+.4})"
        );
    }


    // Trajectory log for offline rendering (same column layout
    // kyo46rs_squat.rs uses, so the replay tooling is shared).
    let log_joint_order: Vec<&str> = prof.log_joints.to_vec();
    let mut log_file = std::env::var("TRAJ_CSV").ok().map(|path| {
        use std::io::Write;
        let mut f = std::fs::File::create(&path).expect("create trajectory log");
        write!(f, "t,x,y,z,qw,qx,qy,qz,com_x,com_y,com_z,com_ref_z,tilt,com_ref_y,n_stance,swing_z").unwrap();
        // Centre of pressure per foot, in that foot's sole frame (mm), from
        // two independent sources: what the QP's solved contact wrench
        // implies, and what MuJoCo's actual contact set produces. They
        // disagree exactly where the solve degrades, which is the point of
        // logging both. fz<=0 means "no support / no valid solution".
        write!(f, ",degraded,cop_lx,cop_ly,slip_l,slip_r,patch_lx,patch_ly").unwrap();
        for side in ["l", "r"] {
            for src in ["qp", "mj"] {
                for ax in ["x", "y", "z"] {
                    write!(f, ",f{src}_{side}_{ax}").unwrap();
                }
            }
        }
        for side in ["l", "r"] {
            for src in ["qp", "mj"] {
                write!(f, ",cop_{src}_{side}_x,cop_{src}_{side}_y,fz_{src}_{side}").unwrap();
            }
        }
        for n in &log_joint_order {
            write!(f, ",{n}").unwrap();
        }
        // WBC-commanded torque per joint, plus the joint's effort limit, so
        // a replay can show demand against capability and make saturation
        // visible rather than silently clipped.
        for n in &log_joint_order {
            write!(f, ",tau_{n}").unwrap();
        }
        for n in &log_joint_order {
            write!(f, ",lim_{n}").unwrap();
        }
        writeln!(f).unwrap();
        println!("logging trajectory to {path}");
        f
    });

    let mut com_ref0: Option<na::Vector3<f64>> = None;
    let mut swing_home_cell: Option<na::Vector3<f64>> = None;
    let mut last_good: Option<Vec<f64>> = None;
    let mut consec_degraded: u32 = 0;
    // Crossfade state for the fallback <-> QP handover. Swapping controllers
    // in one tick puts a step into the torque, and a step lands in the
    // contact: measured, the stance foot left the ground entirely on the
    // tick after a 40 ms fallback episode ended (fz 0 -> 113.9 N, 1.75x body
    // weight) and the resulting bounce, not the lift-off itself, is what set
    // the final lean.
    let mut cmd_prev: Vec<f64> = vec![0.0; robot.joints.len()];
    let mut blend_from: Vec<f64> = vec![0.0; robot.joints.len()];
    let mut blend_left: u32 = 0;
    let mut in_fallback_prev = false;
    // touchdown pose per foot: (position, rotation) the contact should hold
    let mut anchor: [Option<(na::Vector3<f64>, na::Matrix3<f64>)>; 2] = [None, None];
    let mut prev_com: Option<na::Vector3<f64>> = None;
    let mut prev_body_pos: Option<[f64; 3]> = None;
    let mut n_degraded = 0u32;
    let mut fell = false;
    let mut min_z = f64::INFINITY;
    let mut max_tilt: f64 = 0.0;
    let mut max_jcom_err: f64 = 0.0;

    for tick in 0..n_ticks {
        let t = tick as f64 * dt;

        // ---- sync state ------------------------------------------------
        let body_pos = sim.body_world_position(&robot.root_link).unwrap();
        let body_quat = sim.body_world_orientation(&robot.root_link).unwrap();
        let v_lin_w = sim.body_world_linear_velocity(&robot.root_link).unwrap();
        let v_ang_w = sim.body_world_angular_velocity(&robot.root_link).unwrap();
        let r_wb = body_quat.to_rotation_matrix();
        let r_bw = r_wb.transpose();
        let v_lin_body = r_bw * na::Vector3::new(v_lin_w[0], v_lin_w[1], v_lin_w[2]);
        let v_ang_body = r_bw * na::Vector3::new(v_ang_w[0], v_ang_w[1], v_ang_w[2]);

        // Is `body_world_linear_velocity` actually the velocity of the
        // body ORIGIN (what `body_world_position` reports)? It reads
        // MuJoCo's `cvel`, whose linear part is expressed in the c-frame
        // -- world-aligned axes but origin at the subtree CoM, not at
        // xpos. If so it differs from d(xpos)/dt by omega x (xpos - com).
        if flag("VELCHK", false) {
            if let Some(pp) = prev_body_pos {
                let fd: [f64; 3] = [
                    (body_pos[0] - pp[0]) / dt,
                    (body_pos[1] - pp[1]) / dt,
                    (body_pos[2] - pp[2]) / dt,
                ];
                if tick % 20 == 0 {
                    println!(
                        "  [velchk] d(xpos)/dt=({:+.4},{:+.4},{:+.4})  cvel_lin=({:+.4},{:+.4},{:+.4})",
                        fd[0], fd[1], fd[2], v_lin_w[0], v_lin_w[1], v_lin_w[2]
                    );
                }
            }
        }
        prev_body_pos = Some(body_pos);

        let mut q = model.neutral_q();
        q[0] = body_pos[0];
        q[1] = body_pos[1];
        q[2] = body_pos[2];
        q[3] = body_quat.i;
        q[4] = body_quat.j;
        q[5] = body_quat.k;
        q[6] = body_quat.w;
        let mut v = vec![0.0_f64; nv];
        // FreeFlyer motion subspace is I6 in the BODY frame with row
        // order [angular; linear] (misarta joint.rs), so v[0..3] is
        // omega_body and v[3..6] is v_body.
        v[0] = v_ang_body.x;
        v[1] = v_ang_body.y;
        v[2] = v_ang_body.z;
        v[3] = v_lin_body.x;
        v[4] = v_lin_body.y;
        v[5] = v_lin_body.z;
        for ji in 0..robot.joints.len() {
            let Some(mi) = a2m[ji] else { continue };
            if model.joints[mi].joint_type.nq() == 1 {
                q[model.q_idx[mi]] = robot.joint_positions[ji];
            }
            if model.joints[mi].joint_type.nv() == 1 {
                if let Some((_, qd)) = sim.joint_q_qd(&robot.joints[ji].name) {
                    v[model.v_idx[mi]] = qd.clamp(-5.0, 5.0);
                }
            }
        }
        let v_dvec = na::DVector::from_column_slice(&v);

        let mut mass = misarta::crba::crba(&model, &q);
        let mut h = misarta::rnea::nonlinear_effects(&model, &q, &v);
        // misarta's dynamics Model carries no rotor inertia and no joint
        // damping -- `armature`/`joint_damping` are MJCF export fields and
        // reach the plant only. So the WBC has been solving a model that
        // differs from the simulator on EVERY actuated joint. Reflected
        // rotor inertia is a diagonal add to M; viscous damping is a
        // velocity term in h. Add both at this boundary so the two agree.
        for ji in 0..robot.joints.len() {
            let Some(mi) = a2m[ji] else { continue };
            if model.joints[mi].joint_type.nv() != 1 {
                continue;
            }
            let vi = model.v_idx[mi];
            if vi < 6 {
                continue;
            }
            mass[(vi, vi)] += armature;
            h[vi] += joint_damping * v[vi];
        }
        let data = misarta::fk::forward_kinematics(&model, &q);

        // ---- CoM position, Jacobian and bias, all world-frame ----------
        // Per link: take its parent joint's world Jacobian (rows 0..3 =
        // angular, 3..6 = linear, at the JOINT origin) and shift the
        // linear part out to the link's own CoM,
        //     v_c = v_o + omega x r      =>  J_lin_c = J_lin_o - [r]x J_ang
        // then mass-average. The bias picks up the centripetal term:
        //     dJv_lin_c = dJv_lin_o - [r]x dJv_ang + omega x (omega x r)
        let com = com_of(&data);
        let mut j_com = na::DMatrix::zeros(3, nv);
        let mut djv_com = na::Vector3::zeros();
        for l in &mass_links {
            let rot = misarta::se3::rotation_matrix(&data.oMi[l.mi]);
            let r = rot * l.com_local;
            let skew = na::Matrix3::new(
                0.0, -r.z, r.y,
                r.z, 0.0, -r.x,
                -r.y, r.x, 0.0,
            );
            let j = misarta::jacobian::compute_joint_jacobian_from_data(&model, &q, &data, l.mi);
            let j_ang = j.rows(0, 3).into_owned();
            let j_lin = j.rows(3, 3).into_owned();
            let j_lin_c = &j_lin - &skew * &j_ang;
            j_com += l.m * j_lin_c;

            let dj = misarta::jacobian::compute_joint_jacobian_time_derivative(&model, &q, &v, l.mi);
            let djv = &dj * &v_dvec;
            let djv_ang = na::Vector3::new(djv[0], djv[1], djv[2]);
            let djv_lin = na::Vector3::new(djv[3], djv[4], djv[5]);
            let omega = &j_ang * &v_dvec;
            let omega = na::Vector3::new(omega[0], omega[1], omega[2]);
            djv_com += l.m * (djv_lin - skew * djv_ang + omega.cross(&omega.cross(&r)));
        }
        j_com /= total_mass;
        djv_com /= total_mass;

        // One-shot column-wise check of J_com against finite differences
        // on the joint coordinates (the base columns need quaternion
        // integration, so they are checked via the running J*v vs
        // d(com)/dt comparison below instead).
        if tick == 0 && flag("COLCHK", false) {
            const EPS: f64 = 1e-6;
            let mut worst = (0usize, 0.0_f64, String::new());
            for ji in 0..robot.joints.len() {
                let Some(mi) = a2m[ji] else { continue };
                if model.joints[mi].joint_type.nv() != 1 {
                    continue;
                }
                let (qi, vi) = (model.q_idx[mi], model.v_idx[mi]);
                if vi < 6 {
                    continue;
                }
                let mut qp = q.clone();
                qp[qi] += EPS;
                let fd = (com_of(&misarta::fk::forward_kinematics(&model, &qp)) - com) / EPS;
                let col = na::Vector3::new(j_com[(0, vi)], j_com[(1, vi)], j_com[(2, vi)]);
                let e = (fd - col).norm();
                if e > worst.1 {
                    worst = (vi, e, robot.joints[ji].name.clone());
                }
                if e > 1e-4 {
                    println!(
                        "  [colchk] {:<28} v{vi}: fd=({:+.5},{:+.5},{:+.5}) J=({:+.5},{:+.5},{:+.5}) err={e:.2e}",
                        robot.joints[ji].name, fd.x, fd.y, fd.z, col.x, col.y, col.z
                    );
                }
            }
            println!("  [colchk] worst joint column: {} (v{}) err={:.3e}", worst.2, worst.0, worst.1);
        }

        let com_vel = &j_com * &v_dvec;
        let com_vel = na::Vector3::new(com_vel[0], com_vel[1], com_vel[2]);

        // Verify J_com against a finite difference of the measured CoM
        // rather than trusting the shift algebra.
        if let Some(pc) = prev_com {
            let fd = (com - pc) / dt;
            let err = (fd - com_vel).norm() / fd.norm().max(1e-3);
            max_jcom_err = max_jcom_err.max(err);
            if comchk && tick % 20 == 0 {
                println!(
                    "  [Jcom] fd=({:+.4},{:+.4},{:+.4})  J*v=({:+.4},{:+.4},{:+.4})  rel_err={err:.4}",
                    fd.x, fd.y, fd.z, com_vel.x, com_vel.y, com_vel.z
                );
            }
        }
        prev_com = Some(com);
        let com_ref0 = *com_ref0.get_or_insert(com);
        // Freeze the swing foot's start pose on the first tick so the lift
        // target does not chase the foot as it moves.
        let swing_home =
            *swing_home_cell.get_or_insert(misarta::se3::translation(&data.oMi[right_foot_mi]));

        // ---- contacts --------------------------------------------------
        // In single-support the right foot leaves the ground, so it must
        // also leave the contact set -- keeping its rows would have the QP
        // solve against a reaction force that no longer exists.
        let single = lift_leg && t >= t_shift;
        let stance: Vec<usize> = if single {
            vec![left_foot_mi]
        } else {
            vec![left_foot_mi, right_foot_mi]
        };
        let nc = stance.len();
        // Unloading ramp. Dropping the swing foot's rows in a single tick
        // hands its whole share of the load to a stance foot whose CoP box
        // is already pinned at the lateral edge, and the level-0 cone loses
        // its interior in one step -- that is the NumericalFailure at
        // t=t_shift. Ramping the swing foot's force ceiling to zero *before*
        // it leaves the set lets the remaining box tighten gradually.
        let unload = if lift_leg && unload_ramp > 0.0 {
            ((t - (t_shift - unload_ramp)) / unload_ramp).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let load_share = |foot_mi: usize| -> f64 {
            if foot_mi == right_foot_mi {
                1.0 - unload
            } else {
                1.0
            }
        };
        if single {
            anchor[1] = None;   // right foot is swinging; forget its anchor
        }
        let mut j_contact = na::DMatrix::zeros(6 * nc, nv);
        let mut dj_v = na::DVector::zeros(6 * nc);
        for (slot, foot_mi) in stance.iter().copied().enumerate() {
            let jf = misarta::jacobian::compute_joint_jacobian_from_data(&model, &q, &data, foot_mi);
            let djf = misarta::jacobian::compute_joint_jacobian_time_derivative(&model, &q, &v, foot_mi);
            let djv = &djf * &v_dvec;
            for r in 0..6 {
                for c in 0..nv {
                    j_contact[(6 * slot + r, c)] = jf[(r, c)];
                }
                dj_v[6 * slot + r] = djv[r];
            }
        }

        // Level 0's equality block, assembled explicitly so its conditioning
        // can be watched. x = [qddot(nv); f(6nc); tau(na)].
        //   EoM      : [ M | -Jc^T | -S^T ]
        //   contact  : [ Jc |  0    |  0   ]
        // A Clarabel NumericalFailure at level 0 is a conditioning failure,
        // so this is the matrix to look at when one appears.
        if flag("CONDCHK", false) && tick % 5 == 0 {
            let nx = nv + 6 * nc + na_count;
            let mut a0 = na::DMatrix::zeros(nv + 6 * nc, nx);
            for r in 0..nv {
                for c in 0..nv {
                    a0[(r, c)] = mass[(r, c)];
                }
                for c in 0..6 * nc {
                    a0[(r, nv + c)] = -j_contact[(c, r)];
                }
            }
            for i in 0..na_count {
                a0[(6 + i, nv + 6 * nc + i)] = -1.0;
            }
            for r in 0..6 * nc {
                for c in 0..nv {
                    a0[(nv + r, c)] = j_contact[(r, c)];
                }
            }
            let sv = a0.singular_values();
            let (mx, mn) = (sv.max(), sv.min());
            let jsv = j_contact.clone().singular_values();
            println!(
                "  [cond] t={t:6.3} nc={nc}  A0 cond={:10.1} (sigma_min {:.3e})   Jc cond={:8.1}   status={:?}",
                mx / mn, mn, jsv.max() / jsv.min(), "pending"
            );
        }

        let dyn_ctx = Dynamics::new(Formulation::Explicit, &mass, &h, &j_contact, na_count);
        let forces = dyn_ctx.forces();

        // patch_contact's CoP box is only the real centre-of-pressure
        // condition about the SOLE in the sole's frame; `forces` is about
        // the foot LINK ORIGIN, 0.059 m higher, where a tangential fx
        // fakes 0.059*fx of moment. Transform before constraining.
        // MUST track the URDF's foot collision box. The CoP box is
        // centred on this point, so a stale value silently constrains
        // the pressure centre about the wrong place -- moving the sole
        // in the model and forgetting this makes the QP defend a
        // footprint the robot no longer has.
        let sole_offset_local: [f64; 3] = [sole_centre_x, 0.0, -sole_below_foot_origin];
        let mut p0 = dyn_ctx
            .dynamics_task()
            .expect("Explicit keeps the EoM task")
            + tasks::box_bound(dyn_ctx.tau(), &torque_max);
        // Keep each foot's force -> sole-wrench map so the solved CoP can be
        // measured against the box that constrained it. A level-0
        // NumericalFailure says the cone was hard to navigate; this says
        // whether it was hard because the CoP had nowhere left to go.
        let mut sole_sel: Vec<na::DMatrix<f64>> = Vec::with_capacity(nc);
        for (slot, foot_mi) in stance.iter().copied().enumerate() {
            let js = j_contact.rows(6 * slot, 6).into_owned();
            let djvs = dj_v.rows(6 * slot, 6).into_owned();
            // pose error against the anchor, in world frame, [ang; lin]
            let pos = misarta::se3::translation(&data.oMi[foot_mi]);
            let rot = misarta::se3::rotation_matrix(&data.oMi[foot_mi]);
            let side = if foot_mi == left_foot_mi { 0 } else { 1 };
            anchor[side].get_or_insert((pos, rot));
            // Let the anchor follow a foot that has genuinely moved.
            //
            // The anchor was frozen once and never revisited, and the stance
            // foot slides ~12 mm during the transition. A stale anchor is not
            // a small error: kp_c * 12 mm = 19.6 m/s^2 of lateral foot
            // acceleration demanded forever, which the QP pays for by
            // planning a contact force that does not exist. Measured in the
            // settled single-leg pose, the QP believed it was applying 24 N
            // of tangential force -- 37% of body weight -- while MuJoCo's
            // contacts summed to 0.0 N, and it thought fz was 71.3 N when
            // the true value is 65.1 N = mg exactly. The torque it sends is
            // computed against that phantom reaction.
            //
            // Baumgarte is there to reject drift, not to relitigate where the
            // foot ought to be, so the anchor leaks toward the current pose
            // with a time constant far slower than the contact dynamics.
            if anchor_leak > 0.0 {
                if let Some((ap, ar)) = anchor[side].as_mut() {
                    let a = (anchor_leak * dt).min(1.0);
                    *ap += (pos - *ap) * a;
                    // Orientation is deliberately NOT leaked by default.
                    // Rotational drift is the failure this Baumgarte term
                    // exists for -- unchecked, the stance sole rolled to
                    // 19 deg while the solver still called the contact
                    // satisfied. Translation can be conceded; roll cannot.
                    let ar_a = (anchor_leak_rot * dt).min(1.0);
                    *ar = *ar + (rot - *ar) * ar_a;
                }
            }
            let (p0_, r0_) = anchor[side].expect("anchor set above");
            let dr = r0_ * rot.transpose();
            // rotation vector of dr (small-angle: the skew part)
            let e_ang = na::Vector3::new(dr[(2, 1)] - dr[(1, 2)],
                                        dr[(0, 2)] - dr[(2, 0)],
                                        dr[(1, 0)] - dr[(0, 1)]) * 0.5;
            let e_lin = p0_ - pos;
            let vel = &js * &v_dvec;
            let mut acc_ref = na::DVector::zeros(6);
            for k in 0..3 {
                acc_ref[k] = kp_c * e_ang[k] - kd_c * vel[k];
                acc_ref[3 + k] = kp_c * e_lin[k] - kd_c * vel[3 + k];
            }
            let rot = misarta::se3::rotation_matrix(&data.oMi[foot_mi]);
            let r_w = rot
                * na::Vector3::new(sole_offset_local[0], sole_offset_local[1], sole_offset_local[2]);
            let rt = rot.transpose();
            let skew = na::Matrix3::new(
                0.0, -r_w.z, r_w.y,
                r_w.z, 0.0, -r_w.x,
                -r_w.y, r_w.x, 0.0,
            );
            let top_right = -(rt * skew);
            let mut sel = na::DMatrix::zeros(6, forces.size());
            for i in 0..3 {
                for jj in 0..3 {
                    sel[(i, 6 * slot + jj)] = rt[(i, jj)];
                    sel[(i, 6 * slot + 3 + jj)] = top_right[(i, jj)];
                    sel[(3 + i, 6 * slot + 3 + jj)] = rt[(i, jj)];
                }
            }
            let w_sole = &sel * &forces.as_affine();
            sole_sel.push(sel.clone());
            p0 = p0 + tasks::cartesian_acceleration(dyn_ctx.qddot(), &js, &djvs, &acc_ref);
            if !flag("NO_PATCH", false) {
                // f_max carries the ramp. The CoP box is |m| <= L*fz, so
                // squeezing fz shrinks the box with it -- the swing foot
                // stops being able to argue for a CoP it is about to lose.
                let share = load_share(foot_mi);
                let sole_patch = tasks::ContactPatch {
                    mu: FRICTION_MU,
                    cop_half: (sole_half_l * cop_frac, sole_half_w * cop_frac),
                    mu_torsion: 0.05,
                    f_max: (150.0 * share).max(0.5),
                };
                p0 = p0 + tasks::patch_contact(&w_sole, &sole_patch);
            }
        }

        // ---- P1: the CoM task = balance (x,y) AND squat (z) ------------
        let phase = 2.0 * PI * t / period_s;
        let z_ref = com_ref0.z - squat_amp * (1.0 - phase.cos()) * 0.5;
        let zd_ref = -squat_amp * 0.5 * (2.0 * PI / period_s) * phase.sin();
        let zdd_ref = -squat_amp * 0.5 * (2.0 * PI / period_s).powi(2) * phase.cos();
        // Move the CoM over the stance foot BEFORE releasing the other one.
        let y_ref = if lift_leg {
            let stance_y = misarta::se3::translation(&data.oMi[left_foot_mi]).y;
            let a = (t / t_shift).clamp(0.0, 1.0);
            let a = 0.5 - 0.5 * (PI * a).cos();          // smooth ramp
            com_ref0.y + a * (stance_y - com_ref0.y)
        } else {
            com_ref0.y
        };
        let lean = com_dx * (t / 2.0).clamp(0.0, 1.0);   // ramp in over 2 s
        let c_ref = na::Vector3::new(com_ref0.x + lean, y_ref, z_ref);
        let cd_ref = na::Vector3::new(0.0, 0.0, zd_ref);
        let cdd_ref = na::Vector3::new(0.0, 0.0, zdd_ref);
        let a_com = com_sign * (cdd_ref + kd_com * (cd_ref - com_vel) + kp_com * (c_ref - com));
        let com_accel_ref = na::DVector::from_vec(vec![a_com.x, a_com.y, a_com.z]);
        let p1 = tasks::cartesian_acceleration(
            dyn_ctx.qddot(),
            &j_com,
            &na::DVector::from_vec(vec![djv_com.x, djv_com.y, djv_com.z]),
            &com_accel_ref,
        );

        // ---- P2: trunk upright, via the WORLD-frame angular Jacobian ---
        // Not qddot[0..2]: those are body-frame and carry the sign
        // inversion kyo46rs_squat.rs never explained. Rows 0..3 of the
        // trunk's own world Jacobian map qddot to world angular
        // acceleration, which matches the world-frame roll/pitch error.
        let j_trunk = misarta::jacobian::compute_joint_jacobian_from_data(&model, &q, &data, trunk_mi);
        let dj_trunk = misarta::jacobian::compute_joint_jacobian_time_derivative(&model, &q, &v, trunk_mi);
        let djv_trunk = &dj_trunk * &v_dvec;
        let (roll, pitch, _yaw) = body_quat.euler_angles();
        let mut j_rp = na::DMatrix::zeros(2, nv);
        for c in 0..nv {
            j_rp[(0, c)] = j_trunk[(0, c)];
            j_rp[(1, c)] = j_trunk[(1, c)];
        }
        // Same unexplained inversion kyo46rs_squat.rs hit, and it
        // SURVIVES the cvel fix: re-tested with the base velocity now
        // correct, +1.0 still diverges (0.185 s) and -1.0 still does not
        // (0.525 s). So it is an independent angular-convention
        // mismatch between misarta's model and MuJoCo, not a knock-on of
        // the velocity bug. The CoM task by contrast needs NO flip --
        // com_sign=-1 merely postpones the fall by turning the task into
        // slow positive feedback (CoM z drifts 0.2939 -> 0.3388 instead
        // of tracking), which is why it is not the default despite
        // "surviving" longer.
        let rp_ref = na::DVector::from_vec(vec![
            trunk_sign * (kp_trunk * (0.0 - roll) + kd_trunk * (0.0 - v_ang_w[0])),
            trunk_sign * (kp_trunk * (0.0 - pitch) + kd_trunk * (0.0 - v_ang_w[1])),
        ]);
        let p2 = tasks::cartesian_acceleration(
            dyn_ctx.qddot(),
            &j_rp,
            &na::DVector::from_vec(vec![djv_trunk[0], djv_trunk[1]]),
            &rp_ref,
        );

        // ---- P3: weak posture, so the null space does not wander -------
        let mut j_post = na::DMatrix::zeros(na_count, nv);
        let mut post_ref = na::DVector::zeros(na_count);
        for ji in 0..robot.joints.len() {
            let Some(mi) = a2m[ji] else { continue };
            if model.joints[mi].joint_type.nv() != 1 {
                continue;
            }
            let vi = model.v_idx[mi];
            if vi >= 6 {
                j_post[(vi - 6, vi)] = 1.0;
                post_ref[vi - 6] =
                    kp_post * (q_seed[ji] - robot.joint_positions[ji]) + kd_post * (0.0 - v[vi]);
            }
        }
        let p3 = tasks::cartesian_acceleration(
            dyn_ctx.qddot(),
            &j_post,
            &na::DVector::zeros(na_count),
            &post_ref,
        );

        // ---- lowest: gravity-comp torque + even weight split -----------
        let g_full = misarta::rnea::compute_gravity(&model, &q);
        let mut tau_gravity = na::DVector::zeros(na_count);
        for ji in 0..robot.joints.len() {
            let Some(mi) = a2m[ji] else { continue };
            if model.joints[mi].joint_type.nv() != 1 {
                continue;
            }
            let vi = model.v_idx[mi];
            if vi >= 6 {
                tau_gravity[vi - 6] = g_full[vi];
            }
        }
        // Split the nominal load the same way the ramp splits the ceiling,
        // so the regulariser is not still asking the swing foot for half the
        // weight while the patch constraint is taking it away.
        let mut forces_nominal = na::DVector::zeros(forces.size());
        // Ask each foot for the load that PUTS the net CoP under the CoM
        // reference, not for an equal share. An equal-share target is the
        // reason the CoP box saturated: the CoM task can be met either by
        // transferring load between the feet or by walking the CoP outward,
        // those are interchangeable in the task's null space, and a 50/50
        // force target makes the regulariser pick the second one every time.
        // Measured with the equal-share target: at CoM y = +15 mm the split
        // was still 50.1/49.9 and the stance CoP was already 74% of the way
        // to its edge, when 60.8/39.2 would have held the CoP centred.
        let lat: Vec<f64> = if lat_share {
            let ys: Vec<f64> = stance
                .iter()
                .map(|&mi| misarta::se3::translation(&data.oMi[mi]).y)
                .collect();
            match ys.len() {
                2 => {
                    let (y0, y1) = (ys[0], ys[1]);
                    let a = if (y0 - y1).abs() > 1e-6 {
                        ((y_ref - y1) / (y0 - y1)).clamp(0.0, 1.0)
                    } else {
                        0.5
                    };
                    vec![a, 1.0 - a]
                }
                _ => vec![1.0; ys.len()],
            }
        } else {
            vec![1.0; nc]
        };
        let shares: Vec<f64> = stance
            .iter()
            .copied()
            .zip(&lat)
            .map(|(mi, l)| load_share(mi) * l)
            .collect();
        let share_tot: f64 = shares.iter().sum::<f64>().max(1e-6);
        for slot in 0..nc {
            forces_nominal[6 * slot + 5] = total_mass * G * shares[slot] / share_tot;
        }
        let p_reg = tasks::regularize(dyn_ctx.tau(), &tau_gravity)
            + tasks::regularize(&dyn_ctx.forces(), &forces_nominal);

        // Swing foot: hold it at a clearance above where it started.
        let p_swing = if single {
            let jf = misarta::jacobian::compute_joint_jacobian_from_data(&model, &q, &data, right_foot_mi);
            let djf = misarta::jacobian::compute_joint_jacobian_time_derivative(&model, &q, &v, right_foot_mi);
            let djv = &djf * &v_dvec;
            let pos = misarta::se3::translation(&data.oMi[right_foot_mi]);
            let vel = &jf.rows(3, 3).into_owned() * &v_dvec;
            // Ramp the clearance in rather than stepping it: releasing the
            // contact and jumping the target 40 mm in the same tick is a
            // step input, and its reaction lands straight on the stance
            // foot's narrow CoP budget.
            let a_lift = ((t - t_shift) / lift_ramp).clamp(0.0, 1.0);
            let a_lift = 0.5 - 0.5 * (PI * a_lift).cos();
            let tgt = swing_home + na::Vector3::new(0.0, 0.0, lift_h * a_lift);
            let a = kp_sw * (tgt - pos) - kd_sw * na::Vector3::new(vel[0], vel[1], vel[2]);
            // What this task is FOR is clearance -- the foot must not scuff.
            // Constraining x and y as well pins the swing foot to the world
            // position it happened to occupy at t=0, on a robot that is
            // deliberately translating its body 70 mm sideways, and the
            // reaction for holding it there lands on the stance leg.
            if swing_z_only {
                Some(tasks::cartesian_acceleration(
                    dyn_ctx.qddot(),
                    &jf.rows(5, 1).into_owned(),
                    &na::DVector::from_vec(vec![djv[5]]),
                    &na::DVector::from_vec(vec![a.z]),
                ))
            } else {
                Some(tasks::cartesian_acceleration(
                    dyn_ctx.qddot(),
                    &jf.rows(3, 3).into_owned(),
                    &na::DVector::from_vec(vec![djv[3], djv[4], djv[5]]),
                    &na::DVector::from_vec(vec![a.x, a.y, a.z]),
                ))
            }
        } else {
            None
        };

        let mut levels = vec![p0, p1, p2];
        if let Some(ps) = p_swing {
            levels.push(ps);
        }
        if use_post {
            levels.push(p3);
        }
        levels.push(p_reg);
        let sol = solver
            .solve(&levels, &cfg)
            .unwrap_or_else(|e| panic!("wbc solve failed at t={t:.3}: {e}"));
        if !matches!(sol.status, misa_wbc::SolveStatus::Optimal) {
            n_degraded += 1;
            if n_degraded <= 6 || tick % 200 == 0 {
                println!("    [degraded] t={t:6.3} nc={nc} status={:?}", sol.status);
            }
        }
        let extracted = dyn_ctx.extract(&sol.x);

        // Where did the QP actually put the centre of pressure, and how much
        // of the box was left? w_sole = [m(0..2); f(3..5)] in the sole frame,
        // so cop = (-my/fz, mx/fz) and the patch constraint is |cop| <= L.
        if flag("COPCHK", false) && tick % 5 == 0 {
            let (lx, ly) = (sole_half_l * cop_frac, sole_half_w * cop_frac);
            let mut parts = Vec::new();
            for (slot, sel) in sole_sel.iter().enumerate() {
                let w = sel * &extracted.forces;
                let fz = w[5];
                if fz.abs() < 1e-6 {
                    parts.push(format!("foot{slot}: fz~0"));
                    continue;
                }
                let (cx, cy) = (-w[1] / fz, w[0] / fz);
                parts.push(format!(
                    "foot{slot} fz={fz:6.1}N cop=({:+6.1},{:+6.1})mm  use=({:5.2},{:5.2})",
                    cx * 1e3, cy * 1e3, cx.abs() / lx, cy.abs() / ly
                ));
            }
            println!("  [cop] t={t:6.3} nc={nc}  {}", parts.join("   "));
        }

        // CoP per foot in that foot's sole frame, side 0 = left, 1 = right,
        // as [x, y, fz]. fz = 0 means the foot is unsupported (or, for the
        // QP column, that the solve degraded and returned nothing usable).
        // Both are evaluated against the same `q`/`data` as the QP saw, so
        // they are directly comparable.
        let mut cop_qp = [[0.0_f64; 3]; 2];
        // The force variables themselves are WORLD frame (`sel` is what
        // rotates them into the sole), so this is directly comparable with
        // what MuJoCo's contacts sum to -- same frame, same instant, same q.
        let mut f_qp_w = [[0.0_f64; 3]; 2];
        for (slot, foot_mi) in stance.iter().copied().enumerate() {
            let side = usize::from(foot_mi != left_foot_mi);
            let w = &sole_sel[slot] * &extracted.forces;
            let fz = w[5];
            if fz > 1e-6 {
                cop_qp[side] = [-w[1] / fz, w[0] / fz, fz];
            }
            for k in 0..3 {
                f_qp_w[side][k] = extracted.forces[6 * slot + 3 + k];
            }
        }
        let mut cop_mj = [[0.0_f64; 3]; 2];
        let mut slip = [0.0_f64; 2];   // |f_tangential| / (mu * fz) per foot
        let mut patch_w = [[0.0_f64; 2]; 2];   // world (x,y) of each contact patch
        let mut f_mj_w = [[0.0_f64; 3]; 2];    // MuJoCo world contact force per foot
        {
            // Force-weighted mean of the ground contact points on each foot.
            let mut acc = [[0.0_f64; 4]; 2]; // [sum fz*x, fz*y, fz*z, sum fz]
            // Tangential ground force per foot, to test the stance foot
            // against its own friction cone rather than assuming it sticks.
            let mut ft = [[0.0_f64; 2]; 2];
            for c in sim.contacts() {
                let name = if c.body1.is_empty() { &c.body2 } else { &c.body1 };
                let side = match name.as_str() {
                    n if n == prof.foot_links[0] => 0,
                    n if n == prof.foot_links[1] => 1,
                    _ => continue,
                };
                let fz = c.force_world[2];
                if fz <= 0.0 {
                    continue;
                }
                for k in 0..3 {
                    acc[side][k] += fz * c.pos[k];
                }
                acc[side][3] += fz;
                ft[side][0] += c.force_world[0];
                ft[side][1] += c.force_world[1];
            }
            for side in 0..2 {
                let fz = acc[side][3];
                if fz <= 1e-6 {
                    continue;
                }
                let foot_mi = if side == 0 { left_foot_mi } else { right_foot_mi };
                let o = misarta::se3::translation(&data.oMi[foot_mi]);
                let r = misarta::se3::rotation_matrix(&data.oMi[foot_mi]);
                let pw = na::Vector3::new(acc[side][0] / fz, acc[side][1] / fz, acc[side][2] / fz);
                let pl = r.transpose() * (pw - o);
                cop_mj[side] = [pl.x - sole_centre_x, pl.y, fz];
                let tan = (ft[side][0].powi(2) + ft[side][1].powi(2)).sqrt();
                slip[side] = tan / (FRICTION_MU * fz).max(1e-9);
                // WORLD position of the contact patch. The link origin sits
                // 35 mm above the sole, so it swings sideways when the ankle
                // rolls -- watching the origin cannot tell a foot that slid
                // from a foot that merely tipped. This can.
                patch_w[side] = [pw.x, pw.y];
                f_mj_w[side] = [ft[side][0], ft[side][1], fz];
            }
        }

        let mut robot_taus = vec![0.0_f64; robot.joints.len()];
        for ji in 0..robot.joints.len() {
            let Some(mi) = a2m[ji] else { continue };
            if model.joints[mi].joint_type.nv() != 1 {
                continue;
            }
            let vi = model.v_idx[mi];
            if vi >= 6 {
                robot_taus[ji] = extracted.tau[vi - 6];
            }
        }
        // NO_TORQUE=1 sends zeros: a free base on a real floor must
        // collapse. If it does not, something is holding the robot.
        if flag("NO_TORQUE", false) {
            robot_taus.iter_mut().for_each(|t| *t = 0.0);
        }
        // A degraded solve is NOT a slightly-worse solve. misa-wbc's HoQp
        // returns x_new = prev.x on a failed inner QP, and at level 0 that is
        // the zero vector, so the "solution" satisfies the HOMOGENEOUS EoM --
        // all 65 N of gravity dropped, and the contact Baumgarte rows
        // discarded along with it, on exactly the transition ticks where they
        // work hardest. Hold the last good torque instead.
        // ...which is what the old fallback did, and it was measured to be
        // actively harmful here. A torque solved for two feet supplies about
        // half the support one foot needs, so freezing it let the stance
        // knee sag 49 -> 11 deg over 705 ms; the QP then had to recover from
        // a leg whose own 6x6 Jacobian had gone from cond 49 to cond 206,
        // which cost a 79.5 N (130% body weight) spike and left the robot
        // leaning. Recompute a support torque for the pose we are ACTUALLY
        // in instead: gravity compensation, which is already on hand for the
        // regulariser, plus a PD that holds the seed posture.
        let fallback_tau = |robot_taus: &mut Vec<f64>| {
            for ji in 0..robot.joints.len() {
                let Some(mi) = a2m[ji] else { continue };
                if model.joints[mi].joint_type.nv() != 1 {
                    continue;
                }
                let vi = model.v_idx[mi];
                if vi < 6 {
                    continue;
                }
                let e = q_seed[ji] - robot.joint_positions[ji];
                let tau = tau_gravity[vi - 6] + hold_kp * e - hold_kd * v[vi];
                let lim = torque_max[vi - 6];
                robot_taus[ji] = tau.clamp(-lim, lim);
            }
        };
        let mut in_fallback = false;
        match last_good.as_ref() {
            Some(prev) if !matches!(sol.status, misa_wbc::SolveStatus::Optimal) => {
                in_fallback = true;
                // Bridge a brief hiccup with the last good torque -- over a
                // few ticks it is the smoother choice, and swapping in a
                // different controller every other tick just chatters (a
                // straight swap measured a 256 N contact spike, 4x body
                // weight). Only a failure that PERSISTS means the stale
                // command no longer describes the robot's support state, and
                // that is when the recomputed torque earns its place.
                if flag("HOLD_LAST", false) || consec_degraded < hold_bridge {
                    robot_taus.copy_from_slice(prev);
                } else {
                    fallback_tau(&mut robot_taus);
                }
                consec_degraded += 1;
                // Holding the last good torque bridges an occasional failed
                // solve. It must not quietly become the controller: a long
                // run of failures means the robot is open-loop on a stale
                // command, which reads in the logs as a smooth mechanical
                // collapse rather than as a control fault. (Measured: 540 ms
                // of frozen torque while the stance knee folded 48 -> 11 deg,
                // with the torque columns byte-identical throughout.)
                if consec_degraded == 10 {
                    let src = if flag("HOLD_LAST", false) {
                        format!("still commanding the torque from t={:.3}",
                                t - consec_degraded as f64 * dt)
                    } else {
                        "running on gravity comp + posture PD".to_string()
                    };
                    println!("  [OPEN LOOP] t={t:6.3} nc={nc}: {consec_degraded} consecutive degraded solves, {src}");
                }
            }
            _ => {
                if consec_degraded >= 10 {
                    println!("  [recovered] t={t:6.3} after {consec_degraded} degraded ticks");
                }
                consec_degraded = 0;
                last_good = Some(robot_taus.clone());
            }
        }
        // Crossfade whenever the commanding controller changes, in either
        // direction, from whatever was last actually sent. `last_good` keeps
        // the QP's own output rather than this blended command, so the
        // bridge still freezes a real solution.
        if blend_ticks > 0 && in_fallback != in_fallback_prev {
            blend_left = blend_ticks;
            blend_from.copy_from_slice(&cmd_prev);
        }
        in_fallback_prev = in_fallback;
        if blend_left > 0 {
            let a = 1.0 - f64::from(blend_left) / f64::from(blend_ticks);
            for k in 0..robot_taus.len() {
                robot_taus[k] = blend_from[k] * (1.0 - a) + robot_taus[k] * a;
            }
            blend_left -= 1;
        }
        cmd_prev.copy_from_slice(&robot_taus);

        sim.set_wbc_torques(&robot_taus);
        sim.step_n_frames(&mut robot, mj_substeps, true);

        if let Some(f) = log_file.as_mut() {
            use std::io::Write;
            let p = sim.body_world_position(&robot.root_link).unwrap();
            let qq = sim.body_world_orientation(&robot.root_link).unwrap();
            write!(
                f,
                "{t:.4},{:.5},{:.5},{:.5},{:.5},{:.5},{:.5},{:.5},{:.5},{:.5},{:.5},{z_ref:.5},{:.5}",
                p[0], p[1], p[2], qq.w, qq.i, qq.j, qq.k,
                com.x, com.y, com.z,
                roll.abs().max(pitch.abs())
            )
            .unwrap();
            let sw = sim.body_world_position(prof.foot_links[1]).unwrap()[2];
            write!(f, ",{y_ref:.5},{nc},{sw:.5}").unwrap();
            let deg = u8::from(!matches!(sol.status, misa_wbc::SolveStatus::Optimal));
            write!(f, ",{deg},{:.5},{:.5}", sole_half_l * cop_frac, sole_half_w * cop_frac).unwrap();
            write!(f, ",{:.4},{:.4}", slip[0], slip[1]).unwrap();
            write!(f, ",{:.6},{:.6}", patch_w[0][0], patch_w[0][1]).unwrap();
            for side in 0..2 {
                for src in [&f_qp_w, &f_mj_w] {
                    let v = src[side];
                    write!(f, ",{:.4},{:.4},{:.4}", v[0], v[1], v[2]).unwrap();
                }
            }
            for side in 0..2 {
                for src in [&cop_qp, &cop_mj] {
                    let c = src[side];
                    write!(f, ",{:.6},{:.6},{:.4}", c[0], c[1], c[2]).unwrap();
                }
            }
            for n in &log_joint_order {
                let a = robot.joint_map.get(*n).map(|&ji| robot.joint_positions[ji]).unwrap_or(0.0);
                write!(f, ",{a:.5}").unwrap();
            }
            for n in &log_joint_order {
                let tq = robot.joint_map.get(*n).map(|&ji| robot_taus[ji]).unwrap_or(0.0);
                write!(f, ",{tq:.5}").unwrap();
            }
            for n in &log_joint_order {
                let lm = robot.joint_map.get(*n).map(|&ji| robot.joints[ji].effort).unwrap_or(0.0);
                write!(f, ",{lm:.3}").unwrap();
            }
            writeln!(f).unwrap();
        }

        let cur_z = sim.body_world_position(&robot.root_link).unwrap()[2];
        min_z = min_z.min(cur_z);
        let tilt = roll.abs().max(pitch.abs());
        max_tilt = max_tilt.max(tilt);
        if tick % 20 == 0 {
            println!(
                "  t={t:6.3}  com=({:+.4},{:+.4},{:+.4}) ref_z={z_ref:+.4}  roll={roll:+.3} pitch={pitch:+.3}  status={:?}",
                com.x, com.y, com.z, sol.status
            );
        }
        if cur_z < 0.30 || tilt > 0.52 {
            println!("  FELL at t={t:.3} (z={cur_z:.3}, tilt={tilt:.3})");
            fell = true;
            break;
        }
    }

    println!("\n=== Result (centroidal) ===");
    println!("  max |J_com*v - d(com)/dt| relative error: {max_jcom_err:.4}");
    println!("  min trunk z = {min_z:.3}   max tilt = {max_tilt:.3} rad");
    println!("  degraded solves: {n_degraded}");
    println!("  verdict: {}", if fell { "FELL" } else { "SURVIVED" });
}

#[cfg(not(feature = "mujoco"))]
fn main() {
    eprintln!("This example requires the `mujoco` feature. Run with:");
    eprintln!("  cargo run --features mujoco --example kyo46rs_com_squat");
}
