//! Opt-in control interval audit. No controller state is changed.
//! State endpoints span the whole tick. BipedRig::step refreshes MuJoCo with
//! forward(), so forces are post-state evaluations under the final held
//! command, not interval averages or substep peaks.
use super::rig::BipedRig;
use std::io::Write;

pub struct TickAudit {
    file: std::io::BufWriter<std::fs::File>,
    joints: Vec<(usize, usize, String)>,
}
impl TickAudit {
    pub fn create(path: &str, rig: &BipedRig) -> Self {
        let joints = rig
            .actuated()
            .iter()
            .map(|&(ji, _)| {
                let name = rig.robot.joints[ji].name.clone();
                let adr = rig.sim.joint_dof_adr(&name).expect("audit joint DOF");
                (ji, adr, name)
            })
            .collect();
        let mut file = std::io::BufWriter::new(std::fs::File::create(path).expect("audit file"));
        writeln!(file, "t_pre,t_post,force_sample_t,joint,q_pre,qd_pre,q_post,qd_post,tau_command,tau_actuator_post,qacc_post,tau_constraint_post,tau_passive_post,fz_pre_l,fz_pre_r,fz_qp_l,fz_qp_r,fz_post_l,fz_post_r").unwrap();
        Self { file, joints }
    }
    pub fn sample(&self, rig: &BipedRig) -> Vec<(f64, f64)> {
        self.joints
            .iter()
            .map(|(_, _, n)| rig.sim.joint_q_qd(n).expect("audit state"))
            .collect()
    }
    pub fn write(
        &mut self,
        rig: &BipedRig,
        t: f64,
        dt: f64,
        pre: &[(f64, f64)],
        command: &[f64],
        fz_pre: [f64; 2],
        fz_qp: [f64; 2],
        fz_post: [f64; 2],
    ) {
        let applied = rig.sim.qfrc_actuator();
        let acceleration = rig.sim.qacc();
        let constraint = rig.sim.qfrc_constraint();
        let passive = rig.sim.qfrc_passive();
        for ((ji, adr, name), &(q, qd)) in self.joints.iter().zip(pre) {
            let (post_q, post_qd) = rig.sim.joint_q_qd(name).expect("audit post state");
            writeln!(self.file, "{:.9},{:.9},{:.9},{name},{q:.12},{qd:.12},{post_q:.12},{post_qd:.12},{:.12},{:.12},{:.12},{:.12},{:.12},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9}",
                t, t+dt, t+dt, command[*ji], applied[*adr], acceleration[*adr], constraint[*adr], passive[*adr],
                fz_pre[0], fz_pre[1], fz_qp[0], fz_qp[1], fz_post[0], fz_post[1]).unwrap();
        }
    }
}
