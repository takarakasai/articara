"""Replay a push_recovery.py solution in MuJoCo: joint PD on the optimised
joint trajectory plus the inverse-dynamics feed-forward torque, floating base
free, starting from the optimiser's (pushed) initial state. No balance
feedback at all -- this asks only whether the trajectory is physically
executable and what the bodies do (self-contact, slip, landing).

    /data/articara/venv_trajopt/bin/python scripts/trajopt/replay_mujoco.py /data/articara/trajopt/push7c.json
"""
import json, os, sys
import numpy as np
import mujoco
import pinocchio as pin
from g1_casadi_model import G1Model, URDF, SOLE_OFFSET, SOLE_HALF, FOOT_LINKS

d = json.load(open(sys.argv[1]))
kp = float(os.environ.get('KP', 300)); kd = float(os.environ.get('KD', 10)); hold = float(os.environ.get('HOLD', 1.0))
mu = float(os.environ.get('MU', 0.7))
# Crude balance feedback (BAL_KP > 0): a virtual force on the CoM and a
# torque on the pelvis, proportional to the error against the PLAN, pushed
# through the stance leg(s) as joint torques (tau = -J_foot^T F). This is a
# stand-in for a tracking controller, not the walker's WBC.
bal_kp = float(os.environ.get('BAL_KP', 0)); bal_kd = float(os.environ.get('BAL_KD', 0))
bal_kr = float(os.environ.get('BAL_KR', 0)); bal_kw = float(os.environ.get('BAL_KW', 0))
g = G1Model(); pm = g.pin; pdata = pm.createData()
Q, V, F, t = np.array(d['q']), np.array(d['v']), np.array(d['f']), np.array(d['t'])
N = len(F)

# ---- MuJoCo model from the URDF: free base, floor, torque motors ----------
spec = mujoco.MjSpec.from_file(os.path.abspath(URDF))
# Only the primitive collision geoms (the sole spheres) are needed; the visual
# meshes are not shipped next to the champion URDF.
for geom in list(spec.geoms):
    if geom.type == mujoco.mjtGeom.mjGEOM_MESH:
        spec.delete(geom)
for mesh in list(spec.meshes):
    spec.delete(mesh)
root = spec.worldbody.first_body()
root.add_freejoint()
spec.worldbody.add_geom(type=mujoco.mjtGeom.mjGEOM_PLANE, size=[5, 5, 0.1], friction=[mu, 0.005, 0.0001])
for n in g.names:
    spec.add_actuator(name='m_' + n, target=n, trntype=mujoco.mjtTrn.mjTRN_JOINT, gainprm=[1] + [0] * 9, biastype=mujoco.mjtBias.mjBIAS_NONE)
for geom in spec.geoms:
    geom.friction = [mu, 0.005, 0.0001]
spec.option.timestep = 0.0005
model = spec.compile()
data = mujoco.MjData(model)
jadr = [model.joint(n).qposadr[0] for n in g.names]
dadr = [model.joint(n).dofadr[0] for n in g.names]

def set_state(q, v):
    qp, vp = g.to_pin(q, v)
    data.qpos[0:3] = qp[0:3]
    data.qpos[3:7] = [qp[6], qp[3], qp[4], qp[5]]          # MuJoCo: w x y z
    data.qvel[0:3] = q_world_lin(q, v)                     # MuJoCo free joint: linear velocity in WORLD
    data.qvel[3:6] = vp[3:6]                               # angular in the body frame
    for k in range(g.nj):
        data.qpos[jadr[k]] = q[6 + k]; data.qvel[dadr[k]] = v[6 + k]

def q_world_lin(q, v):
    return v[0:3]

# ---- feed-forward torques per interval from pinocchio ----------------------
corners = [np.array([sx * SOLE_HALF[0], sy * SOLE_HALF[1], 0.0]) for sx in (1, -1) for sy in (1, -1)]
tau_ff = np.zeros((N, g.nj))
for k in range(N):
    dt = t[k + 1] - t[k]
    q1, v1 = g.to_pin(Q[k + 1], V[k + 1]); _, v0 = g.to_pin(Q[k], V[k])
    tau = pin.rnea(pm, pdata, q1, v1, (v1 - v0) / dt)
    pin.computeJointJacobians(pm, pdata, q1)
    for s in (0, 1):
        jid = pm.getJointId(FOOT_LINKS[s])
        for j in range(4):
            f = F[k, s, j]
            if np.any(f):
                M = pdata.oMi[jid]
                J = pin.getJointJacobian(pm, pdata, jid, pin.ReferenceFrame.LOCAL)
                tau -= (M.rotation @ (J[:3] - pin.skew(SOLE_OFFSET + corners[j]) @ J[3:])).T @ f
    tau_ff[k] = np.clip(tau[6:], -g.taumax, g.taumax)

com_ref = np.array(d['com'])
set_state(Q[0], V[0])
mujoco.mj_forward(model, data)
foot_body = [model.body(n.replace('_joint', '_link')).id for n in FOOT_LINKS]
log = []
self_contacts = {}
T_end = t[-1] + hold
while data.time < T_end:
    tt = data.time
    k = min(np.searchsorted(t, tt, side='right') - 1, N - 1)
    if tt < t[-1]:
        a = (tt - t[k]) / (t[k + 1] - t[k])
        qr = (1 - a) * Q[k, 6:] + a * Q[k + 1, 6:]; vr = V[k + 1, 6:]; ff = tau_ff[k]
    else:
        qr = Q[-1, 6:]; vr = np.zeros(g.nj); ff = tau_ff[-1]
    qj = np.array([data.qpos[i] for i in jadr]); vj = np.array([data.qvel[i] for i in dadr])
    tau_cmd = ff + kp * (qr - qj) + kd * (vr - vj)
    if bal_kp > 0 and tt < t[-1] + hold:
        kk = min(k, N - 1)
        a_ = min(max((tt - t[kk]) / (t[kk + 1] - t[kk]), 0.0), 1.0) if tt < t[-1] else 1.0
        c_ref = (1 - a_) * com_ref[kk] + a_ * com_ref[kk + 1] if tt < t[-1] else com_ref[-1]
        cdot_ref = (com_ref[kk + 1] - com_ref[kk]) / (t[kk + 1] - t[kk]) if tt < t[-1] else np.zeros(3)
        c = data.subtree_com[1].copy(); cdot = data.subtree_linvel[1].copy() if hasattr(data, 'subtree_linvel') else np.zeros(3)
        Fv = bal_kp * (c_ref - c) + bal_kd * (cdot_ref - cdot)
        rpy_ref = (1 - a_) * Q[kk, 3:6] + a_ * Q[kk + 1, 3:6] if tt < t[-1] else Q[-1, 3:6]
        Rm = data.xmat[1].reshape(3, 3); Rr = np.array(pin.rpy.rpyToMatrix(*rpy_ref))
        e_rot = pin.log3(Rr @ Rm.T)                       # world-frame rotation error
        w_body = Rm @ data.qvel[3:6]
        Mv = bal_kr * e_rot - bal_kw * w_body
        # Stance feet = those the plan loads in this interval (both when standing).
        stance = [s_ for s_ in (0, 1) if (tt >= t[-1]) or np.any(F[kk, s_])]
        q_now = np.concatenate([data.qpos[0:3], np.zeros(3), qj]); 
        qp_now = np.concatenate([data.qpos[0:3], data.qpos[4:7], data.qpos[3:4], qj])
        pin.computeJointJacobians(pm, pdata, qp_now); pin.framesForwardKinematics(pm, pdata, qp_now)
        for s_ in stance:
            jid = pm.getJointId(FOOT_LINKS[s_]); Mf = pdata.oMi[jid]
            J = pin.getJointJacobian(pm, pdata, jid, pin.ReferenceFrame.LOCAL)
            Jw = np.vstack([Mf.rotation @ (J[:3] - pin.skew(SOLE_OFFSET) @ J[3:]), Mf.rotation @ J[3:]])  # world lin/ang of the sole
            # A wrench on the body with the foot fixed = the opposite wrench applied at the foot.
            tau_cmd -= (Jw[:, 6:].T @ np.concatenate([Fv, Mv])) / len(stance)
    data.ctrl[:] = np.clip(tau_cmd, -g.taumax, g.taumax)
    mujoco.mj_step(model, data)
    for c in data.contact[:data.ncon]:
        b1, b2 = model.geom_bodyid[c.geom1], model.geom_bodyid[c.geom2]
        if b1 != 0 and b2 != 0:
            key = tuple(sorted((model.body(b1).name, model.body(b2).name)))
            self_contacts[key] = self_contacts.get(key, 0) + 1
    if int(round(data.time / model.opt.timestep)) % 40 == 0:
        R = data.xmat[1].reshape(3, 3)
        log.append((data.time, *data.subtree_com[1], np.degrees(np.arccos(np.clip(R[2, 2], -1, 1))),
                    *[data.xpos[b][2] for b in foot_body], *[data.xpos[b][1] for b in foot_body]))
log = np.array(log)
print('impulse %.0f N s, kp %.0f kd %.0f, friction %.2f, balance kp %.0f kd %.0f kr %.0f kw %.0f' % (d['impulse'], kp, kd, mu, bal_kp, bal_kd, bal_kr, bal_kw))
print('   t    com_x  com_y  com_z  tilt[deg]  ankleL_z ankleR_z  ankleL_y ankleR_y')
for row in log[::max(1, len(log) // 24)]:
    print('%5.2f  %6.3f %6.3f %6.3f  %6.1f     %6.3f   %6.3f   %6.3f   %6.3f' % tuple(row))
tilt_end = log[-1, 4]
print('trajectory ends at %.2f s; at %.2f s: tilt %.1f deg, CoM z %.3f -> %s' % (t[-1], T_end, tilt_end, log[-1, 3],
      'STANDING' if tilt_end < 15 and log[-1, 3] > 0.5 else 'FELL'))
i_end = np.searchsorted(log[:, 0], t[-1])
print('at the end of the trajectory (%.2f s): tilt %.1f deg, CoM (%.3f, %.3f, %.3f); optimiser CoM (%.3f, %.3f, %.3f)' % (
    t[-1], log[min(i_end, len(log) - 1), 4], *log[min(i_end, len(log) - 1), 1:4], *d['com'][-1]))
print('self-contacts (body pairs, step counts):', self_contacts if self_contacts else 'none')
