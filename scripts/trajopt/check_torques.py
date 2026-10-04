"""Post-check of a push_recovery.py solution with pinocchio's full rigid-body
dynamics: joint torques tau = RNEA(q, v, a) - sum J_c^T f against the URDF
effort limits, and the unactuated (base) residual, which must vanish if the
centroidal solution is consistent with the full model.

    /data/articara/venv_trajopt/bin/python scripts/trajopt/check_torques.py /data/articara/trajopt/push7.json
"""
import json, sys
import numpy as np
import pinocchio as pin
from g1_casadi_model import G1Model, SOLE_OFFSET, SOLE_HALF, FOOT_LINKS

d = json.load(open(sys.argv[1]))
g = G1Model(); m = g.pin; data = m.createData()
Q, V, F, t = np.array(d['q']), np.array(d['v']), np.array(d['f']), np.array(d['t'])
N = len(F)
corners = [np.array([sx * SOLE_HALF[0], sy * SOLE_HALF[1], 0.0]) for sx in (1, -1) for sy in (1, -1)]
worst = np.zeros(g.nj); base_res = 0.0
for k in range(N):
    dt = t[k + 1] - t[k]
    q1, v1 = g.to_pin(Q[k + 1], V[k + 1])
    _, v0 = g.to_pin(Q[k], V[k])
    # The optimiser integrates momentum, not velocity; the acceleration here is
    # the finite difference of pinocchio's velocity over the interval.
    a = (v1 - v0) / dt
    tau = pin.rnea(m, data, q1, v1, a)
    pin.computeJointJacobians(m, data, q1); pin.framesForwardKinematics(m, data, q1)
    for s in (0, 1):
        jid = m.getJointId(FOOT_LINKS[s])
        for j in range(4):
            f = F[k, s, j]
            if not np.any(f):
                continue
            M = data.oMi[jid]
            off = SOLE_OFFSET + corners[j]
            J = pin.getJointJacobian(m, data, jid, pin.ReferenceFrame.LOCAL)
            Jp = M.rotation @ (J[:3] - pin.skew(off) @ J[3:])     # world velocity of the corner point
            tau -= Jp.T @ f
    base_res = max(base_res, np.abs(tau[:6]).max())
    worst = np.maximum(worst, np.abs(tau[6:]))
ratio = worst / g.taumax
print('impulse %.0f N s: base residual (should be ~0) %.1f N / N m' % (d['impulse'], base_res))
order = np.argsort(-ratio)[:8]
for i in order:
    print('  %-28s |tau| max %6.1f N m  limit %5.1f  ratio %.2f' % (g.names[i], worst[i], g.taumax[i], ratio[i]))
print('worst torque ratio %.2f' % ratio.max())
