"""CasADi rigid-body kinematics of the G1 built from the URDF through pinocchio.

The pip wheel of pinocchio has no CasADi bindings, so the symbolic model is
written out here from pinocchio's parsed tree (joint placements, axes, link
inertias) and checked NUMERICALLY against pinocchio in `validate()`.

Generalised coordinates (nq = 6 + nj):
    q = [base position (world, 3), base roll-pitch-yaw (3), joint angles (nj)]
    v = time derivative of q (rpy RATES, not body angular velocity)

Provides, as CasADi expressions of (q, v):
    com(q), link/sole positions and rotations, centroidal momentum h(q, v)
    (linear = M * com velocity, angular about the CoM, world frame).

Run with the trajopt venv:
    /data/articara/venv_trajopt/bin/python scripts/trajopt/g1_casadi_model.py
"""
import numpy as np
import casadi as ca
import pinocchio as pin

import os
URDF = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'log/claude_g1champion_20261004v64/g1_23dof_arms_free.urdf')
# Sole frame of the walker (src/biped/profile.rs, G1): centre 35 mm ahead of
# the ankle-roll origin and 35 mm below it; half-length 85 mm, half-width 30.
SOLE_OFFSET = np.array([0.035, 0.0, -0.035])
SOLE_HALF = (0.085, 0.030)
FOOT_LINKS = ('left_ankle_roll_joint', 'right_ankle_roll_joint')


def _rot(axis, angle):
    """Rodrigues rotation about a constant unit axis by a symbolic angle."""
    a = np.asarray(axis, dtype=float)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return ca.DM.eye(3) + ca.sin(angle) * ca.DM(K) + (1 - ca.cos(angle)) * ca.DM(K @ K)


def rpy_matrix(rpy):
    r, p, y = rpy[0], rpy[1], rpy[2]
    return ca.mtimes([_rot([0, 0, 1], y), _rot([0, 1, 0], p), _rot([1, 0, 0], r)])


def rpy_rate_to_omega(rpy, drpy):
    """World angular velocity from roll-pitch-yaw rates (ZYX convention)."""
    r, p, y = rpy[0], rpy[1], rpy[2]
    Rz = _rot([0, 0, 1], y)
    Ry = _rot([0, 1, 0], p)
    ez = ca.DM([0, 0, 1])
    ey = ca.mtimes(Rz, ca.DM([0, 1, 0]))
    ex = ca.mtimes([Rz, Ry, ca.DM([1, 0, 0])])
    return ez * drpy[2] + ey * drpy[1] + ex * drpy[0]


class G1Model:
    def __init__(self, urdf=URDF):
        self.pin = pin.buildModelFromUrdf(urdf, pin.JointModelFreeFlyer())
        m = self.pin
        self.names = [m.names[i] for i in range(2, m.njoints)]
        self.nj = len(self.names)
        self.nq = 6 + self.nj
        self.mass = pin.computeTotalMass(m)
        self.lower = np.array([m.lowerPositionLimit[7 + k] for k in range(self.nj)])
        self.upper = np.array([m.upperPositionLimit[7 + k] for k in range(self.nj)])
        self.vmax = np.array([m.velocityLimit[6 + k] for k in range(self.nj)])
        self.taumax = np.array([m.effortLimit[6 + k] for k in range(self.nj)])
        # Joint axes in the joint frame.
        self.axes = []
        for i in range(2, m.njoints):
            jm = m.joints[i]
            sn = jm.shortname()
            if sn.endswith('RX'):
                ax = [1, 0, 0]
            elif sn.endswith('RY'):
                ax = [0, 1, 0]
            elif sn.endswith('RZ'):
                ax = [0, 0, 1]
            else:  # RevoluteUnaligned
                ax = list(np.asarray(jm.extract().axis).ravel())
            self.axes.append(np.array(ax, dtype=float))

    # -- symbolic kinematics -------------------------------------------------
    def frames(self, q):
        """World rotation and origin of every joint frame (index 0 = base)."""
        m = self.pin
        R = [None] * m.njoints
        p = [None] * m.njoints
        R[1] = rpy_matrix(q[3:6])
        p[1] = q[0:3]
        for i in range(2, m.njoints):
            par = m.parents[i]
            pl = m.jointPlacements[i]
            Rj = ca.mtimes(R[par], ca.DM(pl.rotation))
            p[i] = p[par] + ca.mtimes(R[par], ca.DM(pl.translation))
            R[i] = ca.mtimes(Rj, _rot(self.axes[i - 2], q[6 + i - 2]))
        return R, p

    def com(self, q):
        m = self.pin
        R, p = self.frames(q)
        c = ca.MX.zeros(3) if isinstance(q, ca.MX) else ca.SX.zeros(3)
        for i in range(1, m.njoints):
            I = m.inertias[i]
            c += I.mass * (p[i] + ca.mtimes(R[i], ca.DM(I.lever)))
        return c / self.mass

    def sole(self, q, side):
        """(rotation, position of the sole centre) of foot `side` (0 = left)."""
        m = self.pin
        R, p = self.frames(q)
        i = m.getJointId(FOOT_LINKS[side])
        return R[i], p[i] + ca.mtimes(R[i], ca.DM(SOLE_OFFSET))

    def centroidal_momentum(self, q, v):
        """(linear, angular about the CoM), world frame."""
        m = self.pin
        R, p = self.frames(q)
        sym = ca.MX if isinstance(q, ca.MX) else ca.SX
        # Angular velocity of every joint frame.
        w = [None] * m.njoints
        w[1] = rpy_rate_to_omega(q[3:6], v[3:6])
        for i in range(2, m.njoints):
            w[i] = w[m.parents[i]] + ca.mtimes(R[i], ca.DM(self.axes[i - 2])) * v[6 + i - 2]
        c = self.com(q)
        cs, ms = [], []
        lin = sym.zeros(3)
        ang = sym.zeros(3)
        vc = ca.jtimes(c, q, v)
        for i in range(1, m.njoints):
            I = m.inertias[i]
            ci = p[i] + ca.mtimes(R[i], ca.DM(I.lever))
            vi = ca.jtimes(ci, q, v)
            lin += I.mass * vi
            Iw = ca.mtimes([R[i], ca.DM(I.inertia), R[i].T])
            ang += I.mass * ca.cross(ci - c, vi - vc) + ca.mtimes(Iw, w[i])
        return lin, ang

    # -- conversion to pinocchio's coordinates (for validation) ----------------
    def to_pin(self, q, v):
        Rb = np.array(ca.DM(rpy_matrix(ca.DM(q[3:6]))))
        quat = pin.Quaternion(Rb).coeffs()
        qp = np.concatenate([q[0:3], quat, q[6:]])
        om = np.array(ca.DM(rpy_rate_to_omega(ca.DM(q[3:6]), ca.DM(v[3:6])))).ravel()
        vp = np.concatenate([Rb.T @ v[0:3], Rb.T @ om, v[6:]])
        return qp, vp


def validate(n=20, seed=0):
    g = G1Model()
    q = ca.SX.sym('q', g.nq)
    v = ca.SX.sym('v', g.nq)
    lin, ang = g.centroidal_momentum(q, v)
    Rl, pl = g.sole(q, 0)
    Rr, pr = g.sole(q, 1)
    f = ca.Function('f', [q, v], [g.com(q), lin, ang, pl, pr, Rl, Rr])
    rng = np.random.default_rng(seed)
    m = g.pin
    d = m.createData()
    worst = np.zeros(5)
    for _ in range(n):
        qn = np.concatenate([rng.uniform(-0.3, 0.3, 3) + [0, 0, 0.75], rng.uniform(-0.4, 0.4, 3),
                             rng.uniform(g.lower, g.upper) * 0.6])
        vn = rng.uniform(-1.0, 1.0, g.nq)
        com, lin_n, ang_n, pl_n, pr_n, Rl_n, Rr_n = [np.array(x) for x in f(qn, vn)]
        qp, vp = g.to_pin(qn, vn)
        pin.forwardKinematics(m, d, qp, vp)
        pin.centerOfMass(m, d, qp, vp)
        h = pin.computeCentroidalMomentum(m, d, qp, vp)
        errs = [np.abs(com.ravel() - d.com[0]).max(),
                np.abs(lin_n.ravel() - h.linear).max(),
                np.abs(ang_n.ravel() - h.angular).max()]
        for side, (pn, Rn) in enumerate([(pl_n, Rl_n), (pr_n, Rr_n)]):
            M = d.oMi[m.getJointId(FOOT_LINKS[side])]
            errs.append(max(np.abs(pn.ravel() - (M.translation + M.rotation @ SOLE_OFFSET)).max(),
                            np.abs(Rn - M.rotation).max()))
        worst = np.maximum(worst, errs)
    print(f'mass {g.mass:.3f} kg, nj {g.nj}')
    print('max |error| over %d random states: com %.2e m, lin %.2e, ang %.2e, sole L %.2e, sole R %.2e' % (n, *worst))
    assert worst.max() < 1e-9, 'CasADi model disagrees with pinocchio'
    print('OK: CasADi kinematics and centroidal momentum match pinocchio')


if __name__ == '__main__':
    validate()
