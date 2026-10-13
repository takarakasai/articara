"""Kinodynamic trajectory optimisation of a push recovery on the G1.

Model: centroidal dynamics + full kinematics (Dai, Valenzuela & Tedrake 2014;
"simple dynamics, full kinematics"). Decision variables per node: q, v
(base position, roll-pitch-yaw, 22 joints) and the forces at the four corners
of every sole in contact. The centroidal momentum of the full model must
follow the contact wrench; the feet in contact must stay put and flat.

Scenario (first question of doc/from_claude/20261005_trajopt_reference_plan.md):
the robot stands in single support on the LEFT foot, the right foot in mid
swing, and receives a lateral impulse TOWARD THE STANCE SIDE (+y). Schedule:
SS(left) -> DS -> SS(right) -> DS, phase durations free. Is there ANY
trajectory within the joint, friction and collision limits that ends at rest?

    /data/articara/venv_trajopt/bin/python scripts/trajopt/push_recovery.py --impulse 7
"""
import argparse, json, sys, time
import numpy as np
import casadi as ca
from g1_casadi_model import G1Model, SOLE_HALF

p = argparse.ArgumentParser()
p.add_argument('--impulse', type=float, default=7.0, help='lateral impulse toward the stance side [N s]')
p.add_argument('--mu', type=float, default=0.6)
p.add_argument('--zcom', type=float, default=0.684)
p.add_argument('--width', type=float, default=0.105, help='stance width [m]')
p.add_argument('--nodes', type=str, default='10,4,10,6')
p.add_argument('--tmin', type=str, default='0.20,0.05,0.25,0.20')
p.add_argument('--tmax', type=str, default='0.80,0.40,0.80,0.80')
p.add_argument('--vscale', type=float, default=1.0, help='fraction of the URDF joint velocity limits allowed')
p.add_argument('--clear', type=float, default=0.11, help='leg-leg clearance between sampled centre-line points [m]')
p.add_argument('--tau-scale', type=float, default=0.9, help='joint torque bound as a fraction of the URDF effort limits (0 = no torque constraint)')
p.add_argument('--swing-z', type=float, default=0.05, help='initial height of the swing (right) sole [m]')
p.add_argument('--lmax', type=float, default=0.0, help='bound on |angular momentum| about the CoM [N m s] (0 = none)')
p.add_argument('--swing-ymin', type=float, default=-9.0, help='lower bound on the swing (right) sole y during the first phase [m] (no outward wind-up)')
p.add_argument('--phase', type=float, default=0.5, help='swing phase of the walker this reference is meant for (stored in the JSON for library selection)')
p.add_argument('--end-width', type=float, default=0.0, help='end with the feet at this stance width and nearly side by side (0 = free), so the gait can resume from its nominal stance')
p.add_argument('--cross', type=float, default=-9.0, help='force a crossover: the first landing of the swing (right) foot at least this far to the +y side of the stance foot [m]')
p.add_argument('--cross-x', type=float, default=-9.0, help='with --cross: the first landing also at least this far AHEAD of the stance foot [m] (cross in front, not behind)')
p.add_argument('--delay', type=float, default=0.0, help='detection delay [s]: the reference starts this long after the push, with the CoM already displaced by vy * delay')
p.add_argument('--impulse-deg', type=float, default=90.0, help='direction of the impulse in the horizontal plane [deg]: 90 = +y toward the stance (left) foot, 0 = forward')
p.add_argument('--vx', type=float, default=0.0, help='forward CoM speed at the push [m/s] (walking scenario)')
p.add_argument('--swing-x', type=float, default=0.0, help='initial x of the swing (right) sole [m] (walking scenario: mid-swing ahead)')
p.add_argument('--lbfgs', action='store_true', help='limited-memory Hessian in IPOPT (the exact Hessian of the torque constraints takes ~28 GB)')
p.add_argument('--landed', action='store_true', help='the swing (right) foot starts ON the ground, unloaded, at (--swing-x, -width/2): the re-step case (doc/from_claude/20261012_restep_design.md). Needs --swing-z 0')
p.add_argument('--com-x', type=float, default=None, help='override the initial CoM x [m] (default: (vx + dvx) * delay)')
p.add_argument('--com-y', type=float, default=None, help='override the initial CoM y [m] (default: 0.7 * width/2 + dvy * delay)')
p.add_argument('--vy', type=float, default=None, help='override the initial lateral CoM speed [m/s] (default: the impulse / mass)')
p.add_argument('--land-xmax', type=float, default=-9.0, help='upper bound on the x of both landings relative to the stance foot [m] (keep the feet from running ahead of a CoM the walker will not push)')
p.add_argument('--end-vx', type=float, default=None, help='forward CoM speed at the END [m/s] instead of coming to rest (hand back to a walking gait); the CoM then ends a little behind the feet midpoint')
p.add_argument('--end-over', type=int, default=None, help='end with the CoM over this foot (0 = left/stance, 1 = right) instead of the feet midpoint: the gait that resumes swings the OTHER foot first')
p.add_argument('--out', type=str, default='')
p.add_argument('--max-iter', type=int, default=1500)
args = p.parse_args()

g = G1Model()
nq, nj, M = g.nq, g.nj, g.mass
G = 9.81
qs, vs = ca.SX.sym('q', nq), ca.SX.sym('v', nq)
lin, ang = g.centroidal_momentum(qs, vs)
f_mom = ca.Function('mom', [qs, vs], [ca.vertcat(lin, ang)])
f_com = ca.Function('com', [qs], [g.com(qs)])
soles = [g.sole(qs, s) for s in (0, 1)]
f_sole = [ca.Function(f'sole{s}', [qs], [soles[s][1], soles[s][0]]) for s in (0, 1)]
R_fr, p_fr = g.frames(qs)
knee_id = [g.pin.getJointId(n) for n in ('left_knee_joint', 'right_knee_joint')]
f_knee = ca.Function('knee', [qs], [p_fr[knee_id[0]], p_fr[knee_id[1]]])
# Leg capsules as point chains: hip, mid-thigh, knee, mid-shank, ankle per leg.
def _chain(side):
    ids = [g.pin.getJointId(f'{side}_{n}_joint') for n in ('hip_pitch', 'knee', 'ankle_roll')]
    h, k, a_ = (p_fr[i] for i in ids)
    return [0.5 * (h + k), k, 0.67 * k + 0.33 * a_, 0.33 * k + 0.67 * a_, a_]
f_legs = ca.Function('legs', [qs], [ca.horzcat(*_chain('left')), ca.horzcat(*_chain('right'))])
corners = [np.array([sx * SOLE_HALF[0], sy * SOLE_HALF[1], 0.0]) for sx in (1, -1) for sy in (1, -1)]
# Joint torques of the full model: Lagrangian inverse dynamics minus the
# contact forces mapped through the corner Jacobians (validated against
# pinocchio's RNEA in g1_casadi_model.py).
_as, _fs = ca.SX.sym('a', nq), ca.SX.sym('f', 24)
_cpos = ca.vertcat(*[soles[s][1] + ca.mtimes(soles[s][0], ca.DM(c)) for s in (0, 1) for c in corners])
_tau = g.inverse_dynamics(qs, vs, _as) - ca.mtimes(ca.jacobian(_cpos, qs).T, _fs)
f_tau = ca.Function('tau', [qs, vs, _as, _fs], [_tau[6:]])

# ---- standing pose by a small NLP: both soles flat, CoM at height, arms 0 ----
def standing_pose(com_xy, right_z, right_x=0.0):
    o = ca.Opti()
    q = o.variable(nq)
    w = args.width / 2
    for s, (x, y, z) in enumerate([(0.0, w, 0.0), (right_x, -w, right_z)]):
        ps, Rs = f_sole[s](q)
        o.subject_to(ps == ca.DM([x, y, z]))
        o.subject_to(Rs[2, 0] == 0); o.subject_to(Rs[2, 1] == 0); o.subject_to(Rs[1, 0] == 0)
    c = f_com(q)
    o.subject_to(c == ca.DM([com_xy[0], com_xy[1], args.zcom]))
    o.subject_to(q[3:6] == 0)
    o.subject_to(o.bounded(g.lower, q[6:], g.upper))
    arms = [k for k, n in enumerate(g.names) if 'shoulder' in n or 'elbow' in n or 'wrist' in n]
    o.minimize(ca.sumsqr(q[6:][arms]) + 1e-2 * ca.sumsqr(q[6:]))
    q0 = np.zeros(nq); q0[2] = 0.75
    for k, n in enumerate(g.names):
        if 'hip_pitch' in n: q0[6 + k] = -0.3
        if 'knee' in n: q0[6 + k] = 0.6
        if 'ankle_pitch' in n: q0[6 + k] = -0.3
    o.set_initial(q, q0)
    o.solver('ipopt', {'print_time': False}, {'print_level': 0, 'tol': 1e-10})
    return np.array(o.solve().value(q))

w = args.width / 2
th = np.radians(args.impulse_deg)
vx_imp, vy_pre = args.impulse / g.mass * np.cos(th), args.impulse / g.mass * np.sin(th)
com0 = [(args.vx + vx_imp) * args.delay, w * 0.7 + vy_pre * args.delay]   # CoM mostly over the left (stance) foot, plus the drift during the detection delay
if args.com_x is not None: com0[0] = args.com_x
if args.com_y is not None: com0[1] = args.com_y
q_init = standing_pose(com0, args.swing_z, args.swing_x)
q_stand = standing_pose((0.0, 0.0), 0.0)
vy = vy_pre if args.vy is None else args.vy
omega = np.sqrt(G / args.zcom)
print(f'impulse {args.impulse} N s at {args.impulse_deg:.0f} deg -> dvx {vx_imp:.3f} dvy {vy:.3f} m/s; DCM y = {w*0.7 + vy/omega:.3f} m, stance outer edge {w + SOLE_HALF[1]:.3f} m')

# ---- the trajectory optimisation -------------------------------------------
n_ph = [int(x) for x in args.nodes.split(',')]
tmin = [float(x) for x in args.tmin.split(',')]
tmax = [float(x) for x in args.tmax.split(',')]
# (left, right) per phase: SS(left) -> DS -> SS(right) -> DS -> SS(left) -> DS ... (as many phases as --nodes gives)
contact = [[(True, False), (True, True), (False, True), (True, True)][i % 4] for i in range(len(n_ph))]
N = sum(n_ph)
phase_of = sum(([i] * n for i, n in enumerate(n_ph)), [])

o = ca.Opti()
Q = [o.variable(nq) for _ in range(N + 1)]
V = [o.variable(nq) for _ in range(N + 1)]
T = [o.variable() for _ in n_ph]
F = [[[o.variable(3) for _ in range(4)] if contact[phase_of[k]][s] else None for s in (0, 1)] for k in range(N)]
for i, t in enumerate(T):
    o.subject_to(o.bounded(tmin[i], t, tmax[i]))
    o.set_initial(t, 0.5 * (tmin[i] + tmax[i]))

# Initial velocity: the push gives the CoM the lateral speed, but the stance
# sole is on the ground and does not move. Minimum-norm generalised velocity
# with (stance sole corners at rest) and (linear momentum = M * [0, vy, 0]).
# (The first version moved the whole body, stance foot included, at vy -- an
# impact at t = 0 that the MuJoCo replay exposed.)
_pl, _Rl = soles[0][1], soles[0][0]
_cs = ca.vertcat(*[_pl + ca.mtimes(_Rl, ca.DM(c)) for c in corners[:3]])
_A = ca.Function('A', [qs], [ca.vertcat(ca.jacobian(_cs, qs), ca.jacobian(f_mom(qs, vs)[0:3], vs))])
A0 = np.array(_A(q_init))
b0 = np.concatenate([np.zeros(9), [M * (args.vx + vx_imp), M * vy, 0.0]])
v0 = np.linalg.pinv(A0) @ b0
print('initial velocity: |A v0 - b| = %.1e, base lin %s, max joint speed %.2f rad/s' % (
    np.abs(A0 @ v0 - b0).max(), np.round(v0[0:3], 3), np.abs(v0[6:]).max()))
o.subject_to(Q[0] == q_init)
o.subject_to(V[0] == v0)
if args.end_vx is None:
    o.subject_to(V[N] == 0)
else:
    # Joint velocities at rest, the base moving forward at --end-vx.
    o.subject_to(V[N][6:] == 0)
    o.subject_to(V[N][1:6] == 0)
    o.subject_to(V[N][0] == args.end_vx)

cost = 0
anchor = {}   # (side, phase) -> (position, rotation) at the first node of that contact phase
for k in range(N + 1):
    ph = phase_of[min(k, N - 1)]
    q, v = Q[k], V[k]
    o.subject_to(o.bounded(g.lower, q[6:], g.upper))
    o.subject_to(o.bounded(-g.vmax * args.vscale, v[6:], g.vmax * args.vscale))
    o.subject_to(o.bounded(-0.6, q[3:6], 0.6))
    sp = [f_sole[s](q) for s in (0, 1)]
    for s in (0, 1):
        ps, Rs = sp[s]
        # A node on a phase boundary belongs to both phases for the contact.
        phs = {ph} | ({phase_of[k - 1]} if 0 < k <= N else set())
        in_contact = any(contact[x][s] for x in phs)
        if in_contact:
            o.subject_to(ps[2] == 0)
            o.subject_to(Rs[2, 0] == 0); o.subject_to(Rs[2, 1] == 0)
            # Same footprint for the whole stretch this foot stays down.
            key = s
            if key in anchor and anchor[key][2] == k - 1:
                o.subject_to(ps[0:2] == anchor[key][0][0:2])
                o.subject_to(Rs[1, 0] == anchor[key][1][1, 0])
            anchor[key] = (ps, Rs, k)
        elif args.landed and s == 1 and k == 0:
            # Re-step: the foot starts on the ground (unloaded) and lifts.
            anchor.pop(s, None)
        else:
            o.subject_to(ps[2] >= 0.02)
            anchor.pop(s, None)
    if args.swing_ymin > -1.0 and k <= n_ph[0]:
        o.subject_to(sp[1][0][1] >= args.swing_ymin)
    # Self-collision, crude: the soles must not overlap in plan, the knees keep apart.
    d = sp[0][0] - sp[1][0]
    o.subject_to((d[0] / 0.20) ** 2 + (d[1] / 0.085) ** 2 >= 1.0)
    kl, kr = f_knee(q)
    o.subject_to(ca.sumsqr(kl - kr) >= 0.11 ** 2)
    # Leg-leg clearance: every sampled point of one leg at least `--clear`
    # from every sampled point of the other (thigh, knee, shank, ankle).
    LL, RR = f_legs(q)
    for i in range(5):
        for j in range(5):
            o.subject_to(ca.sumsqr(LL[:, i] - RR[:, j]) >= args.clear ** 2)

for k in range(N):
    ph = phase_of[k]
    dt = T[ph] / n_ph[ph]
    q0_, v0_, q1, v1 = Q[k], V[k], Q[k + 1], V[k + 1]
    o.subject_to(q1 == q0_ + dt * v1)
    h0, h1 = f_mom(q0_, v0_), f_mom(q1, v1)
    c1 = f_com(q1)
    force = ca.MX.zeros(3); moment = ca.MX.zeros(3)
    for s in (0, 1):
        if F[k][s] is None:
            continue
        ps, Rs = f_sole[s](q1)
        for j, fc in enumerate(F[k][s]):
            o.subject_to(fc[2] >= 0)
            o.subject_to(o.bounded(-args.mu * fc[2], fc[0], args.mu * fc[2]))
            o.subject_to(o.bounded(-args.mu * fc[2], fc[1], args.mu * fc[2]))
            o.set_initial(fc, [0, 0, M * G / 8])
            r = ps + ca.mtimes(Rs, ca.DM(corners[j]))
            force += fc
            moment += ca.cross(r - c1, fc)
            cost += 1e-5 * dt * ca.sumsqr(fc)
    if args.tau_scale > 0:
        fvec = ca.vertcat(*[(F[k][s_][j] if F[k][s_] is not None else ca.DM.zeros(3)) for s_ in (0, 1) for j in range(4)])
        tau_k = f_tau(q1, v1, (v1 - v0_) / dt, fvec)
        o.subject_to(o.bounded(-g.taumax * args.tau_scale, tau_k, g.taumax * args.tau_scale))
        cost += dt * 1e-5 * ca.sumsqr(tau_k)
    o.subject_to(h1[0:3] == h0[0:3] + dt * (force + ca.DM([0, 0, -M * G])))
    o.subject_to(h1[3:6] == h0[3:6] + dt * moment)
    if args.lmax > 0:
        o.subject_to(ca.sumsqr(h1[3:6]) <= args.lmax ** 2)
    cost += dt * (1e-2 * ca.sumsqr(v1[6:]) + 1e-1 * ca.sumsqr(v1[3:6]) + 1e-3 * ca.sumsqr((v1 - v0_) / dt) * 1e-2)
    cost += dt * 1e-1 * ca.sumsqr(q1[6:] - q_stand[6:])

# End: at rest, CoM between the feet at standing height.
pl, _ = f_sole[0](Q[N]); pr, _ = f_sole[1](Q[N])
cN = f_com(Q[N])
if args.end_over is not None:
    p_over = pl if args.end_over == 0 else pr
    vx_end = 0.0 if args.end_vx is None else args.end_vx
    o.subject_to(cN[0] + vx_end / omega == p_over[0])
    o.subject_to(cN[1] == p_over[1])
elif args.end_vx is None:
    o.subject_to(cN[0:2] == 0.5 * (pl[0:2] + pr[0:2]))
else:
    # Walking hand-back: the DCM (not the CoM) at the feet midpoint.
    o.subject_to(cN[0] + args.end_vx / omega == 0.5 * (pl[0] + pr[0]))
    o.subject_to(cN[1] == 0.5 * (pl[1] + pr[1]))
if args.cross > -1.0:
    pr1, _ = f_sole[1](Q[n_ph[0]])
    o.subject_to(pr1[1] >= w + args.cross)
    if args.cross_x > -1.0:
        o.subject_to(pr1[0] >= args.cross_x)
if args.land_xmax > -1.0:
    pr1, _ = f_sole[1](Q[n_ph[0]])
    pl2, _ = f_sole[0](Q[n_ph[0] + n_ph[1] + n_ph[2]])
    o.subject_to(pr1[0] <= args.land_xmax)
    o.subject_to(pl2[0] <= args.land_xmax)
if args.end_width > 0:
    # Hand back to the gait from its nominal stance: left foot a stance width
    # to the left of the right one and at most 8 cm ahead/behind it.
    o.subject_to(pl[1] - pr[1] == args.end_width)
    o.subject_to(o.bounded(-0.08, pl[0] - pr[0], 0.08))
o.subject_to(o.bounded(args.zcom - 0.08, cN[2], args.zcom + 0.02))
cost += 1.0 * ca.sumsqr(Q[N][3:6])

o.minimize(cost)
for k in range(N + 1):
    a = k / N
    o.set_initial(Q[k], (1 - a) * q_init + a * q_stand)
    o.set_initial(V[k], np.zeros(nq))
ipopt_opts = {'max_iter': args.max_iter, 'tol': 1e-5, 'acceptable_tol': 1e-3, 'acceptable_iter': 10, 'print_level': 3, 'linear_solver': 'mumps'}
if args.lbfgs:
    ipopt_opts['hessian_approximation'] = 'limited-memory'
o.solver('ipopt', {'print_time': False}, ipopt_opts)
t0 = time.time()
try:
    sol = o.solve()
    ok = True
except RuntimeError as e:
    sol = o.debug
    ok = False
    print('NOT SOLVED:', str(e).splitlines()[-1][:120])
print(f'solve time {time.time() - t0:.1f} s, status {o.stats().get("return_status")}')

Tn = [float(sol.value(t)) for t in T]
Qn = np.array([sol.value(q) for q in Q]); Vn = np.array([sol.value(v) for v in V])
viol = float(np.max(np.abs(sol.value(o.g) - np.clip(sol.value(o.g), sol.value(o.lbg), sol.value(o.ubg))))) if True else 0
print('feasible' if ok else 'infeasible/at limit', '| max constraint violation %.2e' % viol)
print('phase durations [s]:', np.round(Tn, 3), 'total', round(sum(Tn), 3))
sole_xy = [[np.array(f_sole[s](Qn[k])[0]).ravel() for k in range(N + 1)] for s in (0, 1)]
k_td = n_ph[0]            # right foot touches down
k_td2 = n_ph[0] + n_ph[1] + n_ph[2]   # left foot touches down
if len(n_ph) >= 6:
    k_td3 = sum(n_ph[:5])
    print('right foot lands again at (x, y) = (%.3f, %.3f)' % tuple(np.array(f_sole[1](Qn[k_td3])[0]).ravel()[:2]))
print('right foot lands at (x, y) = (%.3f, %.3f)  [left stance foot at (0, %.3f)]' % (*sole_xy[1][k_td][:2], w))
print('left  foot lands at (x, y) = (%.3f, %.3f)' % tuple(sole_xy[0][k_td2][:2]))
com = np.array([np.array(f_com(Qn[k])).ravel() for k in range(N + 1)])
print('CoM y: start %.3f max %.3f end %.3f | CoM z min %.3f' % (com[0, 1], com[:, 1].max(), com[-1, 1], com[:, 2].min()))
print('peak joint speed / limit: %.2f' % np.max(np.abs(Vn[:, 6:]) / g.vmax))
hip_roll = [k for k, n in enumerate(g.names) if 'hip_roll' in n]
print('hip roll range used [rad]:', np.round([Qn[:, 6 + k].min() for k in hip_roll] + [Qn[:, 6 + k].max() for k in hip_roll], 2),
      'limits', np.round(g.lower[hip_roll], 2), np.round(g.upper[hip_roll], 2))
swing_speed = max(np.linalg.norm(sole_xy[1][k + 1][:2] - sole_xy[1][k][:2]) / (Tn[0] / n_ph[0]) for k in range(n_ph[0]))
print('peak swing foot speed in the first step: %.2f m/s' % swing_speed)
if args.out:
    t = np.concatenate([[0], np.cumsum([Tn[phase_of[k]] / n_ph[phase_of[k]] for k in range(N)])])
    Fn = np.zeros((N, 2, 4, 3))
    for k in range(N):
        for s_ in (0, 1):
            if F[k][s_] is not None:
                Fn[k, s_] = np.array([sol.value(fc) for fc in F[k][s_]])
    json.dump({'ok': ok, 'impulse': args.impulse, 'impulse_deg': args.impulse_deg, 'swing_phase': args.phase, 'delay': args.delay, 'landed': bool(args.landed), 'vx': args.vx, 'T': Tn, 't': t.tolist(), 'q': Qn.tolist(), 'v': Vn.tolist(), 'f': Fn.tolist(),
               'names': g.names, 'com': com.tolist(), 'sole_l': np.array(sole_xy[0]).tolist(), 'sole_r': np.array(sole_xy[1]).tolist()},
              open(args.out, 'w'))
