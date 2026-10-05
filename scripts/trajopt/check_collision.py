"""Leg self-collision post-check of a push_recovery.py solution: minimum
distance between the capsules hip->knee->ankle of the two legs and between
each sole centre and the other shank, interpolated between nodes.

    /data/articara/venv_trajopt/bin/python scripts/trajopt/check_collision.py /data/articara/trajopt/push7.json
"""
import json, sys
import numpy as np
import pinocchio as pin
from g1_casadi_model import G1Model, SOLE_OFFSET

def seg_dist(p1, q1, p2, q2):
    """Minimum distance between segments p1q1 and p2q2 (Ericson)."""
    d1, d2, r = q1 - p1, q2 - p2, p1 - p2
    a, e, f = d1 @ d1, d2 @ d2, d2 @ r
    c, b = d1 @ r, d1 @ d2
    den = a * e - b * b
    if a < 1e-12:      # the first segment is a point
        t = np.clip(f / e, 0, 1)
        return np.linalg.norm(p1 - p2 - d2 * t)
    s = np.clip((b * f - c * e) / den, 0, 1) if den > 1e-12 else 0.0
    t = np.clip((b * s + f) / e, 0, 1)
    s = np.clip((b * t - c) / a, 0, 1)
    return np.linalg.norm(p1 + d1 * s - p2 - d2 * t)

d = json.load(open(sys.argv[1]))
g = G1Model(); m = g.pin; data = m.createData()
Q = np.array(d['q'])
ids = {s: [m.getJointId(f'{s}_{n}_joint') for n in ('hip_pitch', 'knee', 'ankle_roll')] for s in ('left', 'right')}
best = {'thigh-thigh': 9, 'shank-shank': 9, 'thigh-shank': 9, 'sole-shank': 9}
for k in range(len(Q) - 1):
    for a in np.linspace(0, 1, 6):
        q = (1 - a) * Q[k] + a * Q[k + 1]
        qp, _ = g.to_pin(q, np.zeros(g.nq))
        pin.forwardKinematics(m, data, qp)
        P = {s: [data.oMi[i].translation.copy() for i in ids[s]] for s in ids}
        S = {s: data.oMi[ids[s][2]].translation + data.oMi[ids[s][2]].rotation @ SOLE_OFFSET for s in ids}
        L, R = P['left'], P['right']
        best['thigh-thigh'] = min(best['thigh-thigh'], seg_dist(L[0], L[1], R[0], R[1]))
        best['shank-shank'] = min(best['shank-shank'], seg_dist(L[1], L[2], R[1], R[2]))
        best['thigh-shank'] = min(best['thigh-shank'], seg_dist(L[0], L[1], R[1], R[2]), seg_dist(R[0], R[1], L[1], L[2]))
        best['sole-shank'] = min(best['sole-shank'], seg_dist(S['left'], S['left'], R[1], R[2]), seg_dist(S['right'], S['right'], L[1], L[2]))
print('impulse %.0f N s: minimum centre-line distances [m] (a leg segment is ~0.10 m thick):' % d['impulse'])
for k, v in best.items():
    print('  %-12s %.3f %s' % (k, v, '' if v > 0.10 else '<-- closer than one limb thickness'))
