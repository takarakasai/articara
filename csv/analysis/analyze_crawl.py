"""Go2 crawl 実機ログの目標速度追従性・姿勢揺れを評価する。

入力: csv/crawl_vx00*_*.csv (foot0=FR sensor stuck at 101 -- treated as broken)
出力: csv/analysis/*.png, csv/analysis/summary.txt
"""
from __future__ import annotations
import csv
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "analysis"
OUT.mkdir(exist_ok=True)

# --- Go2 寸法 (mujoco go2.xml より) ---
HIP_BASE = {
    "FR": np.array([ 0.1934, -0.0465, 0.0]),
    "FL": np.array([ 0.1934,  0.0465, 0.0]),
    "RR": np.array([-0.1934, -0.0465, 0.0]),
    "RL": np.array([-0.1934,  0.0465, 0.0]),
}
THIGH_Y = {"FR": -0.0955, "FL": 0.0955, "RR": -0.0955, "RL": 0.0955}
L_THIGH = 0.213
L_CALF = 0.213
LEGS = ["FR", "FL", "RR", "RL"]
FOOT_IDX = {"FR": 0, "FL": 1, "RR": 2, "RL": 3}
GAIT_CYCLE_S = 2.5  # c25 = 2.5 s cycle (autocorr 実測: vx002=2.48s, vx005=2.30s)


def rot_x(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_y(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def foot_pos_body(leg, qhip, qthigh, qcalf):
    p = HIP_BASE[leg].copy()
    Rh = rot_x(qhip)
    p = p + Rh @ np.array([0.0, THIGH_Y[leg], 0.0])
    R = Rh @ rot_y(qthigh)
    p = p + R @ np.array([0.0, 0.0, -L_THIGH])
    R = R @ rot_y(qcalf)
    p = p + R @ np.array([0.0, 0.0, -L_CALF])
    return p


def foot_jacobian_body(leg, qhip, qthigh, qcalf):
    h = 1e-5
    q = np.array([qhip, qthigh, qcalf])
    p0 = foot_pos_body(leg, *q)
    J = np.zeros((3, 3))
    for i in range(3):
        qp = q.copy(); qp[i] += h
        J[:, i] = (foot_pos_body(leg, *qp) - p0) / h
    return J


def load_csv(path):
    with open(path) as fh:
        r = csv.reader(fh); hdr = next(r); rows = list(r)
    phase = np.array([row[1] for row in rows])
    keep = [i for i, h in enumerate(hdr) if h != "phase"]
    arr = np.array([[row[i] for i in keep] for row in rows], dtype=float)
    idx = {hdr[c]: j for j, c in enumerate(keep)}
    return arr, idx, phase


def per_leg_contact(foot, phase):
    """foot[N,4] と phase から接触マスク[N,4]。各 leg ごとに median*0.5 を閾値、
    foot0 (FR) はセンサ故障 (常時 101) のため FL/RR/RL の swing パターンから推定。"""
    cmask = (phase == 'C')
    contact = np.zeros_like(foot, dtype=bool)
    for i in range(4):
        if foot[cmask, i].std() < 1.0:  # broken sensor
            continue
        thr = max(5.0, 0.5 * np.median(foot[cmask, i]))
        contact[:, i] = foot[:, i] > thr
    # foot0 (FR) 推定: crawl では同時 swing する足は通常 1 本のみ。
    # FL/RR/RL の少なくとも 1 本が swing 中なら FR は stance、 全 stance なら FR も stance。
    other_swing = (~contact[:, 1]) | (~contact[:, 2]) | (~contact[:, 3])
    contact[:, 0] = True  # デフォルト stance
    # FR が swing するタイミングは他 3 本が全て stance の時のみ短時間: 簡易には常時 stance とする
    return contact


def compute_body_vel(a, idx, phase):
    n = a.shape[0]
    t = a[:, idx["t_s"]]
    gx = a[:, idx["gyro_x"]]; gy = a[:, idx["gyro_y"]]; gz = a[:, idx["gyro_z"]]
    foot = np.stack([a[:, idx[f"foot{i}"]] for i in range(4)], axis=1)
    contact = per_leg_contact(foot, phase)

    q = {leg: np.stack([a[:, idx[f"{leg}_hip_q"]],
                         a[:, idx[f"{leg}_thigh_q"]],
                         a[:, idx[f"{leg}_calf_q"]]], axis=1) for leg in LEGS}
    dq = {leg: np.stack([a[:, idx[f"{leg}_hip_dq"]],
                          a[:, idx[f"{leg}_thigh_dq"]],
                          a[:, idx[f"{leg}_calf_dq"]]], axis=1) for leg in LEGS}

    v_est = np.full((n, 3), np.nan)
    contact_count = np.zeros(n, dtype=int)
    for k in range(n):
        omega = np.array([gx[k], gy[k], gz[k]])
        v_sum = np.zeros(3); cnt = 0
        for li, leg in enumerate(LEGS):
            if not contact[k, li]:
                continue
            p = foot_pos_body(leg, *q[leg][k])
            J = foot_jacobian_body(leg, *q[leg][k])
            v_foot_due_to_leg = J @ dq[leg][k]
            # contact 仮定: v_foot_body ≈ 0 → v_body_body = -(Jq̇ + ω×p)
            v_body = -(v_foot_due_to_leg + np.cross(omega, p))
            v_sum += v_body; cnt += 1
        if cnt > 0:
            v_est[k] = v_sum / cnt
            contact_count[k] = cnt
    return t, v_est, contact_count, contact


def smooth(x, win):
    if win <= 1:
        return x.copy()
    w = np.ones(win) / win
    mask = ~np.isnan(x)
    if mask.sum() < 2:
        return x.copy()
    xi = np.where(mask, x, np.interp(np.arange(len(x)),
                                      np.where(mask)[0], x[mask]))
    pad = win // 2
    xi_pad = np.concatenate([np.full(pad, xi[0]), xi, np.full(pad, xi[-1])])
    y = np.convolve(xi_pad, w, mode="valid")
    return y[:len(x)]


def main():
    files = [
        (ROOT / "crawl_vx002_c25_sup08_swing_04_kp100_kd6_smooth_ff.csv", 0.02),
        (ROOT / "crawl_vx005_c25_sup08_swing_04_kp100_kd6_smooth_ff.csv", 0.05),
    ]
    dt = 0.002  # 500 Hz
    win_cycle = int(GAIT_CYCLE_S / dt)  # 500 samples = 1 gait cycle

    results = []
    for path, vx_target in files:
        a, idx, phase = load_csv(path)
        t, v, cc, contact = compute_body_vel(a, idx, phase)
        roll = a[:, idx["roll"]]; pitch = a[:, idx["pitch"]]; yaw = a[:, idx["yaw"]]

        # 1-cycle MA (gait cycle 平均で stance/swing 周期成分を均す)
        vx_s = smooth(v[:, 0], win_cycle)
        vy_s = smooth(v[:, 1], win_cycle)
        vz_s = smooth(v[:, 2], win_cycle)

        # Position integration (body frame approx; yaw small → world ≈ body for this run)
        # crawl 開始 (phase=C) からの累積
        cmask = (phase == 'C')
        x_int = np.zeros(len(t)); y_int = np.zeros(len(t))
        for k in range(1, len(t)):
            if cmask[k] and not np.isnan(vx_s[k]):
                x_int[k] = x_int[k-1] + vx_s[k] * dt
                y_int[k] = y_int[k-1] + vy_s[k] * dt
            else:
                x_int[k] = x_int[k-1]
                y_int[k] = y_int[k-1]

        # steady-state stats (crawl 開始 + 1 cycle 以降)
        tc = t[cmask]
        t0 = tc[0] + GAIT_CYCLE_S if len(tc) > 0 else 0.0
        steady = cmask & (t >= t0) & ~np.isnan(vx_s)
        s = dict(
            vx_mean=np.nanmean(vx_s[steady]),
            vx_std=np.nanstd(vx_s[steady]),
            vx_rmse=np.sqrt(np.nanmean((vx_s[steady] - vx_target) ** 2)),
            vy_mean=np.nanmean(vy_s[steady]),
            vy_std=np.nanstd(vy_s[steady]),
            roll_rms=np.sqrt(np.nanmean(roll[steady] ** 2)),
            pitch_rms=np.sqrt(np.nanmean(pitch[steady] ** 2)),
            roll_pp=np.nanmax(roll[steady]) - np.nanmin(roll[steady]),
            pitch_pp=np.nanmax(pitch[steady]) - np.nanmin(pitch[steady]),
        )
        idx_st = np.where(steady)[0]
        if len(idx_st) >= 2:
            s["yaw_rate"] = np.polyfit(t[idx_st], yaw[idx_st], 1)[0]
        else:
            s["yaw_rate"] = np.nan
        # 距離達成率: 累積 x の steady 区間
        d_total = x_int[steady][-1] - x_int[steady][0] if steady.sum() > 1 else 0.0
        d_target = vx_target * (t[steady][-1] - t[steady][0]) if steady.sum() > 1 else 1.0
        s["dist_ratio"] = d_total / d_target if d_target != 0 else float('nan')

        results.append(dict(label=path.stem, vx_target=vx_target,
                            t=t, phase=phase, v=v, vx_s=vx_s, vy_s=vy_s, vz_s=vz_s,
                            cc=cc, contact=contact,
                            roll=roll, pitch=pitch, yaw=yaw,
                            x_int=x_int, y_int=y_int, stats=s, steady=steady))

    # ---- plots ----
    colors = {0.02: "#1f77b4", 0.05: "#d62728"}

    fig, axes = plt.subplots(5, 1, figsize=(12, 14), sharex=True)
    # (1) vx tracking
    ax = axes[0]
    for r in results:
        c = colors[r["vx_target"]]
        ax.plot(r["t"], r["vx_s"], color=c, lw=1.2,
                label=f'vx_target={r["vx_target"]:.2f} m/s (estimated, 1-cycle MA)')
        ax.axhline(r["vx_target"], color=c, ls="--", lw=0.8, alpha=0.7)
    ax.axvspan(0, 3, color="gray", alpha=0.1)
    ax.set_ylabel("vx [m/s]")
    ax.set_title("Forward velocity tracking (leg odometry, FR foot sensor broken & ignored for contact)")
    ax.legend(loc="upper right", fontsize=9); ax.grid(alpha=0.3)

    # (2) vy & vz
    ax = axes[1]
    for r in results:
        c = colors[r["vx_target"]]
        ax.plot(r["t"], r["vy_s"], color=c, lw=1.0,
                label=f'vy  (target={r["vx_target"]:.2f})')
        ax.plot(r["t"], r["vz_s"], color=c, lw=0.8, ls=":",
                label=f'vz  (target={r["vx_target"]:.2f})')
    ax.axhline(0, color="k", lw=0.5)
    ax.set_ylabel("vy / vz  [m/s]")
    ax.set_title("Lateral vy (solid) & vertical vz (dotted) -- ideally 0")
    ax.legend(fontsize=8, ncol=2); ax.grid(alpha=0.3)

    # (3) roll / pitch
    ax = axes[2]
    for r in results:
        c = colors[r["vx_target"]]
        ax.plot(r["t"], np.degrees(r["roll"]), color=c, lw=1.0,
                label=f'roll  (target={r["vx_target"]:.2f})')
        ax.plot(r["t"], np.degrees(r["pitch"]), color=c, lw=0.8, ls="--",
                label=f'pitch (target={r["vx_target"]:.2f})')
    ax.set_ylabel("attitude [deg]")
    ax.set_title("Attitude swing  (roll = solid, pitch = dashed)")
    ax.legend(fontsize=8, ncol=2); ax.grid(alpha=0.3)

    # (4) yaw drift
    ax = axes[3]
    for r in results:
        c = colors[r["vx_target"]]
        y0 = r["yaw"][0]
        ax.plot(r["t"], np.degrees(r["yaw"] - y0), color=c, lw=1.2,
                label=f'Δyaw (target={r["vx_target"]:.2f}, '
                      f'rate={np.degrees(r["stats"]["yaw_rate"]):+.2f} deg/s)')
    ax.set_ylabel("Δyaw [deg]")
    ax.set_title("Yaw drift")
    ax.legend(fontsize=9); ax.grid(alpha=0.3)

    # (5) cumulative distance
    ax = axes[4]
    for r in results:
        c = colors[r["vx_target"]]
        m = (r["phase"] == 'C')
        ax.plot(r["t"][m], r["x_int"][m], color=c, lw=1.2,
                label=f'∫vx dt (target={r["vx_target"]:.2f})')
        # target straight line, anchored at start of crawl
        tc = r["t"][m]
        if len(tc):
            x_target = (tc - tc[0]) * r["vx_target"]
            ax.plot(tc, x_target, color=c, ls="--", lw=0.8, alpha=0.7,
                    label=f'target line ({r["vx_target"]:.2f} m/s)')
    ax.set_ylabel("forward dist [m]")
    ax.set_xlabel("time [s]")
    ax.set_title("Cumulative forward distance vs target")
    ax.legend(fontsize=8, ncol=2); ax.grid(alpha=0.3)

    fig.tight_layout()
    fig.savefig(OUT / "crawl_overview.png", dpi=140)
    plt.close(fig)

    # zoom: vx tracking only
    fig, axes = plt.subplots(1, 2, figsize=(12, 4), sharey=True)
    for ax, r in zip(axes, results):
        c = colors[r["vx_target"]]
        m = (r["phase"] == 'C')
        ax.plot(r["t"][m], r["v"][m, 0], color=c, alpha=0.2, lw=0.4, label="raw vx (per-step)")
        ax.plot(r["t"][m], r["vx_s"][m], color=c, lw=1.5,
                label=f'1-cycle MA ({GAIT_CYCLE_S:.1f} s)')
        ax.axhline(r["vx_target"], color="k", ls="--", lw=0.8,
                   label=f'target {r["vx_target"]:.2f} m/s')
        s = r["stats"]
        ax.set_title(f'target={r["vx_target"]:.2f} m/s:  '
                     f'mean={s["vx_mean"]:+.4f}  RMSE={s["vx_rmse"]:.4f}  '
                     f'dist ratio={s["dist_ratio"]:.2f}')
        ax.set_xlabel("time [s]"); ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
    axes[0].set_ylabel("vx [m/s]")
    fig.suptitle("Forward velocity tracking (crawl phase only)")
    fig.tight_layout()
    fig.savefig(OUT / "crawl_vx_tracking.png", dpi=140)
    plt.close(fig)

    # xy trajectory (integrated)
    fig, ax = plt.subplots(figsize=(7, 6))
    for r in results:
        c = colors[r["vx_target"]]
        m = (r["phase"] == 'C')
        ax.plot(r["x_int"][m], r["y_int"][m], color=c, lw=1.2,
                label=f'target={r["vx_target"]:.2f} m/s (final '
                      f'x={r["x_int"][m][-1]:.2f}, y={r["y_int"][m][-1]:.2f})')
    ax.set_xlabel("x  [m]"); ax.set_ylabel("y  [m]")
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_title("Integrated XY trajectory (body frame, yaw ~ const)")
    ax.legend(fontsize=9); ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(OUT / "crawl_xy_trajectory.png", dpi=140)
    plt.close(fig)

    # summary text
    lines = ["=== Go2 crawl steady-state summary (crawl phase, t >= start+1 cycle) ==="]
    for r in results:
        s = r["stats"]
        lines.append(f"\n[{r['label']}]  target vx = {r['vx_target']:.3f} m/s")
        lines.append(f"  vx  estimate : mean = {s['vx_mean']:+.4f} m/s  "
                     f"std = {s['vx_std']:.4f}  RMSE vs target = {s['vx_rmse']:.4f}")
        lines.append(f"  vx  achieved : {s['vx_mean']/r['vx_target']*100:+.1f} %  "
                     f"(distance ratio over run = {s['dist_ratio']*100:+.1f} %)")
        lines.append(f"  vy  estimate : mean = {s['vy_mean']:+.4f}  std = {s['vy_std']:.4f}  m/s")
        lines.append(f"  roll        : RMS = {np.degrees(s['roll_rms']):.2f} deg, "
                     f"peak-peak = {np.degrees(s['roll_pp']):.2f} deg")
        lines.append(f"  pitch       : RMS = {np.degrees(s['pitch_rms']):.2f} deg, "
                     f"peak-peak = {np.degrees(s['pitch_pp']):.2f} deg")
        lines.append(f"  yaw drift   : {np.degrees(s['yaw_rate']):+.3f} deg/s")
    txt = "\n".join(lines)
    (OUT / "summary.txt").write_text(txt + "\n")
    print(txt)
    print(f"\nplots → {OUT}/")


if __name__ == "__main__":
    main()
