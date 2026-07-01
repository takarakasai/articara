"""Go2 crawl 実機ログの姿勢振動・Y/Z 揺れを詳細評価する。

- gyro (roll/pitch/yaw 角速度) の時系列とスペクトル
- 加速度から重力を除いた lateral/vertical 成分
- gait-cycle (2.5 s) で phase-folding した平均±包絡
- Y / Z 位置ドリフト (vy, vz 積分)

入力: csv/crawl_vx00*_*.csv
出力: csv/analysis/crawl_vibration.png, crawl_phase_fold.png
"""
from __future__ import annotations
import csv
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "analysis"
OUT.mkdir(exist_ok=True)
GAIT_CYCLE_S = 2.5
DT = 0.002  # 500 Hz


def load(path):
    with open(path) as fh:
        r = csv.reader(fh); hdr = next(r); rows = list(r)
    phase = np.array([row[1] for row in rows])
    keep = [i for i, h in enumerate(hdr) if h != "phase"]
    arr = np.array([[row[i] for i in keep] for row in rows], dtype=float)
    idx = {hdr[c]: j for j, c in enumerate(keep)}
    return arr, idx, phase


def quat_rotate(q, v):
    """q = (w,x,y,z) で v を回す (body→world)。 v: (N,3), q: (N,4)。"""
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    # R @ v 形式
    R00 = 1 - 2 * (y * y + z * z); R01 = 2 * (x * y - z * w); R02 = 2 * (x * z + y * w)
    R10 = 2 * (x * y + z * w); R11 = 1 - 2 * (x * x + z * z); R12 = 2 * (y * z - x * w)
    R20 = 2 * (x * z - y * w); R21 = 2 * (y * z + x * w); R22 = 1 - 2 * (x * x + y * y)
    out = np.stack([
        R00 * v[:, 0] + R01 * v[:, 1] + R02 * v[:, 2],
        R10 * v[:, 0] + R11 * v[:, 1] + R12 * v[:, 2],
        R20 * v[:, 0] + R21 * v[:, 1] + R22 * v[:, 2],
    ], axis=1)
    return out


def gravity_removed_accel(a, idx):
    """IMU の生 acc (body frame) から重力を引いた純粋運動加速度 (body frame)。"""
    acc_b = np.stack([a[:, idx["acc_x"]], a[:, idx["acc_y"]], a[:, idx["acc_z"]]], axis=1)
    quat = np.stack([a[:, idx["quat_w"]], a[:, idx["quat_x"]],
                     a[:, idx["quat_y"]], a[:, idx["quat_z"]]], axis=1)
    # quat を正規化
    quat = quat / np.linalg.norm(quat, axis=1, keepdims=True)
    # gravity in world = (0,0,-9.81). IMU measures specific force = a - g (body frame).
    # body frame gravity = R^T @ g_world.  ただし IMU 出力規約は機種依存。
    # ここでは: acc_body 静止時 ≈ (0,0,+9.81) (上向き重力反力) という前提。
    # → motion = acc_body - R_b←w @ (0,0,+9.81)
    # R_w←b is built from quat (body→world). Need R^T (world→body):
    # R^T @ (0,0,9.81) = 行列の z 列(世界) ↔ row3 of R を使う。 簡単化:
    # body-frame gravity vector = R^T @ [0,0,9.81]. quat_rotate(q, [0,0,9.81]) は body→world なので
    # 逆: quat_rotate の conjugate を使うか、 直接行列を作る。
    # 単純実装: a_motion_body = acc_body - R_b←w @ [0,0,9.81]
    w = quat[:, 0]; x = quat[:, 1]; y = quat[:, 2]; z = quat[:, 3]
    # R_world←body 列のうち z 列 (= world z軸を body 表現)
    gz = 9.81
    g_body_x = 2 * (x * z - y * w) * gz
    g_body_y = 2 * (y * z + x * w) * gz
    g_body_z = (1 - 2 * (x * x + y * y)) * gz
    motion = np.stack([acc_b[:, 0] - g_body_x,
                       acc_b[:, 1] - g_body_y,
                       acc_b[:, 2] - g_body_z], axis=1)
    return acc_b, motion


def welch_spectrum(x, fs, nperseg=2048):
    """シンプルな Welch (window=Hann, overlap 50%)。"""
    n = len(x)
    if n < nperseg:
        nperseg = 2 ** int(np.floor(np.log2(n)))
    step = nperseg // 2
    win = np.hanning(nperseg)
    nf = nperseg // 2 + 1
    psd = np.zeros(nf)
    cnt = 0
    for i in range(0, n - nperseg + 1, step):
        seg = x[i:i + nperseg] - np.mean(x[i:i + nperseg])
        seg = seg * win
        F = np.fft.rfft(seg)
        psd += (np.abs(F) ** 2)
        cnt += 1
    if cnt == 0:
        return np.array([]), np.array([])
    psd /= cnt
    psd /= (fs * (win ** 2).sum())
    psd[1:-1] *= 2
    freqs = np.fft.rfftfreq(nperseg, d=1.0 / fs)
    return freqs, psd


def phase_fold(t, x, t0, cycle_s):
    """t0 以降の x を cycle_s で折り返し、 phase ∈ [0,1) でビン化して mean/±std を返す。"""
    mask = t >= t0
    if not mask.any():
        return None
    tt = t[mask] - t0
    xx = x[mask]
    ph = (tt % cycle_s) / cycle_s
    n_bin = 100
    bins = np.linspace(0, 1, n_bin + 1)
    cb = 0.5 * (bins[:-1] + bins[1:])
    mean = np.full(n_bin, np.nan); std = np.full(n_bin, np.nan)
    for i in range(n_bin):
        m = (ph >= bins[i]) & (ph < bins[i + 1])
        if m.any():
            mean[i] = np.nanmean(xx[m])
            std[i] = np.nanstd(xx[m])
    return cb, mean, std


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
    return np.convolve(xi_pad, w, mode="valid")[:len(x)]


def highpass_1pole(x, fc_hz, dt):
    """1-pole IIR high-pass。 y[n] = α(y[n-1] + x[n] - x[n-1])"""
    rc = 1.0 / (2 * np.pi * fc_hz)
    alpha = rc / (rc + dt)
    y = np.zeros_like(x); y[0] = 0.0
    for i in range(1, len(x)):
        y[i] = alpha * (y[i - 1] + x[i] - x[i - 1])
    return y


def integrate_hp(accel, dt, fc_hz=0.3):
    """積分前後で high-pass を当てて DC/低周波ドリフトを除去した v, p を返す。"""
    a_hp = highpass_1pole(accel - np.nanmean(accel), fc_hz, dt)
    v = np.cumsum(a_hp) * dt
    v = highpass_1pole(v, fc_hz, dt)
    p = np.cumsum(v) * dt
    p = highpass_1pole(p, fc_hz, dt)
    return v, p


def main():
    files = [
        (ROOT / "crawl_vx002_c25_sup08_swing_04_kp100_kd6_smooth_ff.csv", 0.02),
        (ROOT / "crawl_vx005_c25_sup08_swing_04_kp100_kd6_smooth_ff.csv", 0.05),
    ]
    results = []
    for path, vx_target in files:
        a, idx, phase = load(path)
        t = a[:, idx["t_s"]]
        gx = a[:, idx["gyro_x"]]; gy = a[:, idx["gyro_y"]]; gz = a[:, idx["gyro_z"]]
        roll = a[:, idx["roll"]]; pitch = a[:, idx["pitch"]]; yaw = a[:, idx["yaw"]]
        acc_b, acc_m = gravity_removed_accel(a, idx)
        cmask = (phase == 'C')
        tc = t[cmask]
        t0 = tc[0] + GAIT_CYCLE_S if len(tc) > 0 else 0.0
        steady = cmask & (t >= t0)

        # 加速度から motion 成分のみ抽出 → high-pass 付き 2 重積分で oscillation 推定
        # (drift 抑制のため fc=0.3 Hz で HP、 gait 周期 ~0.4 Hz より低い)
        ay_m = acc_m[:, 1]; az_m = acc_m[:, 2]
        vy_int, py_int = integrate_hp(ay_m, DT, fc_hz=0.3)
        vz_int, pz_int = integrate_hp(az_m, DT, fc_hz=0.3)

        results.append(dict(label=path.stem, vx_target=vx_target,
                            t=t, phase=phase, steady=steady, t0=t0,
                            roll=roll, pitch=pitch, yaw=yaw,
                            gx=gx, gy=gy, gz=gz,
                            acc_b=acc_b, acc_m=acc_m,
                            vy_int=vy_int, vz_int=vz_int,
                            py_int=py_int, pz_int=pz_int))

    colors = {0.02: "#1f77b4", 0.05: "#d62728"}
    gait_f = 1.0 / GAIT_CYCLE_S  # ≈ 0.4 Hz
    leg_step_f = 4.0 / GAIT_CYCLE_S  # 各 cycle で 4 足 step → ≈ 1.6 Hz

    # ===== Figure 1: vibration (time series + spectrum) =====
    fig, axes = plt.subplots(4, 2, figsize=(14, 11))

    # (1,左) gyro_x (roll rate) 時系列
    ax = axes[0, 0]
    for r in results:
        c = colors[r["vx_target"]]
        ax.plot(r["t"][r["steady"]], np.degrees(r["gx"][r["steady"]]),
                color=c, lw=0.6, alpha=0.7,
                label=f'gx (target={r["vx_target"]:.2f})  '
                      f'RMS={np.degrees(np.std(r["gx"][r["steady"]])):.1f} dps')
    ax.set_ylabel("roll rate [deg/s]")
    ax.set_title("Roll rate (gyro_x) — steady-state crawl")
    ax.legend(fontsize=8); ax.grid(alpha=0.3)

    # (1,右) gyro_x スペクトル
    ax = axes[0, 1]
    for r in results:
        c = colors[r["vx_target"]]
        f, p = welch_spectrum(r["gx"][r["steady"]], fs=1 / DT)
        ax.semilogy(f, np.degrees(np.degrees(p)) ** 0.5, color=c, lw=1.0,
                    label=f'target={r["vx_target"]:.2f}')
    ax.axvline(gait_f, color="k", lw=0.6, ls=":", label=f"gait f = {gait_f:.2f} Hz")
    ax.axvline(leg_step_f, color="k", lw=0.6, ls="--", label=f"4×gait = {leg_step_f:.2f} Hz")
    ax.set_xlim(0, 25); ax.set_xlabel("freq [Hz]"); ax.set_ylabel("amp [√PSD]")
    ax.set_title("Roll-rate spectrum (Welch)")
    ax.legend(fontsize=8); ax.grid(alpha=0.3, which="both")

    # (2,左) gyro_y (pitch rate)
    ax = axes[1, 0]
    for r in results:
        c = colors[r["vx_target"]]
        ax.plot(r["t"][r["steady"]], np.degrees(r["gy"][r["steady"]]),
                color=c, lw=0.6, alpha=0.7,
                label=f'gy  RMS={np.degrees(np.std(r["gy"][r["steady"]])):.1f} dps')
    ax.set_ylabel("pitch rate [deg/s]")
    ax.set_title("Pitch rate (gyro_y)")
    ax.legend(fontsize=8); ax.grid(alpha=0.3)

    # (2,右) gyro_y spectrum
    ax = axes[1, 1]
    for r in results:
        c = colors[r["vx_target"]]
        f, p = welch_spectrum(r["gy"][r["steady"]], fs=1 / DT)
        ax.semilogy(f, p ** 0.5, color=c, lw=1.0,
                    label=f'target={r["vx_target"]:.2f}')
    ax.axvline(gait_f, color="k", lw=0.6, ls=":")
    ax.axvline(leg_step_f, color="k", lw=0.6, ls="--")
    ax.set_xlim(0, 25); ax.set_xlabel("freq [Hz]"); ax.set_ylabel("amp [√PSD]")
    ax.set_title("Pitch-rate spectrum")
    ax.legend(fontsize=8); ax.grid(alpha=0.3, which="both")

    # (3,左) lateral accel (motion = grav 除去後 body-y)
    ax = axes[2, 0]
    for r in results:
        c = colors[r["vx_target"]]
        ax.plot(r["t"][r["steady"]], r["acc_m"][r["steady"], 1],
                color=c, lw=0.6, alpha=0.7,
                label=f'a_y  RMS={np.std(r["acc_m"][r["steady"],1]):.2f} m/s²')
    ax.set_ylabel("lateral accel [m/s²]")
    ax.set_title("Lateral accel a_y  (gravity removed)")
    ax.legend(fontsize=8); ax.grid(alpha=0.3)

    # (3,右) a_y spectrum
    ax = axes[2, 1]
    for r in results:
        c = colors[r["vx_target"]]
        f, p = welch_spectrum(r["acc_m"][r["steady"], 1], fs=1 / DT)
        ax.semilogy(f, p ** 0.5, color=c, lw=1.0,
                    label=f'target={r["vx_target"]:.2f}')
    ax.axvline(gait_f, color="k", lw=0.6, ls=":")
    ax.axvline(leg_step_f, color="k", lw=0.6, ls="--")
    ax.set_xlim(0, 25); ax.set_xlabel("freq [Hz]"); ax.set_ylabel("amp [√PSD]")
    ax.set_title("Lateral accel spectrum")
    ax.legend(fontsize=8); ax.grid(alpha=0.3, which="both")

    # (4,左) vertical accel
    ax = axes[3, 0]
    for r in results:
        c = colors[r["vx_target"]]
        ax.plot(r["t"][r["steady"]], r["acc_m"][r["steady"], 2],
                color=c, lw=0.6, alpha=0.7,
                label=f'a_z  RMS={np.std(r["acc_m"][r["steady"],2]):.2f} m/s²')
    ax.set_xlabel("time [s]")
    ax.set_ylabel("vertical accel [m/s²]")
    ax.set_title("Vertical accel a_z  (gravity removed)")
    ax.legend(fontsize=8); ax.grid(alpha=0.3)

    # (4,右) a_z spectrum
    ax = axes[3, 1]
    for r in results:
        c = colors[r["vx_target"]]
        f, p = welch_spectrum(r["acc_m"][r["steady"], 2], fs=1 / DT)
        ax.semilogy(f, p ** 0.5, color=c, lw=1.0,
                    label=f'target={r["vx_target"]:.2f}')
    ax.axvline(gait_f, color="k", lw=0.6, ls=":")
    ax.axvline(leg_step_f, color="k", lw=0.6, ls="--")
    ax.set_xlim(0, 25); ax.set_xlabel("freq [Hz]"); ax.set_ylabel("amp [√PSD]")
    ax.set_title("Vertical accel spectrum")
    ax.legend(fontsize=8); ax.grid(alpha=0.3, which="both")

    fig.suptitle("Attitude vibration & Y/Z acceleration  (crawl steady-state, gravity removed)",
                 fontsize=13)
    fig.tight_layout()
    fig.savefig(OUT / "crawl_vibration.png", dpi=140)
    plt.close(fig)

    # ===== Figure 2: gait-cycle phase fold =====
    fig, axes = plt.subplots(3, 2, figsize=(13, 9), sharex=True)
    pairs = [
        ("roll", "deg", lambda r: np.degrees(r["roll"])),
        ("pitch", "deg", lambda r: np.degrees(r["pitch"])),
        ("a_y (motion)", "m/s²", lambda r: r["acc_m"][:, 1]),
        ("a_z (motion)", "m/s²", lambda r: r["acc_m"][:, 2]),
        ("p_y integrated", "m", lambda r: r["py_int"]),
        ("p_z integrated", "m", lambda r: r["pz_int"]),
    ]
    for ax, (name, unit, fn) in zip(axes.flat, pairs):
        for r in results:
            c = colors[r["vx_target"]]
            res = phase_fold(r["t"], fn(r), r["t0"], GAIT_CYCLE_S)
            if res is None:
                continue
            cb, mean, std = res
            ax.fill_between(cb, mean - std, mean + std, color=c, alpha=0.2)
            ax.plot(cb, mean, color=c, lw=1.4,
                    label=f'target={r["vx_target"]:.2f}  pp={np.nanmax(mean)-np.nanmin(mean):.3f} {unit}')
        ax.set_ylabel(f"{name} [{unit}]")
        ax.set_title(f"phase-folded {name}")
        ax.set_xlim(0, 1); ax.grid(alpha=0.3); ax.legend(fontsize=8)
    axes[-1, 0].set_xlabel("phase  (0 = cycle start)")
    axes[-1, 1].set_xlabel("phase")
    fig.suptitle(f"Gait-cycle phase-folded waveforms  (cycle = {GAIT_CYCLE_S:.1f} s, "
                 f"mean ± 1σ across cycles)", fontsize=12)
    fig.tight_layout()
    fig.savefig(OUT / "crawl_phase_fold.png", dpi=140)
    plt.close(fig)

    # ===== Figure 3: Y/Z 揺れ 時系列 (integrated) =====
    fig, axes = plt.subplots(2, 1, figsize=(12, 6), sharex=True)
    ax = axes[0]
    for r in results:
        c = colors[r["vx_target"]]
        m = r["steady"]
        ax.plot(r["t"][m], r["py_int"][m] * 1000, color=c, lw=1.0,
                label=f'p_y (target={r["vx_target"]:.2f})  '
                      f'pp={np.ptp(r["py_int"][m])*1000:.1f} mm  '
                      f'RMS={np.std(r["py_int"][m])*1000:.1f} mm')
        ax.plot(r["t"][m], r["pz_int"][m] * 1000, color=c, lw=1.0, ls="--",
                label=f'p_z (target={r["vx_target"]:.2f})  '
                      f'pp={np.ptp(r["pz_int"][m])*1000:.1f} mm  '
                      f'RMS={np.std(r["pz_int"][m])*1000:.1f} mm')
    ax.set_ylabel("position deviation [mm]")
    ax.set_title("Y / Z position oscillation (acc 2x integration, detrended)")
    ax.legend(fontsize=8, ncol=2); ax.grid(alpha=0.3)

    ax = axes[1]
    for r in results:
        c = colors[r["vx_target"]]
        m = r["steady"]
        ax.plot(r["t"][m], r["vy_int"][m], color=c, lw=1.0,
                label=f'v_y (target={r["vx_target"]:.2f})  '
                      f'RMS={np.std(r["vy_int"][m]):.3f} m/s')
        ax.plot(r["t"][m], r["vz_int"][m], color=c, lw=1.0, ls="--",
                label=f'v_z (target={r["vx_target"]:.2f})  '
                      f'RMS={np.std(r["vz_int"][m]):.3f} m/s')
    ax.set_xlabel("time [s]"); ax.set_ylabel("velocity [m/s]")
    ax.set_title("Y / Z velocity oscillation (acc integration, detrended)")
    ax.legend(fontsize=8, ncol=2); ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(OUT / "crawl_yz_oscillation.png", dpi=140)
    plt.close(fig)

    # サマリ更新
    lines = ["", "=== Vibration / Y-Z oscillation summary (steady-state crawl) ==="]
    for r in results:
        m = r["steady"]
        lines.append(f"\n[{r['label']}]  target vx = {r['vx_target']:.3f} m/s")
        lines.append(f"  gyro_x (roll rate) : RMS = {np.degrees(np.std(r['gx'][m])):.2f} deg/s, "
                     f"peak = {np.degrees(np.max(np.abs(r['gx'][m]))):.2f} deg/s")
        lines.append(f"  gyro_y (pitch rate): RMS = {np.degrees(np.std(r['gy'][m])):.2f} deg/s, "
                     f"peak = {np.degrees(np.max(np.abs(r['gy'][m]))):.2f} deg/s")
        lines.append(f"  gyro_z (yaw rate)  : RMS = {np.degrees(np.std(r['gz'][m])):.2f} deg/s")
        lines.append(f"  a_y (lateral) : RMS = {np.std(r['acc_m'][m,1]):.3f} m/s², "
                     f"peak = {np.max(np.abs(r['acc_m'][m,1])):.3f}")
        lines.append(f"  a_z (vertical): RMS = {np.std(r['acc_m'][m,2]):.3f} m/s², "
                     f"peak = {np.max(np.abs(r['acc_m'][m,2])):.3f}")
        lines.append(f"  p_y oscillation: RMS = {np.std(r['py_int'][m])*1000:.2f} mm, "
                     f"p-p = {np.ptp(r['py_int'][m])*1000:.2f} mm")
        lines.append(f"  p_z oscillation: RMS = {np.std(r['pz_int'][m])*1000:.2f} mm, "
                     f"p-p = {np.ptp(r['pz_int'][m])*1000:.2f} mm")
    txt = "\n".join(lines)
    with open(OUT / "summary.txt", "a") as fh:
        fh.write(txt + "\n")
    print(txt)
    print(f"\nadded: {OUT}/crawl_vibration.png, crawl_phase_fold.png, crawl_yz_oscillation.png")


if __name__ == "__main__":
    main()
