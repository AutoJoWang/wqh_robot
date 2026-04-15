"""复杂连续轨迹综合抗扰：S 弯 + 随机水坑 (ηv/ηω 骤降 + 侧向推力)，无补偿 vs 残差补偿。"""

from __future__ import annotations

import argparse
import os
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np


def _wrap_pi(angle: float) -> float:
    return float((angle + np.pi) % (2.0 * np.pi) - np.pi)


@dataclass(frozen=True)
class Puddle:
    """沿参考弧长 [s0, s1] 的水坑参数。"""

    s0: float
    s1: float
    eta_v: float
    eta_omega: float
    lateral_mag: float
    lateral_sign: float  # ±1，沿路径左法向


def build_s_lane_path(
    *,
    x_straight0: float,
    x_curve_end: float,
    x_straight1: float,
    amp: float,
    wave_len: float,
    ds_sample: float,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    直线段 + 正弦 S 弯 + 直线出口；返回 (px, py, s_cum, psi)。
    """
    pts: List[Tuple[float, float]] = []
    for x in np.arange(0.0, x_straight0, ds_sample):
        pts.append((float(x), 0.0))
    pts.append((float(x_straight0), 0.0))

    xc = np.arange(x_straight0, x_curve_end + ds_sample * 0.5, ds_sample)
    for x in xc:
        y = float(amp) * np.sin(2.0 * np.pi * (float(x) - float(x_straight0)) / float(wave_len))
        pts.append((float(x), y))

    y_flat = float(amp) * np.sin(
        2.0 * np.pi * (float(x_curve_end) - float(x_straight0)) / float(wave_len)
    )
    for x in np.arange(x_curve_end + ds_sample, x_straight1 + ds_sample * 0.5, ds_sample):
        pts.append((float(x), y_flat))

    px = np.array([p[0] for p in pts], dtype=float)
    py = np.array([p[1] for p in pts], dtype=float)
    dx = np.diff(px)
    dy = np.diff(py)
    seg = np.hypot(dx, dy)
    s_cum = np.concatenate([[0.0], np.cumsum(seg)])

    psi = np.zeros_like(px)
    for i in range(len(px) - 1):
        psi[i] = float(np.arctan2(py[i + 1] - py[i], px[i + 1] - px[i]))
    psi[-1] = psi[-2]

    return px, py, s_cum, psi


def _fix_closest_psi(
    x: float,
    y: float,
    px: np.ndarray,
    py: np.ndarray,
    s_cum: np.ndarray,
    *,
    i_lo: int = 0,
    i_hi: int | None = None,
) -> Tuple[float, float, float, float, float]:
    """在折线段索引 [i_lo, i_hi) 上找最近投影点及 Frenet 横向误差；i_hi=None 表示搜到末尾。"""
    nseg = len(px) - 1
    lo = max(0, int(i_lo))
    hi = nseg if i_hi is None else min(int(i_hi), nseg)
    best_d2 = 1e30
    s_ref, xr, yr, ps_r, e_y = 0.0, float(px[0]), float(py[0]), 0.0, 0.0
    for i in range(lo, hi):
        p0 = np.array([px[i], py[i]])
        p1 = np.array([px[i + 1], py[i + 1]])
        w = p1 - p0
        ww = float(np.dot(w, w))
        if ww < 1e-18:
            continue
        t = float(np.clip(np.dot(np.array([x, y]) - p0, w) / ww, 0.0, 1.0))
        q = p0 + t * w
        d2 = float((x - q[0]) ** 2 + (y - q[1]) ** 2)
        if d2 < best_d2:
            best_d2 = d2
            tang = w / np.sqrt(ww)
            ps_i = float(np.arctan2(tang[1], tang[0]))
            n_left = np.array([-tang[1], tang[0]])
            e_y = float(np.dot(np.array([x, y]) - q, n_left))
            s_ref = float((1.0 - t) * s_cum[i] + t * s_cum[i + 1])
            xr, yr, ps_r = float(q[0]), float(q[1]), ps_i
    return s_ref, xr, yr, ps_r, e_y


def _segment_hint_from_s(s_query: float, s_cum: np.ndarray) -> int:
    """返回满足 s_cum[i] <= s_query 的最大 i（用于局部最近点搜索）。"""
    return int(np.clip(np.searchsorted(s_cum, s_query, side="right") - 1, 0, len(s_cum) - 2))


def _precompute_omega_ref(
    px: np.ndarray,
    py: np.ndarray,
    s_cum: np.ndarray,
    v_ref: float,
    *,
    omega_abs_max: float,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    折线顶点处估计 κ ≈ Δψ/Δs，ω_ref=v_ref·κ；Δs 下限避免顶点尖峰。
    结果限幅，避免参考横摆过大导致补偿发散。
    """
    n = len(px)
    if n < 2:
        return s_cum, np.zeros_like(s_cum)
    psi_seg = np.zeros(n - 1)
    for i in range(n - 1):
        psi_seg[i] = float(np.arctan2(py[i + 1] - py[i], px[i + 1] - px[i]))
    kappa = np.zeros(n)
    ds_floor = 0.28
    for i in range(1, n - 1):
        dpsi = _wrap_pi(psi_seg[i] - psi_seg[i - 1])
        ds = 0.5 * ((s_cum[i] - s_cum[i - 1]) + (s_cum[i + 1] - s_cum[i]))
        ds = max(float(ds), ds_floor)
        kappa[i] = dpsi / ds
    omega_ref_tab = float(v_ref) * kappa
    lim = max(float(omega_abs_max), 0.05)
    omega_ref_tab = np.clip(omega_ref_tab, -lim, lim)
    return s_cum, omega_ref_tab


def generate_random_puddles(
    s_total: float,
    *,
    n_puddle: int,
    seed: int,
    eta_v_range: Tuple[float, float],
    eta_w_range: Tuple[float, float],
    lateral_range: Tuple[float, float],
    width_range: Tuple[float, float],
    eta_v_fixed: Optional[float] = None,
    eta_omega_fixed: Optional[float] = None,
) -> List[Puddle]:
    rng = np.random.default_rng(seed)
    puddles: List[Puddle] = []
    margin = 0.05 * s_total
    tries = 0
    while len(puddles) < n_puddle and tries < n_puddle * 80:
        tries += 1
        w = float(rng.uniform(*width_range))
        s0 = float(rng.uniform(margin, max(s_total - margin - w, margin + 1e-3)))
        s1 = s0 + w
        if s1 > s_total - margin:
            continue
        overlap = False
        for p in puddles:
            if not (s1 < p.s0 - 0.5 or s0 > p.s1 + 0.5):
                overlap = True
                break
        if overlap:
            continue
        ev = float(eta_v_fixed) if eta_v_fixed is not None else float(rng.uniform(*eta_v_range))
        ew = (
            float(eta_omega_fixed)
            if eta_omega_fixed is not None
            else float(rng.uniform(*eta_w_range))
        )
        puddles.append(
            Puddle(
                s0=s0,
                s1=s1,
                eta_v=ev,
                eta_omega=ew,
                lateral_mag=float(rng.uniform(*lateral_range)),
                lateral_sign=float(rng.choice([-1.0, 1.0])),
            )
        )
    puddles.sort(key=lambda p: p.s0)
    return puddles


def _puddle_at_s(s: float, puddles: List[Puddle]) -> Tuple[float, float, float, float]:
    """返回 (eta_v, eta_w, lat_mag, lat_sign)，无坑时为 (1,1,0,0)。"""
    for p in puddles:
        if p.s0 <= s <= p.s1:
            return p.eta_v, p.eta_omega, p.lateral_mag, p.lateral_sign
    return 1.0, 1.0, 0.0, 0.0


def simulate_mixed_path(
    *,
    px: np.ndarray,
    py: np.ndarray,
    s_cum: np.ndarray,
    v_ref: float,
    dt: float,
    puddles: List[Puddle],
    use_compensation: bool,
    k_ey: float,
    k_psi: float,
    k_v: float,
    omega_cmd_max: float,
    v_cmd_max: float,
    omega_ref_abs_max: float,
    no_comp_heading_gain: float,
) -> Dict[str, np.ndarray]:
    """
    双积分单车：ẋ=v cosψ, ẏ=v sinψ, ψ̇=ω；水坑内 v_act=ηv v_cmd, ω_act=ηω ω_cmd，并加法向扰动。
    无补偿：v_cmd=v_ref；ω_cmd=ω_ref + k_h·e_ψ（仅极弱航向对齐，不含 η 前馈，避免纯开环在折线参考上误差积得过大）。
    有补偿：v_cmd≈v_ref/ηv + k_v·dist，ω_cmd=ω_ref/ηω + k_ey·(−e_y) + k_psi·e_ψ。
    """
    s_end = float(s_cum[-1])
    s_tab, omega_tab = _precompute_omega_ref(
        px, py, s_cum, v_ref, omega_abs_max=omega_ref_abs_max
    )
    t_nominal = s_end / max(float(v_ref), 0.1)

    x = float(px[0])
    y = float(py[0])
    psi = float(np.arctan2(py[1] - py[0], px[1] - px[0]))

    t = 0.0

    t_hist: List[float] = []
    x_hist: List[float] = []
    y_hist: List[float] = []
    x_ref_hist: List[float] = []
    y_ref_hist: List[float] = []
    e_y_hist: List[float] = []
    dist_hist: List[float] = []
    s_ref_hist: List[float] = []

    nseg = len(px) - 1
    # 约需步数：弧长 / (vmin·dt)，留余量
    v_min = max(0.15 * float(v_ref), 0.08)
    max_steps = min(int(np.ceil(s_end / (v_min * dt))) + 4000, 120000)
    s_prev = 0.0

    for _ in range(max_steps):
        ih = _segment_hint_from_s(s_prev, s_cum)
        i_lo = max(0, ih - 55)
        i_hi = min(nseg + 1, ih + 56)
        s_star, xr, yr, psir, e_y = _fix_closest_psi(
            x, y, px, py, s_cum, i_lo=i_lo, i_hi=i_hi
        )
        s_prev = s_star
        e_psi = _wrap_pi(psir - psi)
        # 沿路径弧长误差（近似）：用 s_star 与虚拟参考 s_ref_virt = v_ref*t 的差不宜用；改用位置距
        dist = float(np.hypot(x - xr, y - yr))

        eta_v, eta_w, lat_mag, lat_sign = _puddle_at_s(s_star, puddles)

        omega_ref = float(np.interp(s_star, s_tab, omega_tab))

        if not use_compensation:
            v_cmd_t = float(v_ref)
            omega_cmd = float(omega_ref) + float(no_comp_heading_gain) * e_psi
            omega_cmd = float(np.clip(omega_cmd, -float(omega_cmd_max), float(omega_cmd_max)))
        else:
            v_cmd_t = float(v_ref) / max(eta_v, 1e-6) + float(k_v) * dist
            v_cmd_t = float(np.clip(v_cmd_t, 0.0, float(v_cmd_max)))
            # e_y 为左法向：外侧为负时需加大 CCW 横摆，故用 −e_y
            omega_cmd = float(omega_ref) / max(eta_w, 1e-6)
            omega_cmd += float(k_ey) * (-e_y) + float(k_psi) * e_psi
            omega_cmd = float(np.clip(omega_cmd, -float(omega_cmd_max), float(omega_cmd_max)))

        v_act = float(eta_v) * v_cmd_t
        omega_act = float(eta_w) * omega_cmd

        # 左法向 n = (-sin ψr, cos ψr)（与 e_y 定义一致）
        nx = -float(np.sin(psir))
        ny = float(np.cos(psir))
        if lat_mag > 1e-12:
            vx_push = float(lat_sign) * float(lat_mag) * nx
            vy_push = float(lat_sign) * float(lat_mag) * ny
        else:
            vx_push = vy_push = 0.0

        x_dot = v_act * float(np.cos(psi)) + vx_push
        y_dot = v_act * float(np.sin(psi)) + vy_push
        x += x_dot * dt
        y += y_dot * dt
        psi = _wrap_pi(psi + omega_act * dt)
        t += dt

        t_hist.append(t)
        x_hist.append(x)
        y_hist.append(y)
        x_ref_hist.append(xr)
        y_ref_hist.append(yr)
        e_y_hist.append(e_y)
        dist_hist.append(dist)
        s_ref_hist.append(s_star)

        if s_star >= s_end * 0.98 and t > 0.45 * t_nominal:
            break
        if t > 8.0 * t_nominal:
            break

    return {
        "t": np.asarray(t_hist, dtype=float),
        "x": np.asarray(x_hist, dtype=float),
        "y": np.asarray(y_hist, dtype=float),
        "x_ref": np.asarray(x_ref_hist, dtype=float),
        "y_ref": np.asarray(y_ref_hist, dtype=float),
        "e_y": np.asarray(e_y_hist, dtype=float),
        "dist": np.asarray(dist_hist, dtype=float),
        "s_ref": np.asarray(s_ref_hist, dtype=float),
    }


def _rmse(a: np.ndarray) -> float:
    return float(np.sqrt(np.mean(np.square(a))))


def _setup_matplotlib_chinese_font() -> None:
    try:
        import matplotlib
        from matplotlib import font_manager
    except ImportError:
        return
    candidates = [
        "Noto Sans CJK SC",
        "Noto Sans CJK JP",
        "Source Han Sans SC",
        "WenQuanYi Micro Hei",
        "SimHei",
        "Microsoft YaHei",
    ]
    available = {f.name for f in font_manager.fontManager.ttflist}
    chosen = None
    for name in candidates:
        if name in available:
            chosen = name
            break
    if chosen is None:
        for f in font_manager.fontManager.ttflist:
            if "CJK" in f.name or "Hei" in f.name:
                chosen = f.name
                break
    if chosen is not None:
        matplotlib.rcParams["font.sans-serif"] = [chosen, "DejaVu Sans", "sans-serif"]
    matplotlib.rcParams["axes.unicode_minus"] = False


def _save_single_plot(fig, path: str, show: bool) -> None:
    out_dir = os.path.dirname(path)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    fig.savefig(path, dpi=160, bbox_inches="tight")
    print(f"已保存: {path}")
    import matplotlib.pyplot as plt

    if show:
        plt.show()
    else:
        plt.close(fig)


def run_mixed_experiment(
    *,
    v_ref: float,
    dt: float,
    seed: int,
    n_puddle: int,
    k_ey: float,
    k_psi: float,
    k_v: float,
    omega_cmd_max: float,
    v_cmd_max: float,
    omega_ref_abs_max: float,
    no_comp_heading_gain: float,
    path_amp: float,
    path_wave_len: float,
    path_ds: float,
    eta_v_min: float,
    eta_v_max: float,
    eta_w_min: float,
    eta_w_max: float,
    lateral_min: float,
    lateral_max: float,
    puddle_width_min: float,
    puddle_width_max: float,
    eta_v_fixed: Optional[float],
    eta_omega_fixed: Optional[float],
    save_prefix: str,
    show: bool,
) -> None:
    px, py, s_cum, _psi_path = build_s_lane_path(
        x_straight0=10.0,
        x_curve_end=42.0,
        x_straight1=58.0,
        amp=path_amp,
        wave_len=path_wave_len,
        ds_sample=path_ds,
    )
    s_total = float(s_cum[-1])
    puddles = generate_random_puddles(
        s_total,
        n_puddle=n_puddle,
        seed=seed,
        eta_v_range=(eta_v_min, eta_v_max),
        eta_w_range=(eta_w_min, eta_w_max),
        lateral_range=(lateral_min, lateral_max),
        width_range=(puddle_width_min, puddle_width_max),
        eta_v_fixed=eta_v_fixed,
        eta_omega_fixed=eta_omega_fixed,
    )

    no_comp = simulate_mixed_path(
        px=px,
        py=py,
        s_cum=s_cum,
        v_ref=v_ref,
        dt=dt,
        puddles=puddles,
        use_compensation=False,
        k_ey=k_ey,
        k_psi=k_psi,
        k_v=k_v,
        omega_cmd_max=omega_cmd_max,
        v_cmd_max=v_cmd_max,
        omega_ref_abs_max=omega_ref_abs_max,
        no_comp_heading_gain=no_comp_heading_gain,
    )
    with_comp = simulate_mixed_path(
        px=px,
        py=py,
        s_cum=s_cum,
        v_ref=v_ref,
        dt=dt,
        puddles=puddles,
        use_compensation=True,
        k_ey=k_ey,
        k_psi=k_psi,
        k_v=k_v,
        omega_cmd_max=omega_cmd_max,
        v_cmd_max=v_cmd_max,
        omega_ref_abs_max=omega_ref_abs_max,
        no_comp_heading_gain=no_comp_heading_gain,
    )

    rmse_no = _rmse(no_comp["dist"])
    rmse_w = _rmse(with_comp["dist"])
    if eta_v_fixed is not None:
        print(f"水坑纵向滑移系数 ηv = {eta_v_fixed}（固定）")
    if eta_omega_fixed is not None:
        print(f"水坑转向滑移系数 ηω = {eta_omega_fixed}（固定）")
    print(f"全局 RMSE(到参考折线距离): 无补偿 {rmse_no:.4f} m, 有补偿 {rmse_w:.4f} m")

    try:
        import matplotlib.pyplot as plt
    except ImportError as e:
        raise ImportError("需要 matplotlib") from e

    _setup_matplotlib_chinese_font()

    # --- X-Y ---
    fig_xy, ax = plt.subplots(figsize=(10.0, 5.2))
    ax.plot(px, py, "--", color="dimgray", lw=1.6, label="参考轨迹")
    for i, p in enumerate(puddles):
        mask = (s_cum >= p.s0) & (s_cum <= p.s1)
        if np.any(mask):
            ax.plot(px[mask], py[mask], color="orange", lw=4.0, alpha=0.35, zorder=1)
        if i == 0:
            ax.plot([], [], color="orange", lw=4.0, alpha=0.35, label="水坑区 (ηv,ηω↓)")
    ax.plot(no_comp["x"], no_comp["y"], "-", color="royalblue", lw=1.8, label="无补偿")
    ax.plot(with_comp["x"], with_comp["y"], "-", color="crimson", lw=1.8, label="有补偿")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_title("综合路段轨迹对比（直线 + S 弯 + 随机水坑）")
    ax.legend(loc="best", fontsize=8)
    _save_single_plot(fig_xy, f"{save_prefix}_xy.png", False)

    # --- RMSE bar ---
    fig_bar, axb = plt.subplots(figsize=(6.0, 5.0))
    labels = ["无补偿", "有补偿"]
    vals = [rmse_no, rmse_w]
    colors = ["royalblue", "crimson"]
    bars = axb.bar(labels, vals, color=colors, width=0.55, edgecolor="black", linewidth=0.6)
    axb.set_ylabel("RMSE [m]")
    axb.set_title("全局位置跟踪 RMSE（相对参考中心线最近点距离）")
    axb.grid(True, axis="y", alpha=0.3)
    for b, v in zip(bars, vals):
        axb.text(b.get_x() + b.get_width() / 2, v + 0.02 * max(vals), f"{v:.3f}", ha="center", va="bottom", fontsize=10)
    _save_single_plot(fig_bar, f"{save_prefix}_rmse.png", False)

    # --- combined: left xy, right bar ---
    fig_c, (axl, axr) = plt.subplots(
        1, 2, figsize=(12.5, 5.2), gridspec_kw={"width_ratios": [1.55, 1.0]}
    )
    axl.plot(px, py, "--", color="dimgray", lw=1.6, label="参考轨迹")
    for i, p in enumerate(puddles):
        mask = (s_cum >= p.s0) & (s_cum <= p.s1)
        if np.any(mask):
            axl.plot(px[mask], py[mask], color="orange", lw=4.0, alpha=0.35, zorder=1)
        if i == 0:
            axl.plot([], [], color="orange", lw=4.0, alpha=0.35, label="水坑区")
    axl.plot(no_comp["x"], no_comp["y"], "-", color="royalblue", lw=1.8, label="无补偿")
    axl.plot(with_comp["x"], with_comp["y"], "-", color="crimson", lw=1.8, label="有补偿")
    axl.set_aspect("equal", adjustable="box")
    axl.grid(True, alpha=0.3)
    axl.set_xlabel("X [m]")
    axl.set_ylabel("Y [m]")
    axl.set_title("(a) 轨迹对比")
    axl.legend(loc="best", fontsize=7)

    bars2 = axr.bar(labels, vals, color=colors, width=0.5, edgecolor="black", linewidth=0.6)
    axr.set_ylabel("RMSE [m]")
    axr.set_title("(b) 全局 RMSE")
    axr.grid(True, axis="y", alpha=0.3)
    for b, v in zip(bars2, vals):
        axr.text(b.get_x() + b.get_width() / 2, v + 0.02 * max(vals), f"{v:.3f}", ha="center", va="bottom", fontsize=10)
    fig_c.tight_layout()
    _save_single_plot(fig_c, f"{save_prefix}_combined.png", show)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="S 弯综合路段 + 随机水坑：轨迹与全局 RMSE 柱状图")
    parser.add_argument("--v-ref", type=float, default=1.0, help="名义前向速度 [m/s]")
    parser.add_argument("--dt", type=float, default=0.02, help="仿真步长 [s]")
    parser.add_argument("--seed", type=int, default=42, help="水坑随机种子（可复现）")
    parser.add_argument("--n-puddle", type=int, default=3, help="水坑数量")
    parser.add_argument("--k-ey", type=float, default=0.42, help="横向误差增益")
    parser.add_argument("--k-psi", type=float, default=1.35, help="航向误差增益")
    parser.add_argument("--k-v", type=float, default=0.12, help="距离误差对纵向指令增益")
    parser.add_argument("--omega-cmd-max", type=float, default=0.52, help="横摆指令限幅 [rad/s]")
    parser.add_argument("--omega-ref-abs-max", type=float, default=0.38, help="参考横摆 |ω_ref| 上限 [rad/s]")
    parser.add_argument(
        "--no-comp-heading-gain",
        type=float,
        default=0.34,
        help="无补偿组仅航向弱反馈 k·e_ψ（不含滑移 η 前馈）；略大可减小开环漂移",
    )
    parser.add_argument("--path-amp", type=float, default=3.15, help="S 弯正弦幅值 A [m]")
    parser.add_argument("--path-wave-len", type=float, default=19.0, help="S 弯正弦波长 [m]")
    parser.add_argument("--path-ds", type=float, default=0.08, help="参考轨迹折线采样步长 [m]")
    parser.add_argument(
        "--eta-v",
        type=float,
        default=None,
        dest="eta_v_fixed",
        help="水坑纵向滑移系数 ηv（各坑相同、固定）；省略则按 --eta-v-min/max 随机",
    )
    parser.add_argument(
        "--eta-omega",
        type=float,
        default=None,
        dest="eta_omega_fixed",
        help="水坑转向滑移系数 ηω（各坑相同、固定）；省略则按 --eta-w-min/max 随机",
    )
    parser.add_argument(
        "--eta-v-min",
        type=float,
        default=0.62,
        help="未指定 --eta-v 时，水坑内 ηv 随机区间下限",
    )
    parser.add_argument(
        "--eta-v-max",
        type=float,
        default=0.78,
        help="未指定 --eta-v 时，水坑内 ηv 随机区间上限",
    )
    parser.add_argument(
        "--eta-w-min",
        type=float,
        default=0.72,
        help="未指定 --eta-omega 时，水坑内 ηω 随机区间下限",
    )
    parser.add_argument(
        "--eta-w-max",
        type=float,
        default=0.88,
        help="未指定 --eta-omega 时，水坑内 ηω 随机区间上限",
    )
    parser.add_argument("--lateral-min", type=float, default=0.035, help="水坑侧向推力随机下限 [m/s]")
    parser.add_argument("--lateral-max", type=float, default=0.09, help="水坑侧向推力随机上限 [m/s]")
    parser.add_argument("--puddle-width-min", type=float, default=2.4, help="水坑沿弧长宽度下限 [m]")
    parser.add_argument("--puddle-width-max", type=float, default=4.8, help="水坑沿弧长宽度上限 [m]")
    parser.add_argument("--v-cmd-max", type=float, default=1.85, help="速度指令上限 [m/s]")
    parser.add_argument(
        "--save-prefix",
        type=str,
        default="mpc_expriments/docs/综合抗扰_mix",
        help="输出 *_xy.png, *_rmse.png, *_combined.png",
    )
    parser.add_argument("--no-show", action="store_true", help="仅保存不弹窗")
    args = parser.parse_args(argv)

    if float(args.dt) <= 0.0:
        raise ValueError("--dt 必须大于 0")
    if float(args.path_ds) <= 0.0:
        raise ValueError("--path-ds 必须大于 0")
    if float(args.eta_v_min) > float(args.eta_v_max):
        raise ValueError("--eta-v-min 不能大于 --eta-v-max")
    if float(args.eta_w_min) > float(args.eta_w_max):
        raise ValueError("--eta-w-min 不能大于 --eta-w-max")
    if float(args.lateral_min) > float(args.lateral_max):
        raise ValueError("--lateral-min 不能大于 --lateral-max")
    if float(args.puddle_width_min) > float(args.puddle_width_max):
        raise ValueError("--puddle-width-min 不能大于 --puddle-width-max")
    if args.eta_v_fixed is not None and not (0.0 < float(args.eta_v_fixed) <= 1.0):
        raise ValueError("--eta-v 必须在 (0,1]")
    if args.eta_omega_fixed is not None and not (0.0 < float(args.eta_omega_fixed) <= 1.0):
        raise ValueError("--eta-omega 必须在 (0,1]")

    run_mixed_experiment(
        v_ref=float(args.v_ref),
        dt=float(args.dt),
        seed=int(args.seed),
        n_puddle=int(args.n_puddle),
        k_ey=float(args.k_ey),
        k_psi=float(args.k_psi),
        k_v=float(args.k_v),
        omega_cmd_max=float(args.omega_cmd_max),
        v_cmd_max=float(args.v_cmd_max),
        omega_ref_abs_max=float(args.omega_ref_abs_max),
        no_comp_heading_gain=float(args.no_comp_heading_gain),
        path_amp=float(args.path_amp),
        path_wave_len=float(args.path_wave_len),
        path_ds=float(args.path_ds),
        eta_v_min=float(args.eta_v_min),
        eta_v_max=float(args.eta_v_max),
        eta_w_min=float(args.eta_w_min),
        eta_w_max=float(args.eta_w_max),
        lateral_min=float(args.lateral_min),
        lateral_max=float(args.lateral_max),
        puddle_width_min=float(args.puddle_width_min),
        puddle_width_max=float(args.puddle_width_max),
        eta_v_fixed=args.eta_v_fixed,
        eta_omega_fixed=args.eta_omega_fixed,
        save_prefix=str(args.save_prefix),
        show=not bool(args.no_show),
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
