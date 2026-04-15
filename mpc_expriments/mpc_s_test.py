"""转向滑移 + 打滑区离心外甩（径向侧滑）：无补偿 vs 残差补偿（论文曲线）。"""

from __future__ import annotations

import argparse
import os
from typing import Dict, Tuple

import numpy as np


def _wrap_pi(angle: float) -> float:
    return float((angle + np.pi) % (2.0 * np.pi) - np.pi)


def _reference_arc(
    t: float,
    *,
    v_ref: float,
    radius: float,
    theta_max: float,
) -> Tuple[float, float, float, float, float]:
    """
    左转弯圆弧：起点 (0,0)、初始航向沿 +x，圆心在 (0, R)。

    返回 (x_ref, y_ref, psi_ref, omega_ref, theta) ；omega_ref = v/R（弧内常数）。
    """
    theta = min(float(v_ref) * t / float(radius), float(theta_max))
    x_ref = float(radius) * np.sin(theta)
    y_ref = float(radius) * (1.0 - np.cos(theta))
    psi_ref = float(theta)
    omega_ref = float(v_ref) / float(radius) if theta < float(theta_max) - 1e-9 else 0.0
    return x_ref, y_ref, psi_ref, omega_ref, theta


def _lateral_error_circle(
    x: float,
    y: float,
    *,
    radius: float,
) -> float:
    """有符号横向偏离：左弧、圆心 (0,R)，外为正（半径大于 R）。"""
    cx, cy = 0.0, float(radius)
    d = float(np.hypot(x - cx, y - cy))
    return d - float(radius)


def _outward_radial_unit(
    x: float,
    y: float,
    *,
    cx: float,
    cy: float,
) -> tuple[float, float]:
    """
    相对圆心的外法向单位向量 u = (P − C) / ||P−C||。
    左转弯、圆心在弯道内侧时，沿 u 运动等价于被往弯道外侧“甩”。
    """
    rx = float(x) - float(cx)
    ry = float(y) - float(cy)
    n = float(np.hypot(rx, ry))
    if n < 1e-9:
        return 1.0, 0.0
    return rx / n, ry / n


def simulate_turn_slip_lateral(
    *,
    v_ref: float,
    radius: float,
    theta_max: float,
    dt: float,
    eta_omega: float,
    use_compensation: bool,
    slip_outward_v: float,
    slip_t0: float,
    slip_t1: float,
    k_ey: float,
    k_psi: float,
    omega_cmd_max: float,
) -> Dict[str, np.ndarray]:
    """
    单车模型：体轴前向速度 v_ref 常数；横摆 ω_cmd 由路径/补偿给出；
    实际横摆 ω_act = ηω * ω_cmd。

    打滑区 [slip_t0, slip_t1]：叠加相对圆心 (0,R) 的径向离心外甩速度 slip_outward_v·u_out
    （世界系），使 e_y 单调恶化而非被“推向圆心”。体轴 v_yb 易随 ψ 转向产生非物理的内侧分量，故不用。

    无补偿：ω_cmd = ω_ref，ω_act = ηω·ω_cmd → 转向不足、轨迹外扩。
    有补偿：ω_cmd = ω_ref/ηω + k_ey·e_y + k_psi·e_ψ；e_y 增大时 ω_cmd 上调（Δω 尖峰救车）。
    """
    t = 0.0
    x = 0.0
    y = 0.0
    psi = 0.0

    t_hist = []
    x_hist = []
    y_hist = []
    x_ref_hist = []
    y_ref_hist = []
    ey_hist = []
    omega_ref_hist = []
    omega_cmd_hist = []
    delta_omega_hist = []
    v_slip_radial_hist = []

    cx, cy = 0.0, float(radius)

    # 参考弧段时间：扫完 theta_max
    t_arc = float(theta_max) * float(radius) / float(v_ref)
    max_steps = int(np.ceil((t_arc + 2.0) / dt)) + 2000

    for _ in range(max_steps):
        x_r, y_r, psi_r, omega_r, theta_prog = _reference_arc(
            t, v_ref=v_ref, radius=radius, theta_max=theta_max
        )
        e_y = _lateral_error_circle(x, y, radius=radius)
        e_psi = _wrap_pi(psi_r - psi)

        if slip_t0 <= t <= slip_t1:
            uox, uoy = _outward_radial_unit(x, y, cx=cx, cy=cy)
            v_slip = float(slip_outward_v)
        else:
            v_slip = 0.0
            uox, uoy = 0.0, 0.0

        if not use_compensation:
            omega_cmd = float(omega_r)
        else:
            omega_cmd = float(omega_r) / max(float(eta_omega), 1e-6)
            omega_cmd += float(k_ey) * e_y + float(k_psi) * e_psi
            omega_cmd = float(np.clip(omega_cmd, -float(omega_cmd_max), float(omega_cmd_max)))

        omega_act = float(eta_omega) * omega_cmd

        # 前向 v_ref（体轴 xb）+ 打滑区径向外甩（世界系，与 ψ 无关）
        x_dot = float(v_ref) * np.cos(psi) + v_slip * uox
        y_dot = float(v_ref) * np.sin(psi) + v_slip * uoy

        x += x_dot * dt
        y += y_dot * dt
        psi = _wrap_pi(psi + omega_act * dt)
        t += dt

        delta_omega = omega_cmd - float(omega_r)

        t_hist.append(t)
        x_hist.append(x)
        y_hist.append(y)
        x_ref_hist.append(x_r)
        y_ref_hist.append(y_r)
        ey_hist.append(e_y)
        omega_ref_hist.append(omega_r)
        omega_cmd_hist.append(omega_cmd)
        delta_omega_hist.append(delta_omega)
        v_slip_radial_hist.append(v_slip)

        if t >= t_arc:
            break

    return {
        "t": np.asarray(t_hist, dtype=float),
        "x": np.asarray(x_hist, dtype=float),
        "y": np.asarray(y_hist, dtype=float),
        "x_ref": np.asarray(x_ref_hist, dtype=float),
        "y_ref": np.asarray(y_ref_hist, dtype=float),
        "e_y": np.asarray(ey_hist, dtype=float),
        "omega_ref": np.asarray(omega_ref_hist, dtype=float),
        "omega_cmd": np.asarray(omega_cmd_hist, dtype=float),
        "delta_omega": np.asarray(delta_omega_hist, dtype=float),
        "v_slip_radial": np.asarray(v_slip_radial_hist, dtype=float),
    }


def _setup_matplotlib_chinese_font() -> None:
    try:
        import matplotlib
        from matplotlib import font_manager
    except ImportError:
        return
    candidates = [
        "Noto Sans CJK SC",
        "Noto Sans CJK JP",
        "Noto Serif CJK SC",
        "Source Han Sans SC",
        "WenQuanYi Micro Hei",
        "WenQuanYi Zen Hei",
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
            if "CJK" in f.name or "Hei" in f.name or "WenQuanYi" in f.name:
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


def run_turn_slip_experiment(
    *,
    v_ref: float,
    radius: float,
    theta_max: float,
    dt: float,
    eta_omega: float,
    slip_outward_v: float,
    slip_t0: float,
    slip_t1: float,
    k_ey: float,
    k_psi: float,
    omega_cmd_max: float,
    save_prefix: str,
    show: bool,
) -> None:
    no_comp = simulate_turn_slip_lateral(
        v_ref=v_ref,
        radius=radius,
        theta_max=theta_max,
        dt=dt,
        eta_omega=eta_omega,
        use_compensation=False,
        slip_outward_v=slip_outward_v,
        slip_t0=slip_t0,
        slip_t1=slip_t1,
        k_ey=k_ey,
        k_psi=k_psi,
        omega_cmd_max=omega_cmd_max,
    )
    with_comp = simulate_turn_slip_lateral(
        v_ref=v_ref,
        radius=radius,
        theta_max=theta_max,
        dt=dt,
        eta_omega=eta_omega,
        use_compensation=True,
        slip_outward_v=slip_outward_v,
        slip_t0=slip_t0,
        slip_t1=slip_t1,
        k_ey=k_ey,
        k_psi=k_psi,
        omega_cmd_max=omega_cmd_max,
    )

    try:
        import matplotlib.pyplot as plt
    except ImportError as e:
        raise ImportError("需要 matplotlib") from e

    _setup_matplotlib_chinese_font()

    t_max = max(float(no_comp["t"][-1]), float(with_comp["t"][-1])) * 1.02

    def _plot_xy(ax) -> None:
        ax.plot(
            no_comp["x_ref"],
            no_comp["y_ref"],
            "--",
            color="dimgray",
            lw=1.8,
            label="参考圆弧",
        )
        ax.plot(no_comp["x"], no_comp["y"], "-", color="royalblue", lw=2.0, label="无补偿 轨迹")
        ax.plot(with_comp["x"], with_comp["y"], "-", color="crimson", lw=2.0, label="转向补偿 轨迹")
        ax.set_aspect("equal", adjustable="box")
        ax.grid(True, alpha=0.3)
        ax.set_xlabel("X [m]")
        ax.set_ylabel("Y [m]")
        ax.set_title("轨迹对比（X–Y）：转弯处外扩 vs 贴弧")
        ax.legend(loc="best", fontsize=8)

    def _plot_ey(ax) -> None:
        ax.plot(no_comp["t"], no_comp["e_y"], "-", color="royalblue", lw=2.0, label="无补偿 e_y")
        ax.plot(with_comp["t"], with_comp["e_y"], "-", color="crimson", lw=2.0, label="转向补偿 e_y")
        ax.axvspan(slip_t0, slip_t1, color="orange", alpha=0.15, label="打滑区：径向外甩")
        ax.set_xlim(0.0, t_max)
        ax.grid(True, alpha=0.3)
        ax.set_xlabel("时间 t [s]")
        ax.set_ylabel("e_y [m]")
        ax.set_title("横向跟踪误差 e_y – t")
        ax.legend(loc="best", fontsize=8)

    def _plot_domega(ax) -> None:
        ax.plot(
            no_comp["t"],
            no_comp["delta_omega"],
            "-",
            color="royalblue",
            lw=2.0,
            label="无补偿 Δω（≈0）",
        )
        ax.plot(
            with_comp["t"],
            with_comp["delta_omega"],
            "-",
            color="crimson",
            lw=2.0,
            label="转向补偿 Δω = ω_cmd − ω_ref",
        )
        ax.axvspan(slip_t0, slip_t1, color="orange", alpha=0.15, label="打滑区：径向外甩")
        ax.axhline(0.0, color="gray", ls=":", lw=0.8)
        ax.set_xlim(0.0, t_max)
        ax.grid(True, alpha=0.3)
        ax.set_xlabel("时间 t [s]")
        ax.set_ylabel("Δω [rad/s]")
        ax.set_title("角速度补偿量 Δω – t")
        ax.legend(loc="best", fontsize=8)

    fig1, ax1 = plt.subplots(figsize=(8.0, 7.0))
    _plot_xy(ax1)
    _save_single_plot(fig1, f"{save_prefix}_xy.png", False)

    fig2, ax2 = plt.subplots(figsize=(8.8, 4.6))
    _plot_ey(ax2)
    _save_single_plot(fig2, f"{save_prefix}_ey.png", False)

    fig3, ax3 = plt.subplots(figsize=(8.8, 4.6))
    _plot_domega(ax3)
    _save_single_plot(fig3, f"{save_prefix}_domega.png", False)

    fig_c, axes_c = plt.subplots(3, 1, figsize=(8.8, 11.2), constrained_layout=True)
    _plot_xy(axes_c[0])
    _plot_ey(axes_c[1])
    _plot_domega(axes_c[2])
    for i, ax in enumerate(axes_c):
        ax.text(
            -0.07,
            1.02,
            f"({chr(ord('a') + i)})",
            transform=ax.transAxes,
            fontsize=11,
            fontweight="bold",
        )
    _save_single_plot(fig_c, f"{save_prefix}_combined.png", show)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="转向滑移 ηω + 打滑区径向离心外甩：轨迹 / e_y / Δω"
    )
    parser.add_argument("--v-ref", type=float, default=1.0, help="参考前向速度 [m/s]")
    parser.add_argument("--radius", type=float, default=8.0, help="参考圆弧半径 R [m]")
    parser.add_argument(
        "--theta-max",
        type=float,
        default=float(np.pi / 2),
        help="圆弧扫角上限 [rad]，默认 π/2（四分之一圆）",
    )
    parser.add_argument("--dt", type=float, default=0.02, help="仿真步长 [s]")
    parser.add_argument(
        "--eta-omega",
        type=float,
        default=0.7,
        help="转向滑移系数 ηω：实际角速度 = ηω·ω_cmd",
    )
    parser.add_argument(
        "--slip-outward-v",
        type=float,
        default=0.24,
        help="打滑区等效径向外甩速度 [m/s]（沿 P−C 方向，C 为参考圆弧圆心）",
    )
    parser.add_argument("--slip-t0", type=float, default=3.0, help="打滑区起始时间 [s]")
    parser.add_argument("--slip-t1", type=float, default=5.5, help="打滑区结束时间 [s]")
    parser.add_argument("--k-ey", type=float, default=0.55, help="横向误差反馈增益（越大 Δω 尖峰越猛）")
    parser.add_argument("--k-psi", type=float, default=1.35, help="航向误差反馈增益")
    parser.add_argument("--omega-cmd-max", type=float, default=0.9, help="横摆指令限幅 [rad/s]")
    parser.add_argument(
        "--save-prefix",
        type=str,
        default="mpc_expriments/docs/转向滑移侧漂对比",
        help="输出前缀：*_xy.png, *_ey.png, *_domega.png, *_combined.png",
    )
    parser.add_argument("--no-show", action="store_true", help="仅保存不弹窗")
    args = parser.parse_args(argv)

    if not (0.0 < float(args.eta_omega) <= 1.0):
        raise ValueError("--eta-omega 必须在 (0,1]")
    if float(args.dt) <= 0.0:
        raise ValueError("--dt 必须大于 0")
    if float(args.v_ref) <= 0.0:
        raise ValueError("--v-ref 必须大于 0")
    if float(args.radius) <= 0.0:
        raise ValueError("--radius 必须大于 0")
    if float(args.slip_t1) <= float(args.slip_t0):
        raise ValueError("--slip-t1 必须大于 --slip-t0")

    run_turn_slip_experiment(
        v_ref=float(args.v_ref),
        radius=float(args.radius),
        theta_max=float(args.theta_max),
        dt=float(args.dt),
        eta_omega=float(args.eta_omega),
        slip_outward_v=float(args.slip_outward_v),
        slip_t0=float(args.slip_t0),
        slip_t1=float(args.slip_t1),
        k_ey=float(args.k_ey),
        k_psi=float(args.k_psi),
        omega_cmd_max=float(args.omega_cmd_max),
        save_prefix=str(args.save_prefix),
        show=not bool(args.no_show),
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
