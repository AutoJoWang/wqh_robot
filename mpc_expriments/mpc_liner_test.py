"""直线 x 轴阶跃打滑实验：无补偿 vs 运动学残差补偿。"""

from __future__ import annotations

import argparse
import os
from typing import Dict

import numpy as np


def step_slip_profile(x_pos: float, *, x1: float, x2: float, eta_low: float) -> float:
    """阶跃滑移模型：x∈[x1,x2] 使用低附着滑移率。"""
    if x_pos < x1:
        return 1.0
    if x_pos <= x2:
        return float(eta_low)
    return 1.0


def simulate_straight_line_step_disturbance(
    *,
    x_goal: float,
    v_ref: float,
    dt: float,
    eta_low: float,
    disturb_start: float,
    disturb_end: float,
    use_compensation: bool,
    v_cmd_max: float,
    k_track: float,
    k_slip_ff: float,
    cmd_rate_limit: float,
    cmd_tau: float,
) -> Dict[str, np.ndarray]:
    """
    直线沿 x 轴，积分至 x >= x_goal（两组可不同到达时间）。

    x < x1（未进入打滑区）:
      无补偿与有补偿共用同一套执行器：跟踪 v_ref，从静止起步，曲线重合。

    x1 <= x <= x2:
      无补偿: v_act = eta_low * v_ref（恒定）
      有补偿: 前馈 + 执行器，v_act 可波动

    x > x2:
      无补偿: v_act = v_ref
      有补偿: eta=1，执行器跟踪 v_ref
    """
    t = 0.0
    x = 0.0
    v_cmd_applied = 0.0

    t_hist = []
    x_ref_hist = []
    x_act_hist = []
    ex_hist = []
    v_cmd_hist = []
    v_act_hist = []
    eta_hist = []

    # 慢车需更多步数；两组终点时间可不同
    max_steps = int(np.ceil(x_goal / (v_ref * eta_low * 0.5) / dt)) + 5000

    for _ in range(max_steps):
        x_ref = min(float(x_goal), float(v_ref) * t)
        e_x = x_ref - x
        in_slip = float(disturb_start) <= x <= float(disturb_end)

        if x < float(disturb_start):
            # 进入打滑区之前：两组完全相同的起步（从 0 爬升到约 v_ref）
            eta = 1.0
            v_cmd_target = float(v_ref) + float(k_track) * e_x
            v_cmd_target = float(np.clip(v_cmd_target, 0.0, float(v_cmd_max)))
            dv_des = (v_cmd_target - v_cmd_applied) / max(float(cmd_tau), 1e-6)
            dv_des = float(np.clip(dv_des, -float(cmd_rate_limit), float(cmd_rate_limit)))
            v_cmd_applied = float(np.clip(v_cmd_applied + dv_des * dt, 0.0, float(v_cmd_max)))
            v_act = v_cmd_applied
        elif in_slip:
            eta = float(eta_low)
            if not use_compensation:
                v_act = eta * float(v_ref)
                v_cmd_applied = v_act
            else:
                v_cmd_target = float(v_ref) + float(k_track) * e_x
                v_cmd_target += float(k_slip_ff) * float(v_ref) * (1.0 / max(eta, 1e-6) - 1.0)
                v_cmd_target = float(np.clip(v_cmd_target, 0.0, float(v_cmd_max)))
                dv_des = (v_cmd_target - v_cmd_applied) / max(float(cmd_tau), 1e-6)
                dv_des = float(np.clip(dv_des, -float(cmd_rate_limit), float(cmd_rate_limit)))
                v_cmd_applied = float(np.clip(v_cmd_applied + dv_des * dt, 0.0, float(v_cmd_max)))
                v_act = eta * v_cmd_applied
        else:
            # x > disturb_end
            eta = 1.0
            if not use_compensation:
                v_act = float(v_ref)
                v_cmd_applied = v_act
            else:
                v_cmd_target = float(v_ref) + float(k_track) * e_x
                v_cmd_target = float(np.clip(v_cmd_target, 0.0, float(v_cmd_max)))
                dv_des = (v_cmd_target - v_cmd_applied) / max(float(cmd_tau), 1e-6)
                dv_des = float(np.clip(dv_des, -float(cmd_rate_limit), float(cmd_rate_limit)))
                v_cmd_applied = float(np.clip(v_cmd_applied + dv_des * dt, 0.0, float(v_cmd_max)))
                v_act = v_cmd_applied

        x = x + v_act * dt
        t = t + dt

        t_hist.append(t)
        x_ref_hist.append(x_ref)
        x_act_hist.append(x)
        ex_hist.append(x_ref - x)
        v_cmd_hist.append(v_cmd_applied)
        v_act_hist.append(v_act)
        eta_hist.append(eta)

        if x >= x_goal:
            break

    return {
        "t": np.asarray(t_hist, dtype=float),
        "x_ref": np.asarray(x_ref_hist, dtype=float),
        "x_act": np.asarray(x_act_hist, dtype=float),
        "e_x": np.asarray(ex_hist, dtype=float),
        "v_cmd": np.asarray(v_cmd_hist, dtype=float),
        "v_act": np.asarray(v_act_hist, dtype=float),
        "eta": np.asarray(eta_hist, dtype=float),
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
        "Noto Serif CJK JP",
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


def _disturbance_time_window(result: Dict[str, np.ndarray], x1: float, x2: float) -> tuple[float, float] | None:
    x_act = result["x_act"]
    t = result["t"]
    idx = np.where((x_act >= x1) & (x_act <= x2))[0]
    if idx.size == 0:
        return None
    return float(t[idx[0]]), float(t[idx[-1]])


def _save_single_plot(fig, path: str, show: bool) -> None:
    out_dir = os.path.dirname(path)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    fig.savefig(path, dpi=160, bbox_inches="tight")
    print(f"已保存: {path}")
    if show:
        import matplotlib.pyplot as plt
        plt.show()
    else:
        import matplotlib.pyplot as plt
        plt.close(fig)


def _slip_time_union(
    window_no: tuple[float, float] | None,
    window_w: tuple[float, float] | None,
) -> tuple[float, float] | None:
    if window_no is not None and window_w is not None:
        return min(window_no[0], window_w[0]), max(window_no[1], window_w[1])
    if window_no is not None:
        return window_no[0], window_no[1]
    if window_w is not None:
        return window_w[0], window_w[1]
    return None


def _plot_panel_xt(
    ax,
    *,
    no_comp: Dict[str, np.ndarray],
    with_comp: Dict[str, np.ndarray],
    disturb_start: float,
    disturb_end: float,
    t_plot_max: float,
) -> None:
    ax.plot(no_comp["t"], no_comp["x_act"], "-", color="royalblue", lw=2.0, label="无补偿 X")
    ax.plot(with_comp["t"], with_comp["x_act"], "-", color="crimson", lw=2.0, label="滑移补偿 X")
    ax.axhspan(disturb_start, disturb_end, color="gray", alpha=0.18, label="x∈[x1,x2] 打滑区")
    ax.set_xlim(0.0, t_plot_max)
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("时间 t [s]")
    ax.set_ylabel("X [m]")
    ax.set_title("绝对位移跟踪曲线：X - t")
    ax.legend(loc="best", fontsize=8)


def _plot_panel_ex(
    ax,
    *,
    no_comp: Dict[str, np.ndarray],
    with_comp: Dict[str, np.ndarray],
    disturb_start: float,
    disturb_end: float,
) -> None:
    ax.plot(no_comp["x_act"], no_comp["e_x"], "-", color="royalblue", lw=2.0, label="无补偿 e_x")
    ax.plot(with_comp["x_act"], with_comp["e_x"], "-", color="crimson", lw=2.0, label="滑移补偿 e_x")
    ax.axvspan(disturb_start, disturb_end, color="gray", alpha=0.18, label="x∈[x1,x2] 打滑区")
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("直线距离 x [m]")
    ax.set_ylabel("error_x [m]")
    ax.set_title("纵向跟踪误差曲线：error-x")
    ax.legend(loc="best", fontsize=8)


def _plot_panel_v(
    ax,
    *,
    no_comp: Dict[str, np.ndarray],
    with_comp: Dict[str, np.ndarray],
    t_plot_max: float,
    slip_t0: float | None,
    slip_t1: float | None,
) -> None:
    ax.plot(no_comp["t"], no_comp["v_act"], "-", color="royalblue", lw=2.0, label="无补偿 v_act")
    ax.plot(with_comp["t"], with_comp["v_act"], "-", color="crimson", lw=2.0, label="滑移补偿 v_act")
    if slip_t0 is not None:
        ax.axvspan(slip_t0, slip_t1, color="gray", alpha=0.18, label="打滑区(时间并集)")
    ax.set_xlim(0.0, t_plot_max)
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("时间 t [s]")
    ax.set_ylabel("速度 [m/s]")
    ax.set_title("实际线速度曲线：v_act - t")
    ax.legend(loc="best", fontsize=8)


def _plot_disturbance_panels(
    ax_xt,
    ax_ex,
    ax_v,
    *,
    no_comp: Dict[str, np.ndarray],
    with_comp: Dict[str, np.ndarray],
    disturb_start: float,
    disturb_end: float,
    t_plot_max: float,
    window_no: tuple[float, float] | None,
    window_w: tuple[float, float] | None,
    panel_labels: bool = False,
) -> None:
    slip_u = _slip_time_union(window_no, window_w)
    slip_t0 = slip_u[0] if slip_u else None
    slip_t1 = slip_u[1] if slip_u else None

    _plot_panel_xt(
        ax_xt,
        no_comp=no_comp,
        with_comp=with_comp,
        disturb_start=disturb_start,
        disturb_end=disturb_end,
        t_plot_max=t_plot_max,
    )
    _plot_panel_ex(
        ax_ex,
        no_comp=no_comp,
        with_comp=with_comp,
        disturb_start=disturb_start,
        disturb_end=disturb_end,
    )
    _plot_panel_v(
        ax_v,
        no_comp=no_comp,
        with_comp=with_comp,
        t_plot_max=t_plot_max,
        slip_t0=slip_t0,
        slip_t1=slip_t1,
    )

    if panel_labels:
        for i, ax in enumerate((ax_xt, ax_ex, ax_v)):
            ax.text(
                -0.08,
                1.02,
                f"({chr(ord('a') + i)})",
                transform=ax.transAxes,
                fontsize=11,
                fontweight="bold",
            )


def run_step_disturbance_experiment(
    *,
    x_end: float,
    eta_low: float,
    disturb_start: float,
    disturb_end: float,
    x_ref_speed: float,
    dt: float,
    v_cmd_max: float,
    k_track: float,
    k_slip_ff: float,
    cmd_rate_limit: float,
    cmd_tau: float,
    save_prefix: str,
    show: bool,
) -> None:
    no_comp = simulate_straight_line_step_disturbance(
        x_goal=x_end,
        v_ref=x_ref_speed,
        dt=dt,
        eta_low=eta_low,
        disturb_start=disturb_start,
        disturb_end=disturb_end,
        use_compensation=False,
        v_cmd_max=v_cmd_max,
        k_track=k_track,
        k_slip_ff=k_slip_ff,
        cmd_rate_limit=cmd_rate_limit,
        cmd_tau=cmd_tau,
    )
    with_comp = simulate_straight_line_step_disturbance(
        x_goal=x_end,
        v_ref=x_ref_speed,
        dt=dt,
        eta_low=eta_low,
        disturb_start=disturb_start,
        disturb_end=disturb_end,
        use_compensation=True,
        v_cmd_max=v_cmd_max,
        k_track=k_track,
        k_slip_ff=k_slip_ff,
        cmd_rate_limit=cmd_rate_limit,
        cmd_tau=cmd_tau,
    )

    try:
        import matplotlib.pyplot as plt
    except ImportError as e:
        raise ImportError("需要 matplotlib") from e

    _setup_matplotlib_chinese_font()

    window_no = _disturbance_time_window(no_comp, disturb_start, disturb_end)
    window_w = _disturbance_time_window(with_comp, disturb_start, disturb_end)
    t_plot_max = max(float(no_comp["t"][-1]), float(with_comp["t"][-1])) * 1.02
    slip_u = _slip_time_union(window_no, window_w)
    slip_t0 = slip_u[0] if slip_u else None
    slip_t1 = slip_u[1] if slip_u else None

    # 单图（论文排版也可用分栏；与合并图数据一致）
    fig1, ax1 = plt.subplots(figsize=(8.8, 4.6))
    _plot_panel_xt(
        ax1,
        no_comp=no_comp,
        with_comp=with_comp,
        disturb_start=disturb_start,
        disturb_end=disturb_end,
        t_plot_max=t_plot_max,
    )
    _save_single_plot(fig1, f"{save_prefix}_xt.png", False)

    fig2, ax2 = plt.subplots(figsize=(8.8, 4.6))
    _plot_panel_ex(
        ax2,
        no_comp=no_comp,
        with_comp=with_comp,
        disturb_start=disturb_start,
        disturb_end=disturb_end,
    )
    _save_single_plot(fig2, f"{save_prefix}_ex.png", False)

    fig3, ax3 = plt.subplots(figsize=(8.8, 4.6))
    _plot_panel_v(
        ax3,
        no_comp=no_comp,
        with_comp=with_comp,
        t_plot_max=t_plot_max,
        slip_t0=slip_t0,
        slip_t1=slip_t1,
    )
    _save_single_plot(fig3, f"{save_prefix}_vact.png", False)

    # 三合一（便于直接插入论文）
    fig_c, axes_c = plt.subplots(3, 1, figsize=(8.8, 11.2), constrained_layout=True)
    _plot_disturbance_panels(
        axes_c[0],
        axes_c[1],
        axes_c[2],
        no_comp=no_comp,
        with_comp=with_comp,
        disturb_start=disturb_start,
        disturb_end=disturb_end,
        t_plot_max=t_plot_max,
        window_no=window_no,
        window_w=window_w,
        panel_labels=True,
    )
    _save_single_plot(fig_c, f"{save_prefix}_combined.png", show)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="直线 0→12m 阶跃打滑：无补偿 vs 补偿（X-t / error-x / v_act-t）"
    )
    parser.add_argument("--x-end", type=float, default=12.0, help="终点 x [m]")
    parser.add_argument("--disturb-start", type=float, default=2.0, help="扰动起点 x [m]")
    parser.add_argument("--disturb-end", type=float, default=6.0, help="扰动终点 x [m]")
    parser.add_argument("--eta-low", type=float, default=0.6, help="打滑区纵向滑移系数 eta_v")
    parser.add_argument("--x-ref-speed", type=float, default=1.0, help="目标速度 v_ref [m/s]")
    parser.add_argument("--dt", type=float, default=0.05, help="仿真步长 [s]")
    parser.add_argument("--v-cmd-max", type=float, default=2.0, help="最大下发速度 [m/s]")
    parser.add_argument("--k-track", type=float, default=0.6, help="误差反馈增益")
    parser.add_argument("--k-slip-ff", type=float, default=0.9, help="滑移补偿前馈增益")
    parser.add_argument("--cmd-rate-limit", type=float, default=3.5, help="执行器速度指令变化率上限 [m/s^2]")
    parser.add_argument("--cmd-tau", type=float, default=0.18, help="执行器一阶滞后时间常数 [s]")
    parser.add_argument(
        "--save-prefix",
        type=str,
        default="mpc_expriments/docs/阶跃扰动对比",
        help="输出前缀：*_xt.png / *_ex.png / *_vact.png 及 *_combined.png（三子图合一）",
    )
    parser.add_argument("--no-show", action="store_true", help="仅保存不弹窗")
    args = parser.parse_args(argv)

    if not (0.0 < float(args.eta_low) <= 1.0):
        raise ValueError("--eta-low 必须在 (0,1]")
    if float(args.dt) <= 0.0:
        raise ValueError("--dt 必须大于 0")
    if float(args.cmd_rate_limit) <= 0.0:
        raise ValueError("--cmd-rate-limit 必须大于 0")
    if float(args.cmd_tau) <= 0.0:
        raise ValueError("--cmd-tau 必须大于 0")
    if float(args.x_ref_speed) <= 0.0:
        raise ValueError("--x-ref-speed 必须大于 0")
    if float(args.disturb_end) <= float(args.disturb_start):
        raise ValueError("--disturb-end 必须大于 --disturb-start")

    run_step_disturbance_experiment(
        x_end=float(args.x_end),
        eta_low=float(args.eta_low),
        disturb_start=float(args.disturb_start),
        disturb_end=float(args.disturb_end),
        x_ref_speed=float(args.x_ref_speed),
        dt=float(args.dt),
        v_cmd_max=float(args.v_cmd_max),
        k_track=float(args.k_track),
        k_slip_ff=float(args.k_slip_ff),
        cmd_rate_limit=float(args.cmd_rate_limit),
        cmd_tau=float(args.cmd_tau),
        save_prefix=str(args.save_prefix),
        show=not bool(args.no_show),
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

