"""
Python translation of `mpc_realize.m` (MPC motion planning / path tracking).

Original MATLAB entrypoint:
    [pose, traj, flag] = mpc_plan(start, goal, 'path', path, 'map', map)

This module keeps the same structure:
    - mpc_plan(start, goal, path=..., map_data=...)
    - kinematic model f()
    - lookahead point computation
    - MPC control via QP (quadprog -> OSQP / CVXPY)

QP solver dependency:
    - Preferred: `osqp` + `scipy` (fast)
    - Fallback: `cvxpy`

If neither is installed, mpcControl() will raise ImportError with instructions.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import math
import numpy as np
from typing import Sequence, Union


@dataclass
class RobotState:
    x: float
    y: float
    theta: float
    v: float = 0.0
    w: float = 0.0


@dataclass
class Params:
    # common parameters
    dt: float = 0.1
    max_iteration: int = 2000
    goal_dist_tol: float = 0.05
    rotate_tol: float = 0.5
    lookahead_time: float = 1.0
    min_lookahead_dist: float = 1.0
    max_lookahead_dist: float = 2.5
    max_v_inc: float = 0.5
    max_v: float = 1.0
    min_v: float = 0.0
    max_w_inc: float = math.pi / 2.0
    max_w: float = math.pi / 2.0
    min_w: float = 0.0

    # MPC parameters
    Q: np.ndarray = None  # (3x3)
    R: np.ndarray = None  # (2x2)
    p: int = 12
    m: int = 8
    u_min: np.ndarray = None  # (2,)
    u_max: np.ndarray = None  # (2,)
    du_min: np.ndarray = None  # (2,)
    du_max: np.ndarray = None  # (2,)

    def __post_init__(self) -> None:
        if self.Q is None:
            self.Q = np.diag([10.0, 10.0, 2.0])
        if self.R is None:
            self.R = np.diag([2.0, 2.0])
        if self.u_min is None:
            self.u_min = np.array([self.min_v, self.min_w], dtype=float)
        if self.u_max is None:
            self.u_max = np.array([self.max_v, self.max_w], dtype=float)
        if self.du_min is None:
            self.du_min = np.array([self.min_v, -self.max_w_inc], dtype=float)
        if self.du_max is None:
            self.du_max = np.array([self.max_v_inc, self.max_w_inc], dtype=float)


def mpc_plan(
    start: np.ndarray,
    goal: np.ndarray,
    *,
    path: np.ndarray,
    map_data: Optional[np.ndarray] = None,
    param: Optional[Params] = None,
) -> Tuple[np.ndarray, np.ndarray, bool]:
    """
    MPC motion planning / tracking along a provided path.

    Args:
        start: array-like (3,) [x, y, theta]
        goal:  array-like (3,) [x, y, theta]
        path:  array-like (N,2) path points (x,y)
        map_data: kept for signature parity with MATLAB; not used
        param: Params override

    Returns:
        pose: (T,3) stacked [x,y,theta] over iterations
        traj: placeholder (empty), kept for parity
        flag: True if terminated by reaching goal distance tolerance
    """
    if path is None or isinstance(path, str):
        raise ValueError("parameter `path` must be provided as an (N,2) array")
    if map_data is None:
        # MATLAB checks both path/map not strings, but doesn't actually use map.
        pass

    if param is None:
        param = Params()

    path = np.asarray(path, dtype=float)
    if path.ndim != 2 or path.shape[1] != 2:
        raise ValueError("`path` must have shape (N,2)")

    # MATLAB code flips the path before interpolation:
    #     path = flipud(path);
    # That assumes the incoming path may be ordered goal->start.
    # Here we make it robust: ensure the path is ordered start->goal.
    start_xy = np.array([float(start[0]), float(start[1])], dtype=float)
    d0 = float(np.hypot(path[0, 0] - start_xy[0], path[0, 1] - start_xy[1]))
    d1 = float(np.hypot(path[-1, 0] - start_xy[0], path[-1, 1] - start_xy[1]))
    if d1 < d0:
        path = np.flipud(path)
    path = path_interpolation(path, 5)

    robot = RobotState(float(start[0]), float(start[1]), float(start[2]), v=0.0, w=0.0)

    flag = False
    pose_hist: list[list[float]] = []
    traj = np.zeros((0, 0), dtype=float)

    u_p = np.array([0.0, 0.0], dtype=float)  # previous control error

    for _ in range(int(param.max_iteration)):
        # break until goal reached (position + heading)
        if goalReached(robot, goal, param):
            flag = True
            break

        lookahead_pt, theta_trj, kappa = getLookaheadPoint(robot, path, param)
        _ = kappa  # curvature returned but unused in MATLAB main loop

        e_theta_goal = regularizeAngle(robot.theta - float(goal[2])) / 10.0

        if shouldRotateToGoal((robot.x, robot.y), goal, param):
            if not shouldRotateToPath(abs(e_theta_goal), 0.0, param):
                u = np.array([0.0, 0.0], dtype=float)
            else:
                u = np.array([0.0, angularRegularization(robot, e_theta_goal / param.dt, param)], dtype=float)
        else:
            # heading error to the lookahead point
            e_theta = regularizeAngle(
                math.atan2(float(np.real(lookahead_pt[1])) - robot.y,
                           float(np.real(lookahead_pt[0])) - robot.x) - robot.theta
            ) / 10.0

            if shouldRotateToPath(abs(e_theta), math.pi / 4.0, param):
                u = np.array([0.0, angularRegularization(robot, e_theta / param.dt, param)], dtype=float)
            else:
                s = np.array([robot.x, robot.y, robot.theta], dtype=float)
                s_d = np.array([float(np.real(lookahead_pt[0])), float(np.real(lookahead_pt[1])), float(theta_trj)], dtype=float)
                u_r = np.array([robot.v, theta_trj - robot.theta], dtype=float)
                u, u_p = mpcControl(s, s_d, u_r, u_p, robot, param)

        robot = f(robot, u, param.dt)
        pose_hist.append([robot.x, robot.y, robot.theta])

    pose = np.asarray(pose_hist, dtype=float)
    return pose, traj, flag


def f(robot: RobotState, u: np.ndarray, dt: float) -> RobotState:
    """Robotic kinematic update (matches MATLAB f())."""
    v = float(u[0])
    w = float(u[1])

    # MATLAB uses an augmented state and linear map; this is equivalent.
    x_new = robot.x + dt * math.cos(robot.theta) * v
    y_new = robot.y + dt * math.sin(robot.theta) * v
    theta_new = robot.theta + dt * w

    return RobotState(x=x_new, y=y_new, theta=theta_new, v=v, w=w)


def regularizeAngle(angle: float) -> float:
    """Map angle to (-pi, pi]."""
    return angle - 2.0 * math.pi * math.floor((angle + math.pi) / (2.0 * math.pi))


def shouldRotateToGoal(cur_xy: Tuple[float, float], goal: np.ndarray, param: Params) -> bool:
    return math.hypot(cur_xy[0] - float(goal[0]), cur_xy[1] - float(goal[1])) < param.goal_dist_tol


def goalReached(robot: RobotState, goal: np.ndarray, param: Params) -> bool:
    """Stop condition: close in position and heading."""
    dist_ok = math.hypot(robot.x - float(goal[0]), robot.y - float(goal[1])) < param.goal_dist_tol
    yaw_ok = abs(regularizeAngle(robot.theta - float(goal[2]))) < param.rotate_tol
    return dist_ok and yaw_ok


def shouldRotateToPath(angle_to_path: float, tol: float, param: Params) -> bool:
    if tol == 0.0:
        return angle_to_path > param.rotate_tol
    return angle_to_path > tol


def angularRegularization(robot: RobotState, w_d: float, param: Params) -> float:
    w_inc = w_d - robot.w
    if abs(w_inc) > param.max_w_inc:
        w_inc = param.max_w_inc * (1.0 if w_inc >= 0.0 else -1.0)
    w = robot.w + w_inc

    if abs(w) > param.max_w:
        w = param.max_w * (1.0 if w >= 0.0 else -1.0)
    if abs(w) < param.min_w:
        w = param.min_w * (1.0 if w >= 0.0 else -1.0)
    return w


def linearRegularization(robot: RobotState, v_d: float, param: Params) -> float:
    v_inc = v_d - robot.v
    if abs(v_inc) > param.max_v_inc:
        v_inc = param.max_v_inc * (1.0 if v_inc >= 0.0 else -1.0)
    v = robot.v + v_inc

    if abs(v) > param.max_v:
        v = param.max_v * (1.0 if v >= 0.0 else -1.0)
    if abs(v) < param.min_v:
        v = param.min_v * (1.0 if v >= 0.0 else -1.0)
    return v


def getLookaheadDistance(robot: RobotState, param: Params) -> float:
    d = robot.v * param.lookahead_time
    d = max(d, param.min_lookahead_dist)
    d = min(d, param.max_lookahead_dist)
    return float(d)


def getLookaheadPoint(robot: RobotState, path: np.ndarray, param: Params) -> Tuple[np.ndarray, float, float]:
    """
    Find the point on the path exactly one lookahead distance away from robot.
    Returns (pt[x,y], theta_on_traj, curvature_kappa).
    """
    pts_num = path.shape[0]
    dist_to_robot = np.hypot(path[:, 0] - robot.x, path[:, 1] - robot.y)
    idx_closest = int(np.argmin(dist_to_robot))

    idx_goal = pts_num - 1
    idx_prev = idx_goal - 1

    lookahead_dist = getLookaheadDistance(robot, param)
    for i in range(idx_closest, pts_num):
        if math.hypot(path[i, 0] - robot.x, path[i, 1] - robot.y) >= lookahead_dist:
            idx_goal = i
            break

    if idx_goal == pts_num - 1:
        pt = np.array([path[idx_goal, 0], path[idx_goal, 1]], dtype=float)
    else:
        if idx_goal == 0:
            idx_goal = 1
        idx_prev = idx_goal - 1
        px, py = float(path[idx_prev, 0]), float(path[idx_prev, 1])
        gx, gy = float(path[idx_goal, 0]), float(path[idx_goal, 1])

        prev_p = np.array([px - robot.x, py - robot.y], dtype=float)
        goal_p = np.array([gx - robot.x, gy - robot.y], dtype=float)
        i_points = circleSegmentIntersection(prev_p, goal_p, lookahead_dist)
        if i_points.size == 0:
            # Fallback: use the goal point if no intersection found (degenerate).
            pt = np.array([gx, gy], dtype=float)
        else:
            pt = np.array([i_points[0, 0] + robot.x, i_points[0, 1] + robot.y], dtype=float)

    theta = math.atan2(path[idx_goal, 1] - path[idx_prev, 1], path[idx_goal, 0] - path[idx_prev, 0])

    # curvature kappa from 3 consecutive points
    if idx_goal == 1:
        idx_goal = 2
    if idx_goal == 2:
        idx_goal = 3 if pts_num > 3 else 2
    idx_prev = idx_goal - 1
    idx_pprev = idx_prev - 1

    a = math.hypot(path[idx_prev, 0] - path[idx_goal, 0], path[idx_prev, 1] - path[idx_goal, 1])
    b = math.hypot(path[idx_pprev, 0] - path[idx_goal, 0], path[idx_pprev, 1] - path[idx_goal, 1])
    c = math.hypot(path[idx_pprev, 0] - path[idx_prev, 0], path[idx_pprev, 1] - path[idx_prev, 1])

    # Avoid division by zero for degenerate segments
    if a <= 1e-9 or b <= 1e-9 or c <= 1e-9:
        kappa = 0.0
    else:
        cosB = (a * a + c * c - b * b) / (2.0 * a * c)
        cosB = max(-1.0, min(1.0, cosB))
        sinB = math.sin(math.acos(cosB))
        cross = (
            (path[idx_prev, 0] - path[idx_pprev, 0]) * (path[idx_goal, 1] - path[idx_pprev, 1])
            - (path[idx_prev, 1] - path[idx_pprev, 1]) * (path[idx_goal, 0] - path[idx_pprev, 0])
        )
        kappa = 2.0 * sinB / b * (1.0 if cross >= 0.0 else -1.0)

    return pt, float(theta), float(kappa)


def circleSegmentIntersection(p1: np.ndarray, p2: np.ndarray, r: float) -> np.ndarray:
    x1, y1 = float(p1[0]), float(p1[1])
    x2, y2 = float(p2[0]), float(p2[1])

    dx, dy = x2 - x1, y2 - y1
    dr2 = dx * dx + dy * dy
    D = x1 * y2 - x2 * y1

    d1 = x1 * x1 + y1 * y1
    d2 = x2 * x2 + y2 * y2
    dd = d2 - d1

    disc = r * r * dr2 - D * D
    if disc < 0.0:
        return np.zeros((0, 2), dtype=float)

    delta = math.sqrt(max(0.0, disc))
    if delta == 0.0:
        return np.array([[D * dy / dr2, -D * dx / dr2]], dtype=float)

    s = 1.0 if dd >= 0.0 else -1.0
    p_a = np.array([(D * dy + s * dx * delta) / dr2, (-D * dx + s * dy * delta) / dr2], dtype=float)
    p_b = np.array([(D * dy - s * dx * delta) / dr2, (-D * dx - s * dy * delta) / dr2], dtype=float)
    return np.vstack([p_a, p_b])


def path_interpolation(path: np.ndarray, n: int) -> np.ndarray:
    path_new = np.asarray(path, dtype=float)
    for _ in range(int(n)):
        path_inter = path_new[:-1, :] + np.diff(path_new, axis=0) / 2.0
        out = np.zeros((path_new.shape[0] + path_inter.shape[0], 2), dtype=float)
        out[0::2, :] = path_new
        out[1::2, :] = path_inter
        path_new = out
    return path_new


def mpcControl(
    s: np.ndarray,
    s_d: np.ndarray,
    u_r: np.ndarray,
    u_p: np.ndarray,
    robot: RobotState,
    param: Params,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Execute MPC control process (QP).

    This matches the MATLAB `mpcControl` function.
    """
    dim_u = 2
    dim_x = 3
    dt = float(param.dt)

    # state vector (5x1): [s - s_d, u_p]^T
    x = np.concatenate([s - s_d, u_p]).reshape((-1, 1))  # (5,1)

    # original state matrix A (3x3)
    A = np.eye(3)
    A[0, 2] = -u_r[0] * math.sin(s_d[2]) * dt
    A[1, 2] = u_r[0] * math.cos(s_d[2]) * dt

    # original control matrix B (3x2)
    B = np.zeros((3, 2))
    B[0, 0] = math.cos(s_d[2]) * dt
    B[1, 0] = math.sin(s_d[2]) * dt
    B[2, 1] = dt

    # augmented (5x5) A, (5x2) B
    A_aug = np.block([
        [A, B],
        [np.zeros((dim_u, dim_x)), np.eye(dim_u)],
    ])
    B_aug = np.vstack([B, np.eye(dim_u)])

    C = np.hstack([np.eye(dim_x), np.zeros((dim_x, dim_u))])  # (3x5)

    # S_x: (3p x 5)
    S_x_blocks = []
    A_pow = np.eye(A_aug.shape[0])
    for i in range(1, param.p + 1):
        A_pow = A_pow @ A_aug
        S_x_blocks.append(C @ A_pow)
    S_x = np.vstack(S_x_blocks)

    # S_u: (3p x 2m)
    S_u = np.zeros((dim_x * param.p, dim_u * param.m))
    # Precompute powers of A_aug
    A_pows = [np.eye(A_aug.shape[0])]
    for _ in range(1, param.p + 1):
        A_pows.append(A_pows[-1] @ A_aug)
    for i in range(1, param.p + 1):
        for j in range(1, param.m + 1):
            if j <= i:
                block = C @ (A_pows[i - j] @ B_aug)  # (3x2)
                row0 = dim_x * (i - 1)
                col0 = dim_u * (j - 1)
                S_u[row0:row0 + dim_x, col0:col0 + dim_u] = block

    Yr = np.zeros((dim_x * param.p, 1))
    Qb = np.kron(np.eye(param.p), param.Q)  # (3p x 3p)
    Rb = np.kron(np.eye(param.m), param.R)  # (2m x 2m)

    H = S_u.T @ Qb @ S_u + Rb
    g = S_u.T @ Qb @ (S_x @ x - Yr)  # (2m,1)

    # Constraints
    A_I = np.kron(np.tril(np.ones((param.m, param.m))), np.diag([1.0, 1.0]))  # (2m x 2m)
    U_min = np.kron(np.ones((param.m, 1)), param.u_min.reshape((2, 1)))  # (2m,1)
    U_max = np.kron(np.ones((param.m, 1)), param.u_max.reshape((2, 1)))  # (2m,1)
    U_k_1 = np.kron(np.ones((param.m, 1)), u_p.reshape((2, 1)))  # (2m,1)

    dU_min = np.kron(np.ones((param.m, 1)), param.du_min.reshape((2, 1)))  # (2m,1)
    dU_max = np.kron(np.ones((param.m, 1)), param.du_max.reshape((2, 1)))  # (2m,1)

    # Inequality: [-A_I; A_I] dU <= [-U_min + U_k_1; U_max - U_k_1]
    A_ineq = np.vstack([-A_I, A_I])
    b_ineq = np.vstack([-U_min + U_k_1, U_max - U_k_1]).reshape((-1,))

    dU_opt = solve_qp(H, g.reshape((-1,)), A_ineq, b_ineq, dU_min.reshape((-1,)), dU_max.reshape((-1,)))

    du = np.array([dU_opt[0], dU_opt[1]], dtype=float)
    u = du + u_p + u_r

    u = np.array(
        [linearRegularization(robot, float(u[0]), param),
         angularRegularization(robot, float(u[1]), param)],
        dtype=float,
    )
    u_p_new = u - u_r
    return u, u_p_new


def solve_qp(
    H: np.ndarray,
    f: np.ndarray,
    A: np.ndarray,
    b: np.ndarray,
    lb: np.ndarray,
    ub: np.ndarray,
) -> np.ndarray:
    """
    Solve:
        min 1/2 x^T H x + f^T x
        s.t. A x <= b
             lb <= x <= ub
    """
    # Try OSQP first (fast, robust)
    try:
        import scipy.sparse as sp  # type: ignore
        import osqp  # type: ignore

        P = sp.csc_matrix((H + H.T) * 0.5)
        q = f
        A_osqp = sp.csc_matrix(A)
        # OSQP uses l <= A x <= u. For A x <= b, set l=-inf.
        l = -np.inf * np.ones_like(b, dtype=float)
        u = b.astype(float)

        # Add bound constraints by augmenting A with identity
        I = sp.eye(H.shape[0], format="csc")
        A2 = sp.vstack([A_osqp, I], format="csc")
        l2 = np.concatenate([l, lb.astype(float)])
        u2 = np.concatenate([u, ub.astype(float)])

        prob = osqp.OSQP()
        prob.setup(P=P, q=q, A=A2, l=l2, u=u2, verbose=False, max_iter=100)
        res = prob.solve()
        if res.info.status_val not in (1, 2):  # solved / solved inaccurate
            raise RuntimeError(f"OSQP failed: {res.info.status}")
        return np.asarray(res.x, dtype=float)
    except ImportError:
        pass

    # Fallback: CVXPY
    try:
        import cvxpy as cp  # type: ignore

        x = cp.Variable(H.shape[0])
        obj = 0.5 * cp.quad_form(x, (H + H.T) * 0.5) + f @ x
        cons = [A @ x <= b, x >= lb, x <= ub]
        prob = cp.Problem(cp.Minimize(obj), cons)
        prob.solve(solver=cp.OSQP, verbose=False, max_iter=100)
        if x.value is None:
            raise RuntimeError("CVXPY failed to solve QP (no solution).")
        return np.asarray(x.value, dtype=float).reshape((-1,))
    except ImportError as e:
        raise ImportError(
            "QP solver not available. Install one of:\n"
            "  - osqp + scipy  (recommended)\n"
            "  - cvxpy\n"
        ) from e


def plot_mpc_result(
    *,
    path: np.ndarray,
    pose: np.ndarray,
    start: Union[np.ndarray, Sequence[float]],
    goal: Union[np.ndarray, Sequence[float]],
    title: str = "MPC path tracking",
    show: bool = True,
    save: Optional[str] = None,
):
    """
    Visualize the input path and the simulated pose trajectory.

    Args:
        path: (N,2)
        pose: (T,3) returned by mpc_plan
        start: (3,) [x,y,theta]
        goal: (3,) [x,y,theta]
        title: plot title
        show: call plt.show()
        save: optional filepath to save figure (e.g. "mpc.png")
    """
    try:
        import matplotlib.pyplot as plt  # type: ignore
    except ImportError as e:
        raise ImportError(
            "matplotlib not installed. Install it in your environment, e.g.\n"
            "  conda install -c conda-forge matplotlib\n"
            "or\n"
            "  pip install matplotlib\n"
        ) from e

    path = np.asarray(path, dtype=float)
    pose = np.asarray(pose, dtype=float)
    start = np.asarray(start, dtype=float).reshape((-1,))
    goal = np.asarray(goal, dtype=float).reshape((-1,))

    fig, ax = plt.subplots(figsize=(7, 6))
    ax.plot(path[:, 0], path[:, 1], "g-", lw=2, label="path")

    if pose.size:
        ax.plot(pose[:, 0], pose[:, 1], "r-", lw=2, label="mpc pose")
        ax.plot(pose[-1, 0], pose[-1, 1], "ro", ms=6, label="final")

    ax.plot(start[0], start[1], "bo", ms=7, label="start")
    ax.plot(goal[0], goal[1], "ko", ms=7, label="goal")

    def _arrow(x, y, th, color, label):
        ax.arrow(
            x, y,
            0.25 * math.cos(th),
            0.25 * math.sin(th),
            head_width=0.08,
            head_length=0.10,
            length_includes_head=True,
            color=color,
            label=label,
        )

    if start.size >= 3:
        _arrow(float(start[0]), float(start[1]), float(start[2]), "b", "start heading")
    if goal.size >= 3:
        _arrow(float(goal[0]), float(goal[1]), float(goal[2]), "k", "goal heading")

    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title(title)
    ax.legend(loc="best")

    if save:
        fig.savefig(save, dpi=150, bbox_inches="tight")
    if show:
        plt.show()
    return fig, ax

