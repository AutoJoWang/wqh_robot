"""带滑移前馈补偿的 MPC 接口。"""

from __future__ import annotations

from typing import Optional, Tuple

import numpy as np

from mpc_realize import Params, mpc_plan


def mpc_plan_with_slip_compensation(
    start: np.ndarray,
    goal: np.ndarray,
    *,
    path: np.ndarray,
    map_data: Optional[np.ndarray] = None,
    param: Optional[Params] = None,
    slip_linear: float = 1.0,
) -> Tuple[np.ndarray, np.ndarray, bool]:
    """
    运行带滑移补偿的 MPC。

    说明:
      - 基础模型: 实际速度 = slip_linear * v_cmd
      - 补偿方式: 基于“运动学残差后验反击”生成 delta_u，并叠加到 MPC 输出
    """
    return mpc_plan(
        start,
        goal,
        path=path,
        map_data=map_data,
        param=param,
        slip_linear=slip_linear,
        slip_compensate=True,
    )


def mpc_plan_without_slip_compensation(
    start: np.ndarray,
    goal: np.ndarray,
    *,
    path: np.ndarray,
    map_data: Optional[np.ndarray] = None,
    param: Optional[Params] = None,
    slip_linear: float = 1.0,
) -> Tuple[np.ndarray, np.ndarray, bool]:
    """运行不带滑移补偿的 MPC（仅考虑滑移衰减）。"""
    return mpc_plan(
        start,
        goal,
        path=path,
        map_data=map_data,
        param=param,
        slip_linear=slip_linear,
        slip_compensate=False,
    )

