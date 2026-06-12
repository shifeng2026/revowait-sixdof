"""
铰链标注约定（与图纸一致，不做自动重排）

- 第 i 行坐标 = bi / pi（i=1..6）
- 电缸 i 连接 bi 与 pi
- Y 轴参考：p1–p2 连线的中垂线方向（用于示意图，不参与正解迭代）
"""

from __future__ import annotations

import numpy as np


def y_axis_from_p1_p2(
    p1: np.ndarray,
    p2: np.ndarray,
    z_up: np.ndarray | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    """
    由 p1、p2 求 Y 轴（中垂线方向）及其中点。

    Y 在水平面内，垂直于 p1→p2，取 Y = Z × X，X 为 p1→p2 水平投影单位向量。
    """
    p1 = np.asarray(p1, dtype=float)
    p2 = np.asarray(p2, dtype=float)
    Z = np.array([0.0, 0.0, 1.0]) if z_up is None else np.asarray(z_up, dtype=float)
    Z = Z / np.linalg.norm(Z)

    mid = 0.5 * (p1 + p2)
    X = p2 - p1
    X[2] = 0.0
    nx = np.linalg.norm(X)
    if nx < 1e-9:
        raise ValueError("p1 与 p2 在水平面几乎重合，无法定义 Y 轴")
    X = X / nx

    Y = np.cross(Z, X)
    Y = Y / np.linalg.norm(Y)
    return Y, mid
