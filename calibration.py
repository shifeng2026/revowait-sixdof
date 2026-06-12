"""
界面电缸与内部计算映射；位姿显示标定。

RY/RZ 按「主驱动」内部电缸取符号（行程绝对值最大者）：
  显示值 = |计算值| × 目标符号（+1 正 / -1 负）
"""

from __future__ import annotations

import numpy as np

UI_TO_INTERNAL = np.array([3, 4, 5, 0, 1, 2], dtype=int)
INTERNAL_TO_UI = np.array([3, 4, 5, 0, 1, 2], dtype=int)

# X, Y, Z, RX, RY, RZ（RY/RZ 在 apply_ry_rz_display 中再处理）
DEFAULT_POSE_DISPLAY_SIGNS = np.array([-1.0, -1.0, 1.0, -1.0, 1.0, 1.0])

# 各缸 |ΔL| 均小于此值 (mm) 时，RY/RZ 显示规则用默认内部缸 1
STROKE_ACTIVE_EPS = 1e-3

# 内部电缸下标 0..5 对应 内部1..6 的 RY、RZ 显示目标符号
# 界面1→内4(3): RY+ RZ+  界面2→内5(4): RY- RZ-
# 界面3→内6(5): RY- RZ+  界面4→内1(0): RY+ RZ-
# 界面5→内2(1): RY- RZ+  界面6→内3(2): RY+ RZ-
RY_DISPLAY_BY_INTERNAL = np.array([1.0, -1.0, 1.0, 1.0, -1.0, -1.0])
RZ_DISPLAY_BY_INTERNAL = np.array([-1.0, 1.0, -1.0, 1.0, -1.0, 1.0])

def internal_to_ui_index(internal_i: int) -> int:
    """内部下标 0..5 → 界面电缸号 1..6 的下标 0..5。"""
    return int(INTERNAL_TO_UI[int(internal_i)])


def format_stroke_limit_message(
    strokes_internal: np.ndarray,
    stroke_min: np.ndarray | None,
    stroke_max: np.ndarray | None,
) -> str | None:
    """超出配置行程限时返回说明（含界面电缸编号）。"""
    s = np.asarray(strokes_internal, dtype=float)
    if stroke_min is not None:
        bad = np.where(s < stroke_min - 1e-9)[0]
        if len(bad):
            i = int(bad[0])
            ui = internal_to_ui_index(i) + 1
            return (
                f"界面电缸 {ui}（内部 {i + 1}）ΔL={s[i]:.2f} mm，"
                f"低于配置下限 {stroke_min[i]:.1f} mm"
            )
    if stroke_max is not None:
        bad = np.where(s > stroke_max + 1e-9)[0]
        if len(bad):
            i = int(bad[0])
            ui = internal_to_ui_index(i) + 1
            return (
                f"界面电缸 {ui}（内部 {i + 1}）ΔL={s[i]:.2f} mm，"
                f"超过配置上限 {stroke_max[i]:.1f} mm"
            )
    return None


def ui_strokes_to_internal(ui_strokes: np.ndarray) -> np.ndarray:
    ui = np.asarray(ui_strokes, dtype=float)
    internal = np.zeros(6, dtype=float)
    for ui_i in range(6):
        internal[UI_TO_INTERNAL[ui_i]] = ui[ui_i]
    return internal


def internal_array_to_ui(internal: np.ndarray) -> np.ndarray:
    internal = np.asarray(internal, dtype=float)
    ui = np.zeros(6, dtype=float)
    for ui_i in range(6):
        ui[ui_i] = internal[UI_TO_INTERNAL[ui_i]]
    return ui


def dominant_internal_index(internal_strokes: np.ndarray | None) -> int:
    """行程绝对值最大的内部电缸下标；无行程时返回 0。"""
    if internal_strokes is None:
        return 0
    s = np.asarray(internal_strokes, dtype=float)
    if np.max(np.abs(s)) < STROKE_ACTIVE_EPS:
        return 0
    return int(np.argmax(np.abs(s)))


def apply_ry_rz_display(
    display_pose: np.ndarray,
    raw_pose: np.ndarray,
    internal_strokes: np.ndarray | None,
) -> np.ndarray:
    """在已处理 X,Y,Z,RX 的显示位姿上，按主驱动内部缸设置 RY、RZ 正负。"""
    out = np.asarray(display_pose, dtype=float).copy()
    raw = np.asarray(raw_pose, dtype=float)
    idx = dominant_internal_index(internal_strokes)
    out[4] = abs(raw[4]) * RY_DISPLAY_BY_INTERNAL[idx]
    out[5] = abs(raw[5]) * RZ_DISPLAY_BY_INTERNAL[idx]
    return out


def to_display_pose(
    pose: np.ndarray,
    platform_initial_height: float,
    signs: np.ndarray | None = None,
    internal_strokes: np.ndarray | None = None,
) -> np.ndarray:
    p = np.asarray(pose, dtype=float).copy()
    s = DEFAULT_POSE_DISPLAY_SIGNS if signs is None else np.asarray(signs, dtype=float)
    out = p * s
    out[2] = p[2] - float(platform_initial_height)
    return apply_ry_rz_display(out, p, internal_strokes)


def to_display_delta(
    delta: np.ndarray,
    signs: np.ndarray | None = None,
    internal_strokes: np.ndarray | None = None,
    raw_delta: np.ndarray | None = None,
) -> np.ndarray:
    d = np.asarray(delta, dtype=float).copy()
    raw = d if raw_delta is None else np.asarray(raw_delta, dtype=float)
    s = DEFAULT_POSE_DISPLAY_SIGNS if signs is None else np.asarray(signs, dtype=float)
    out = d * s
    return apply_ry_rz_display(out, raw, internal_strokes)


def ry_rz_rule_lines() -> list[str]:
    lines = ["RY/RZ 显示规则（|计算值|×目标符号，主驱动=行程最大内部缸）:"]
    for ui_i in range(6):
        ii = int(UI_TO_INTERNAL[ui_i])
        ry = "正" if RY_DISPLAY_BY_INTERNAL[ii] > 0 else "负"
        rz = "正" if RZ_DISPLAY_BY_INTERNAL[ii] > 0 else "负"
        lines.append(
            f"  界面{ui_i + 1}→内{ii + 1}: RY={ry}, RZ={rz}"
        )
    return lines


def mapping_description() -> str:
    lines = ["界面电缸 -> 内部计算电缸:"]
    for ui_i in range(6):
        internal_i = UI_TO_INTERNAL[ui_i]
        lines.append(f"  界面 {ui_i + 1}  ->  内部 {internal_i + 1}")
    lines.extend(ry_rz_rule_lines())
    return "\n".join(lines)
