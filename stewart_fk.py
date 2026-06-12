"""
六自由度运动平台 (Stewart Platform) 正解计算

约束（与图纸一致）：
  - 第 i 行下铰链 bi、上铰链 pi，按标注顺序 1~6 依次填写
  - 电缸 i 连接 bi — pi
  - 下铰链：世界坐标；上铰链：动平台体坐标
  - 位姿 ZYX 欧拉角；正解 Newton-Raphson
  - Y 轴定义：p1–p2 连线中垂线（见 hinge_layout，仅作示意）
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Sequence, Tuple, Union

import numpy as np

from calibration import DEFAULT_POSE_DISPLAY_SIGNS, to_display_delta, to_display_pose
from hinge_layout import y_axis_from_p1_p2

try:
    import matplotlib.pyplot as plt

    _HAS_MPL = True
except ImportError:
    _HAS_MPL = False


def rotation_matrix_z(rz: float) -> np.ndarray:
    c, s = np.cos(rz), np.sin(rz)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def rotation_matrix_y(ry: float) -> np.ndarray:
    c, s = np.cos(ry), np.sin(ry)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rotation_matrix_x(rx: float) -> np.ndarray:
    c, s = np.cos(rx), np.sin(rx)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def euler_to_rotation(rx: float, ry: float, rz: float) -> np.ndarray:
    """ZYX: R = Rz @ Ry @ Rx"""
    return rotation_matrix_z(rz) @ rotation_matrix_y(ry) @ rotation_matrix_x(rx)


def dr_drx(rx: float, ry: float, rz: float) -> np.ndarray:
    Rz, Ry = rotation_matrix_z(rz), rotation_matrix_y(ry)
    dRx = np.array(
        [[0, 0, 0], [0, -np.sin(rx), -np.cos(rx)], [0, np.cos(rx), -np.sin(rx)]]
    )
    return Rz @ Ry @ dRx


def dr_dry(rx: float, ry: float, rz: float) -> np.ndarray:
    Rz = rotation_matrix_z(rz)
    dRy = np.array(
        [[-np.sin(ry), 0, np.cos(ry)], [0, 0, 0], [-np.cos(ry), 0, -np.sin(ry)]]
    )
    return Rz @ dRy @ rotation_matrix_x(rx)


def dr_drz(rx: float, ry: float, rz: float) -> np.ndarray:
    dRz = np.array(
        [[-np.sin(rz), -np.cos(rz), 0], [np.cos(rz), -np.sin(rz), 0], [0, 0, 0]]
    )
    return dRz @ rotation_matrix_y(ry) @ rotation_matrix_x(rx)


@dataclass
class PlatformConfig:
    """
    base_points[i] = bi 世界坐标
    platform_points[i] = pi 体坐标（动平台参考点原点，零位姿态与世界轴对齐）
    """

    base_points: np.ndarray
    platform_points: np.ndarray
    initial_lengths: Optional[np.ndarray] = None
    stroke_min: Optional[np.ndarray] = None
    stroke_max: Optional[np.ndarray] = None
    home_height: Optional[float] = None
    home_pose: Optional[np.ndarray] = None
    pose_display_signs: Optional[np.ndarray] = None

    def __post_init__(self) -> None:
        self.B = np.asarray(self.base_points, dtype=float)
        self.P = np.asarray(self.platform_points, dtype=float)
        if self.B.shape != (6, 3) or self.P.shape != (6, 3):
            raise ValueError("铰链坐标必须为 6×3，按 b1~b6 / p1~p6 顺序填写")

        if self.initial_lengths is None:
            if self.home_height is None:
                raise ValueError("请提供 initial_lengths 或 home_height")
            self.L0 = compute_initial_lengths(self.B, self.P, self.home_height)
        else:
            self.L0 = np.asarray(self.initial_lengths, dtype=float)

        if self.stroke_min is not None:
            self.stroke_min = np.asarray(self.stroke_min, dtype=float)
        if self.stroke_max is not None:
            self.stroke_max = np.asarray(self.stroke_max, dtype=float)

        if self.home_pose is None:
            cx, cy = self.B[:, 0].mean(), self.B[:, 1].mean()
            cz = self.B[:, 2].mean() + (self.home_height or 0.0)
            self.home_pose = np.array([cx, cy, cz, 0.0, 0.0, 0.0])
        else:
            self.home_pose = np.asarray(self.home_pose, dtype=float)

        if self.pose_display_signs is None:
            self.pose_display_signs = DEFAULT_POSE_DISPLAY_SIGNS.copy()
        else:
            self.pose_display_signs = np.asarray(self.pose_display_signs, dtype=float)

    def to_dict(self) -> dict:
        return {
            "base_points": self.B.tolist(),
            "platform_points": self.P.tolist(),
            "home_height": self.home_height,
            "stroke_min": (
                self.stroke_min.tolist() if self.stroke_min is not None else None
            ),
            "stroke_max": (
                self.stroke_max.tolist() if self.stroke_max is not None else None
            ),
            "home_pose": self.home_pose.tolist(),
            "initial_lengths": self.L0.tolist(),
            "pose_display_signs": self.pose_display_signs.tolist(),
        }

    def save_json(self, path: Union[str, Path]) -> None:
        with open(path, "w", encoding="utf-8") as f:
            json.dump(self.to_dict(), f, ensure_ascii=False, indent=2)

    @classmethod
    def from_json(cls, path: Union[str, Path]) -> "PlatformConfig":
        with open(path, "r", encoding="utf-8") as f:
            d = json.load(f)
        il = d.get("initial_lengths")
        hp = d.get("home_pose")
        return cls(
            base_points=np.array(d["base_points"]),
            platform_points=np.array(d["platform_points"]),
            initial_lengths=np.array(il) if il is not None else None,
            stroke_min=np.array(d["stroke_min"]) if d.get("stroke_min") else None,
            stroke_max=np.array(d["stroke_max"]) if d.get("stroke_max") else None,
            home_height=d.get("home_height"),
            home_pose=np.array(hp) if hp is not None else None,
            pose_display_signs=(
                np.array(d["pose_display_signs"])
                if d.get("pose_display_signs")
                else None
            ),
        )


def compute_initial_lengths(
    base_points: np.ndarray,
    platform_points: np.ndarray,
    home_height: float,
    home_pose: Optional[np.ndarray] = None,
) -> np.ndarray:
    B = np.asarray(base_points, dtype=float)
    P = np.asarray(platform_points, dtype=float)
    if home_pose is None:
        cx, cy = B[:, 0].mean(), B[:, 1].mean()
        cz = B[:, 2].mean() + home_height
        home_pose = np.array([cx, cy, cz, 0.0, 0.0, 0.0])
    else:
        home_pose = np.asarray(home_pose, dtype=float)

    x, y, z, rx, ry, rz = home_pose
    R = euler_to_rotation(rx, ry, rz)
    T = np.array([x, y, z])
    return np.array([np.linalg.norm(R @ P[i] + T - B[i]) for i in range(6)])


class StewartPlatform:
    def __init__(
        self,
        base_points: np.ndarray,
        platform_points: np.ndarray,
        initial_lengths: np.ndarray,
        home_pose: Optional[np.ndarray] = None,
        stroke_min: Optional[np.ndarray] = None,
        stroke_max: Optional[np.ndarray] = None,
        home_height: Optional[float] = None,
    ):
        cfg = PlatformConfig(
            base_points=base_points,
            platform_points=platform_points,
            initial_lengths=initial_lengths,
            stroke_min=stroke_min,
            stroke_max=stroke_max,
            home_pose=home_pose,
            home_height=home_height,
        )
        self.B = cfg.B
        self.P = cfg.P
        self.L0 = cfg.L0
        self.home_pose = cfg.home_pose
        self.home_height = cfg.home_height
        self.stroke_min = cfg.stroke_min
        self.stroke_max = cfg.stroke_max
        self.pose_display_signs = cfg.pose_display_signs
        self._last_pose = cfg.home_pose.copy()

    def display_pose(
        self,
        pose: np.ndarray,
        internal_strokes: np.ndarray | None = None,
    ) -> np.ndarray:
        """内部位姿 -> 界面显示。"""
        h = self.home_height if self.home_height is not None else 0.0
        return to_display_pose(
            pose, h, self.pose_display_signs, internal_strokes
        )

    def display_delta(
        self,
        delta: np.ndarray,
        internal_strokes: np.ndarray | None = None,
        raw_pose: np.ndarray | None = None,
        raw_home_pose: np.ndarray | None = None,
    ) -> np.ndarray:
        raw_delta = None
        if raw_pose is not None and raw_home_pose is not None:
            raw_delta = np.asarray(raw_pose, dtype=float) - np.asarray(
                raw_home_pose, dtype=float
            )
        return to_display_delta(
            delta,
            self.pose_display_signs,
            internal_strokes,
            raw_delta=raw_delta,
        )

    @classmethod
    def from_config(cls, config: PlatformConfig) -> "StewartPlatform":
        plat = cls(
            config.B,
            config.P,
            config.L0,
            home_pose=config.home_pose,
            stroke_min=config.stroke_min,
            stroke_max=config.stroke_max,
            home_height=config.home_height,
        )
        plat.pose_display_signs = config.pose_display_signs
        return plat

    def check_strokes(self, strokes: Sequence[float]) -> Tuple[bool, str]:
        """检查行程是否在配置范围内（不阻止回放/UDP 时可仅作提示）。"""
        s = np.asarray(strokes, dtype=float)
        if (self.L0 + s <= 0).any():
            i = int(np.argmin(self.L0 + s))
            return False, f"内部电缸 {i + 1} 杆长非正，不可达"
        from calibration import format_stroke_limit_message

        msg = format_stroke_limit_message(s, self.stroke_min, self.stroke_max)
        if msg:
            return False, msg
        return True, "ok"

    def validate_strokes(self, strokes: Sequence[float]) -> Tuple[bool, str]:
        return self.check_strokes(strokes)

    def forward_kinematics(
        self,
        strokes: Sequence[float],
        guess: Optional[np.ndarray] = None,
        tol: float = 1e-10,
        max_iter: int = 200,
        verbose: bool = False,
        *,
        enforce_stroke_limits: bool = True,
    ) -> Tuple[np.ndarray, bool, int, float]:
        strokes = np.asarray(strokes, dtype=float)
        ok, msg = self.check_strokes(strokes)
        if not ok and enforce_stroke_limits:
            raise ValueError(msg)

        L = self.L0 + strokes
        guess = self._last_pose.copy() if guess is None else np.asarray(guess, dtype=float)
        pose, success, n_iter, residual = self._newton_raphson(
            guess, L, tol, max_iter, verbose
        )
        if success:
            self._last_pose = pose.copy()
        return pose, success, n_iter, residual

    def pose_delta_from_strokes(
        self, strokes: Sequence[float], **kwargs
    ) -> Tuple[np.ndarray, bool, int, float]:
        pose, ok, n, res = self.forward_kinematics(strokes, **kwargs)
        return pose - self.home_pose, ok, n, res

    def _newton_raphson(
        self, guess: np.ndarray, L: np.ndarray, tol: float, max_iter: int, verbose: bool
    ) -> Tuple[np.ndarray, bool, int, float]:
        vars = guess.copy()
        for it in range(max_iter):
            x, y, z, rx, ry, rz = vars
            R = euler_to_rotation(rx, ry, rz)
            T = np.array([x, y, z])
            dR_drx_ = dr_drx(rx, ry, rz)
            dR_dry_ = dr_dry(rx, ry, rz)
            dR_drz_ = dr_drz(rx, ry, rz)
            F = np.zeros(6)
            J = np.zeros((6, 6))
            for i in range(6):
                diff = R @ self.P[i] + T - self.B[i]
                F[i] = np.dot(diff, diff) - L[i] ** 2
                J[i, :] = [
                    2.0 * diff[0],
                    2.0 * diff[1],
                    2.0 * diff[2],
                    2.0 * np.dot(diff, dR_drx_ @ self.P[i]),
                    2.0 * np.dot(diff, dR_dry_ @ self.P[i]),
                    2.0 * np.dot(diff, dR_drz_ @ self.P[i]),
                ]
            residual = float(np.dot(F, F))
            if verbose:
                print(f"  iter {it}: residual={residual:.3e}")
            if residual < tol:
                return vars, True, it, residual
            try:
                delta = np.linalg.solve(J, -F)
            except np.linalg.LinAlgError:
                delta = np.linalg.lstsq(J, -F, rcond=None)[0]
            alpha = 1.0
            for _ in range(20):
                vn = vars + alpha * delta
                R2 = euler_to_rotation(vn[3], vn[4], vn[5])
                T2 = vn[:3]
                F2 = np.array(
                    [
                        np.dot(R2 @ self.P[i] + T2 - self.B[i], R2 @ self.P[i] + T2 - self.B[i])
                        - L[i] ** 2
                        for i in range(6)
                    ]
                )
                if np.dot(F2, F2) < residual:
                    break
                alpha *= 0.5
            vars = vars + alpha * delta

        R = euler_to_rotation(vars[3], vars[4], vars[5])
        T = vars[:3]
        F = np.array(
            [
                np.dot(R @ self.P[i] + T - self.B[i], R @ self.P[i] + T - self.B[i])
                - L[i] ** 2
                for i in range(6)
            ]
        )
        return vars, False, max_iter, float(np.dot(F, F))

    def inverse_kinematics(self, pose: Sequence[float]) -> np.ndarray:
        x, y, z, rx, ry, rz = pose
        R = euler_to_rotation(rx, ry, rz)
        T = np.array([x, y, z])
        return np.array(
            [np.linalg.norm(R @ self.P[i] + T - self.B[i]) for i in range(6)]
        )

    def reset_guess(self, pose: Optional[np.ndarray] = None) -> None:
        self._last_pose = (
            self.home_pose.copy()
            if pose is None
            else np.asarray(pose, dtype=float)
        )

    def plot_platform(self, pose: np.ndarray) -> None:
        if not _HAS_MPL:
            raise ImportError("请安装 matplotlib")
        x, y, z, rx, ry, rz = pose
        R = euler_to_rotation(rx, ry, rz)
        T = np.array([x, y, z])
        Pg = np.array([R @ p + T for p in self.P])

        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection="3d")
        ax.scatter(self.B[:, 0], self.B[:, 1], self.B[:, 2], c="b", s=70)
        ax.scatter(Pg[:, 0], Pg[:, 1], Pg[:, 2], c="r", s=70)
        for i in range(6):
            ax.plot(
                [self.B[i, 0], Pg[i, 0]],
                [self.B[i, 1], Pg[i, 1]],
                [self.B[i, 2], Pg[i, 2]],
                "k--",
                alpha=0.4,
            )
            ax.text(*self.B[i], f" b{i + 1}", color="blue", fontsize=9)
            ax.text(*Pg[i], f" p{i + 1}", color="red", fontsize=9)

        try:
            Y, mid = y_axis_from_p1_p2(Pg[0], Pg[1])
            L = 200.0
            ax.quiver(
                mid[0], mid[1], mid[2],
                Y[0] * L, Y[1] * L, Y[2] * L,
                color="orange",
                arrow_length_ratio=0.15,
            )
            ax.text(mid[0] + Y[0] * L, mid[1] + Y[1] * L, mid[2] + Y[2] * L, "Y")
        except ValueError:
            pass

        ax.set_title(
            f"x={x:.1f} y={y:.1f} z={z:.1f} mm | "
            f"rx={np.degrees(rx):.2f}° ry={np.degrees(ry):.2f}° rz={np.degrees(rz):.2f}°"
        )
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        ax.legend(["下铰链 bi", "上铰链 pi"])
        plt.tight_layout()
        plt.show()
