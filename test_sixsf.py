"""
六轴平台轻量化控制接口 (Lightweight Six-Axis Platform Control API)

Features:
  1. UDP 连接/断开
  2. 设置目标位姿（相对中位增量，通过 0x20 位姿随动一次性发送）
  3. 实时读取目标位姿
  4. 电缸伸长量 -> 位姿正解
  5. 加载平台默认配置
  6. 移动到中位
  7. 回原点

Usage:
    from test_sixsf import SixAxisPlatform

    plat = SixAxisPlatform("platform_config.json")
    plat.connect()

    pose = plat.read_current_pose()
    plat.set_pose([0, 0, 100, 0, 0, 0])       # Z 抬高 100mm
    plat.set_pose([50, 0, 100, 5, 0, 0])       # X+50mm, rx+5°

    result = plat.compute_pose_from_strokes([10, -5, 3, 7, -2, 0])

    plat.move_to_mid()
    plat.move_to_home()
    plat.disconnect()
"""

from __future__ import annotations

import json
import os
import socket
import sys
import threading
import time
from collections import deque
from typing import Any, Optional

import numpy as np

try:
    import matplotlib.pyplot as plt
    _HAS_MPL = True
except ImportError:
    _HAS_MPL = False

_project_dir = os.path.dirname(os.path.abspath(__file__))
if _project_dir not in sys.path:
    sys.path.insert(0, _project_dir)

from calibration import UI_TO_INTERNAL
from platform_client import PlatformClient, PlatformTarget
from protocol_control import (
    build_get_info,
    build_plat_mid,
    build_plat_reset,
    build_plat_stop,
    build_pose_follow_xyz_abc,
)
from protocol_udp import (
    CylinderFormula,
    CyclicFeedback,
    absolute_lengths_from_strokes,
    parse_udp_packet,
    pulses_to_strokes_internal,
)
from stewart_fk import PlatformConfig, StewartPlatform
from udp_service import UdpBindError, bind_udp_listen_socket

# 控制器中位绝对位姿（mm / rad）
# 下位机以该位姿为相对零点的参考基准，与 kinematic home_pose 不同。
# home_pose = 运动学零位（行程=0），中位 = 控制器相对坐标的原点。
MID_POSE_ABS = np.array([0, 0, 920, 0, 0, 0], dtype=float)
DEFAULT_POSE_Z_OFFSET_MM = 10.0

# FK 坐标 → 控制器坐标：X/Y/RX/RY 取反
_FK_TO_CONTROLLER_SIGN = np.array([-1.0, -1.0, 1.0, -1.0, -1.0, 1.0])

# 实时曲线默认滚动窗口（秒）
_PLOT_WINDOW_SEC = 10.0
_PLOT_MAX_POINTS = 300
_SCURVE_DT_SEC = 0.02
_TARGET_POS_TOL_MM = 0.5
_TARGET_ATT_TOL_DEG = 0.05
_TARGET_STABLE_SAMPLES = 5


class SixAxisPlatform:
    """六轴平台控制接口"""

    def __init__(self, config_path: str = "platform_config.json"):
        self._config_path: str = config_path
        self._platform: StewartPlatform | None = None
        self._client: PlatformClient | None = None
        self._stop_event = threading.Event()
        self._listen_thread: threading.Thread | None = None
        self._listen_sock: socket.socket | None = None
        self._latest: dict | None = None
        self._lock = threading.Lock()
        self._tracked_lock = threading.Lock()

        self._formula = CylinderFormula()
        self._proto_map: tuple = tuple(UI_TO_INTERNAL)
        self._listen_host = ""
        self._listen_port = 0
        self._platform_ip = ""
        self._platform_port = 0
        self._pose_z_offset_mm = DEFAULT_POSE_Z_OFFSET_MM

        self.last_error: str | None = None
        self._tracked_rel: np.ndarray = np.zeros(6)  # 开环追踪最近一次目标/反馈位姿（角度为弧度）

        # ── 实时曲线状态 ──
        self._plot_buffers: dict | None = None
        self._plot_fig: plt.Figure | None = None
        self._plot_axes: list[plt.Axes] | None = None
        self._plot_twin_axes: list[plt.Axes] | None = None
        self._plot_lines_pos: list[plt.Line2D] | None = None
        self._plot_lines_vel: list[plt.Line2D] | None = None
        self._animation: Any = None
        self._plot_lock = threading.Lock()

        self.load_config(config_path)

    # ── 5. 加载配置 ──────────────────────────────────────────

    def load_config(self, config_path: str) -> bool:
        """重新加载平台配置文件（几何 + UDP 参数）"""
        path = config_path if os.path.isabs(config_path) else os.path.join(_project_dir, config_path)
        if not os.path.exists(path):
            self.last_error = f"配置不存在: {path}"
            return False

        try:
            self._platform = StewartPlatform.from_config(PlatformConfig.from_json(path))

            with open(path, "r", encoding="utf-8") as f:
                d = json.load(f)

            udp = d.get("udp", {})
            self._listen_host = str(udp.get("listen_host", "192.168.3.100"))
            self._listen_port = int(udp.get("listen_port", 8080))
            self._platform_ip = str(udp.get("platform_ip", "192.168.31.88"))
            self._platform_port = int(udp.get("platform_port", 8080))
            pose_control = d.get("pose_control", {})
            self._pose_z_offset_mm = float(
                pose_control.get("z_offset_mm", DEFAULT_POSE_Z_OFFSET_MM)
            )

            cyl = udp.get("cylinder", {})
            self._formula = CylinderFormula(
                lead_mm=float(cyl.get("lead_mm", 5)),
                gear_ratio=float(cyl.get("gear_ratio", 1.5)),
                pulse_scale=float(cyl.get("pulse_scale", 10000)),
                pulses_per_rev=int(cyl.get("pulses_per_rev", 262144)),
                offset_mm=float(cyl.get("offset_mm", 10)),
                mode=str(cyl.get("mode", "protocol_doc")),
            )
            self._proto_map = tuple(udp.get("protocol_to_internal", list(UI_TO_INTERNAL)))

            # 正解零位校准：FK(ΔL=0) → home_pose，保证原点 pose_rel = 0
            try:
                zero_fk, _, _, _ = self._platform.forward_kinematics(
                    np.zeros(6), guess=self._platform.home_pose.copy(), enforce_stroke_limits=False
                )
                self._platform.home_pose = zero_fk
            except Exception:
                pass

            self._client = PlatformClient(PlatformTarget(host=self._platform_ip, port=self._platform_port))
            self._config_path = config_path
            self.last_error = None
            return True
        except Exception as e:
            self.last_error = str(e)
            return False

    # ── 1. UDP 连接 / 断开 ───────────────────────────────────

    def connect(self, platform_ip: str | None = None, platform_port: int | None = None) -> bool:
        """连接平台：启动 UDP 监听 + 发送探测包"""
        if platform_ip:
            self._platform_ip = platform_ip
            self._client = PlatformClient(PlatformTarget(host=platform_ip, port=platform_port or self._platform_port))
        if platform_port:
            self._platform_port = platform_port
            if self._client:
                self._client.target.port = platform_port

        if not self._start_listener():
            return False

        ok, msg = self._client.connect(probe=build_get_info())
        if not ok:
            self.last_error = msg
            self._stop_listener()
            return False

        self.last_error = None
        return True

    def disconnect(self) -> None:
        """断开平台连接"""
        self._stop_listener()
        if self._client:
            self._client.disconnect()

    @property
    def connected(self) -> bool:
        return self._client is not None and self._client.connected

    def _start_listener(self) -> bool:
        self._stop_listener()
        self._stop_event.clear()
        try:
            self._listen_sock = bind_udp_listen_socket(self._listen_host, self._listen_port)
        except UdpBindError as e:
            self.last_error = str(e)
            return False
        self._listen_thread = threading.Thread(target=self._listen_loop, daemon=True)
        self._listen_thread.start()
        return True

    def _stop_listener(self) -> None:
        self._stop_event.set()
        if self._listen_sock:
            try:
                self._listen_sock.close()
            except OSError:
                pass
            self._listen_sock = None
        if self._listen_thread:
            self._listen_thread.join(timeout=1)
            self._listen_thread = None

    def _listen_loop(self) -> None:
        sock = self._listen_sock
        if not sock:
            return
        sock.settimeout(0.5)
        while not self._stop_event.is_set():
            try:
                data, _ = sock.recvfrom(4096)
                self._process_packet(data)
            except socket.timeout:
                continue
            except OSError:
                break

    def _process_packet(self, data: bytes) -> None:
        parsed = parse_udp_packet(data, self._formula)
        if not isinstance(parsed, CyclicFeedback):
            return

        fb = parsed
        strokes_int = pulses_to_strokes_internal(fb.pulses, self._formula, self._proto_map)

        with self._lock:
            self._latest = {
                "timestamp": time.time(),
                "pulses": list(fb.pulses),
                "torques": list(fb.torques),
                "strokes_internal": list(strokes_int),
                "error_code": fb.error_code,
            }

    def _get_latest(self) -> dict | None:
        with self._lock:
            return self._latest

    def _pose_for_protocol(self, pose_mid: np.ndarray) -> np.ndarray:
        """用户位姿以中位为零点；29 字节协议 Z 轴需要扣除硬件保护偏置。"""
        pose = np.asarray(pose_mid, dtype=float)
        if pose.shape != (6,):
            raise ValueError("需要 6 个位姿值 [x_mm,y_mm,z_mm,rx_deg,ry_deg,rz_deg]")
        protocol_pose = pose.copy()
        protocol_pose[2] -= self._pose_z_offset_mm
        return protocol_pose

    # ── 3. 实时读取位姿 + 扭矩（调用时实时正解） ──────────────

    def read_current_pose(self, show_plot: bool = False, record_error: bool = True) -> dict | None:
        """
        返回最新 UDP 反馈，实时正解，输出绝对位姿 + 相对 home 增量。

        Parameters
        ----------
        show_plot : bool
            True 时弹出实时位置/速度曲线窗口（需 matplotlib）。
        record_error : bool
            True 时把读取/FK/绘图错误写入 last_error；后台显示线程应使用 False。
        """
        snap = self._get_latest()
        if snap is None:
            if record_error:
                self.last_error = "未收到 UDP 反馈"
            return None

        strokes = np.array(snap["strokes_internal"], dtype=float)
        lengths = absolute_lengths_from_strokes(strokes, self._platform.L0)

        pose_raw = None
        fk_ok = False
        fk_res = 0.0
        try:
            pose_raw, fk_ok, _, fk_res = self._platform.forward_kinematics(
                strokes, guess=self._platform.home_pose.copy(), enforce_stroke_limits=False
            )
        except Exception as e:
            if record_error:
                self.last_error = f"FK 正解失败: {e}"

        if pose_raw is not None:
            pose_raw = pose_raw * _FK_TO_CONTROLLER_SIGN

        home = self._platform.home_pose
        if pose_raw is not None:
            self.last_error = None
            delta = pose_raw - home
            with self._tracked_lock:
                self._tracked_rel = delta.copy()
            pose_deg = [round(delta[0], 2), round(delta[1], 2), round(delta[2], 2),
                        round(np.degrees(delta[3]), 4),
                        round(np.degrees(delta[4]), 4),
                        round(np.degrees(delta[5]), 4)]
        else:
            delta = None
            pose_deg = None

        result = {
            "timestamp": snap["timestamp"],
            "pose_raw": pose_raw.tolist() if pose_raw is not None else None,
            "pose_rel": delta.tolist() if delta is not None else None,
            "pose_deg": pose_deg,
            "strokes": snap["strokes_internal"],
            "lengths": [round(float(x), 4) for x in lengths],
            "pulses": snap["pulses"],
            "torques": snap["torques"],
            "fk_ok": fk_ok,
            "fk_residual": fk_res,
            "error_code": snap["error_code"],
        }

        if show_plot and delta is not None and self._plot_buffers is not None:
            self._update_realtime_plot(snap["timestamp"], delta)

        return result

    # ── 2. 设置目标位姿（相对中位增量） ─────────────────────

    def set_pose(self, pose_deg: list[float]) -> bool:
        """
        位姿随动，输入 = 相对中位的增量 [x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg]。
        发送前统一转换为 29 字节协议位姿。
        """
        if not self.connected:
            self.last_error = "未连接"
            return False

        target_mid = np.array(pose_deg, dtype=float)

        try:
            protocol_pose = self._pose_for_protocol(target_mid)
            pkt = build_pose_follow_xyz_abc(protocol_pose)
        except ValueError as e:
            self.last_error = str(e)
            return False

        ok, msg = self._client.send(pkt)
        if not ok:
            self.last_error = msg
            return False

        self.last_error = None
        target_mid_rad = target_mid.copy()
        target_mid_rad[3:] = np.radians(target_mid_rad[3:])
        with self._tracked_lock:
            self._tracked_rel = target_mid_rad.copy()
        return True

    # ── 4. 电缸伸长量 -> 位姿（正解） ────────────────────────

    def compute_pose_from_strokes(self, strokes: list[float]) -> dict:
        """正解：输入 6 个电缸行程 ΔL (mm)，返回绝对 + 相对位姿"""
        s = np.array(strokes, dtype=float)
        pose, ok, n_iter, residual = self._platform.forward_kinematics(s, enforce_stroke_limits=False)
        delta = pose - self._platform.home_pose
        return {
            "pose_raw": pose.tolist(),
            "pose_rel": delta.tolist(),
            "pose_deg": [round(delta[0], 2), round(delta[1], 2), round(delta[2], 2),
                         round(np.degrees(delta[3]), 4),
                         round(np.degrees(delta[4]), 4),
                         round(np.degrees(delta[5]), 4)],
            "success": ok,
            "iterations": n_iter,
            "residual": residual,
        }

    # ── 6. 移动到中位 ────────────────────────────────────────

    def move_to_mid(self) -> bool:
        """
        发送中位指令，轮询到位后立即 stop，再补发一次中位，
        避免下位机过冲后继续运行。
        到位判定：绝对位姿与 MID_POSE_ABS 偏差 < 阈值。
        """
        if not self.connected:
            self.last_error = "未连接"
            return False

        # ── 0. 等待首包 UDP 反馈 ──────────────────────────────
        first = self._wait_for_pose()
        if first is None:
            self.last_error = "未收到 UDP 反馈"
            return False

        # ── 1. 发中位 ──────────────────────────────────────────
        ok, msg = self._client.send(build_plat_mid())
        if not ok:
            self.last_error = msg
            return False

        # ── 2. 轮询到位（与 MID_POSE_ABS 偏差 < 阈值） ────────
        pos_tol = 0.5          # mm
        ang_tol = np.radians(0.05)  # rad
        stable_req = 5
        stable_cnt = 0
        prev = None

        while stable_cnt < stable_req:
            cur = self.read_current_pose()
            if cur is not None and cur["pose_raw"] is not None:
                cur_abs = np.array(cur["pose_raw"])
                diff_target = np.abs(cur_abs - MID_POSE_ABS)
                at_target = all(diff_target[:3] <= pos_tol) and all(diff_target[3:] <= ang_tol)
                if prev is not None:
                    diff_move = np.abs(cur_abs - prev)
                    stable = all(diff_move[:3] <= pos_tol) and all(diff_move[3:] <= ang_tol)
                    if at_target and stable:
                        stable_cnt += 1
                    else:
                        stable_cnt = 0
                prev = cur_abs
            time.sleep(0.01)

        # ── 3. stop ────────────────────────────────────────────
        if not self._send_stop():
            return False

        time.sleep(0.05)

        # ── 5. 再发一次中位 ────────────────────────────────────
        ok, msg = self._client.send(build_plat_mid())
        if not ok:
            self.last_error = msg
            return False

        return True

    # ── 8. S 型位姿规划 ─────────────────────────────────────

    def _wait_for_pose(self, timeout: float = 2.0) -> np.ndarray | None:
        """等待 UDP 反馈到达，返回 pose_raw；超时返回 None"""
        t0 = time.time()
        while time.time() - t0 < timeout:
            cur = self.read_current_pose(record_error=False)
            if cur is not None and cur.get("pose_raw") is not None:
                return np.array(cur["pose_raw"], dtype=float)
            time.sleep(0.02)
        return None

    def _controller_rel_from_abs(self, pose_abs: np.ndarray) -> np.ndarray:
        """绝对反馈位姿 -> 相对中位的控制位姿；平移 mm，姿态 deg。"""
        rel = np.asarray(pose_abs, dtype=float) - MID_POSE_ABS
        rel[3:] = np.degrees(rel[3:])
        return rel

    @staticmethod
    def _pose_close_to_target(
        pose_rel: np.ndarray,
        target_rel: np.ndarray,
        *,
        pos_tol_mm: float = _TARGET_POS_TOL_MM,
        att_tol_deg: float = _TARGET_ATT_TOL_DEG,
    ) -> bool:
        diff = np.abs(np.asarray(pose_rel, dtype=float) - np.asarray(target_rel, dtype=float))
        return bool(np.all(diff[:3] <= pos_tol_mm) and np.all(diff[3:] <= att_tol_deg))

    def _send_stop(self) -> bool:
        ok, msg = self._client.send(build_plat_stop())
        if not ok:
            self.last_error = msg
            return False
        self.last_error = None
        return True

    # ── 9. 实时位置/速度曲线 ─────────────────────────────────

    def start_realtime_plot(self) -> bool:
        """
        在主线程中调用，弹出 matplotlib 实时曲线窗口。
        FuncAnimation 定时从缓冲区读取数据刷新曲线。
        read_current_pose(show_plot=True) 仅向缓冲区追加数据。
        """
        if not _HAS_MPL:
            self.last_error = "matplotlib 未安装，无法显示实时曲线"
            return False

        if self._plot_fig is not None:
            return True

        try:
            plt.ion()
            axis_labels = ["X mm", "Y mm", "Z mm", "RX deg", "RY deg", "RZ deg"]
            fig, axes = plt.subplots(2, 3, figsize=(15, 6))
            manager = getattr(fig.canvas, "manager", None)
            if manager is not None and hasattr(manager, "set_window_title"):
                manager.set_window_title("6-axis realtime feedback")

            lines_pos = []
            lines_vel = []
            twin_axes = []
            for i, ax in enumerate(axes.flat):
                ax.set_title(axis_labels[i])
                ax.set_xlabel("t (s)")
                ax.grid(True, alpha=0.3)
                line_p, = ax.plot([], [], "b-", lw=1.5, label="pos")
                ax_twin = ax.twinx()
                line_v, = ax_twin.plot([], [], "r--", lw=1, alpha=0.7, label="vel")
                ax_twin.legend(loc="upper right", fontsize=8)
                lines_pos.append(line_p)
                lines_vel.append(line_v)
                twin_axes.append(ax_twin)

            fig.tight_layout()
            self._plot_fig = fig
            self._plot_axes = list(axes.flat)
            self._plot_twin_axes = twin_axes
            self._plot_lines_pos = lines_pos
            self._plot_lines_vel = lines_vel

            buf_len = _PLOT_MAX_POINTS
            self._plot_buffers = {
                "t": deque(maxlen=buf_len),
                "pose": [deque(maxlen=buf_len) for _ in range(6)],
                "vel": [deque(maxlen=buf_len) for _ in range(6)],
            }

            from matplotlib.animation import FuncAnimation

            def animate(_frame):
                buf = self._plot_buffers
                if buf is None or not buf["t"]:
                    return
                with self._plot_lock:
                    t_arr = list(buf["t"])
                    pose_snap = [list(buf["pose"][i]) for i in range(6)]
                    vel_snap = [list(buf["vel"][i]) for i in range(6)]
                t_now = t_arr[-1]
                t_min = t_now - _PLOT_WINDOW_SEC
                for i in range(6):
                    self._plot_lines_pos[i].set_data(t_arr, pose_snap[i])
                    self._plot_lines_vel[i].set_data(t_arr, vel_snap[i])
                    self._plot_axes[i].relim()
                    self._plot_twin_axes[i].relim()
                    self._plot_axes[i].autoscale_view(scalex=False, scaley=True)
                    self._plot_twin_axes[i].autoscale_view(scalex=False, scaley=True)
                    self._plot_axes[i].set_xlim(t_min, t_now if t_now > t_min else t_min + 1.0)

            self._animation = FuncAnimation(fig, animate, interval=50, cache_frame_data=False)
            fig.show()
            self.last_error = None
            return True
        except Exception as e:
            self._plot_buffers = None
            self._plot_fig = None
            self._plot_axes = None
            self._plot_twin_axes = None
            self._plot_lines_pos = None
            self._plot_lines_vel = None
            self._animation = None
            self.last_error = f"实时曲线初始化失败: {e}"
            return False

    def _update_realtime_plot(self, timestamp: float, pose_rel: np.ndarray) -> None:
        """向缓冲区追加数据点（线程安全），不碰 GUI。"""
        buf = self._plot_buffers
        if buf is None:
            return

        with self._plot_lock:
            t_elapsed = timestamp - buf["t"][-1] if buf["t"] else 0.0
            buf["t"].append(timestamp)

            for i in range(6):
                v = pose_rel[i] if i < 3 else np.degrees(pose_rel[i])
                prev_v = buf["pose"][i][-1] if buf["pose"][i] else None
                buf["pose"][i].append(v)
                if t_elapsed > 0 and prev_v is not None:
                    dv = (v - prev_v) / t_elapsed
                else:
                    dv = 0.0
                buf["vel"][i].append(dv)

    def close(self) -> None:
        """关闭实时曲线窗口。"""
        if self._animation is not None:
            self._animation.event_source.stop()
            self._animation = None
        if self._plot_fig is not None:
            plt.close(self._plot_fig)
            self._plot_fig = None
        self._plot_buffers = None
        self._plot_axes = None
        self._plot_twin_axes = None
        self._plot_lines_pos = None
        self._plot_lines_vel = None

    def move_pose_s_curve(
        self,
        target_pose: list[float],
        duration: float = 2.0,
        *,
        settle_timeout: float = 5.0,
    ) -> bool:
        """
        S 型规划：在 duration 秒内从当前位置插补到 target_pose。
        target_pose 是相对中位 (MID_POSE_ABS) 的增量
        [x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg]。
        起点 = 当前绝对位姿 .pose_raw − MID_POSE_ABS，统一到中位坐标系。
        轨迹发送完成后轮询反馈，到位并稳定后发送 stop，成功后返回 True。
        """
        if not self.connected:
            self.last_error = "未连接"
            return False

        self.last_error = None
        cur_abs = self._wait_for_pose()
        if cur_abs is None:
            self.last_error = "未收到 UDP 反馈，无法确认 S 曲线起点"
            return False

        print(f"当前绝对位姿: {cur_abs.tolist()}")
        start_rel = self._controller_rel_from_abs(cur_abs)
        print(f"中位绝对位姿: {MID_POSE_ABS.tolist()}")
        print(f"(中位 - home_pose Z差: {MID_POSE_ABS[2] - self._platform.home_pose[2]:.1f}mm)")

        target = np.array(target_pose, dtype=float)
        if target.shape != (6,):
            self.last_error = "target_pose 需要 6 个值 [x_mm,y_mm,z_mm,rx_deg,ry_deg,rz_deg]"
            return False

        delta = target - start_rel
        print(f"当前位姿 (相对中位): {start_rel.tolist()}")
        print(f"目标位姿 (相对中位): {target.tolist()}")
        print(f"位姿增量: {delta.tolist()}")

        dt = _SCURVE_DT_SEC
        steps = max(2, int(duration / dt))

        for i in range(1, steps + 1):
            t = i / steps
            s = 3 * t * t - 2 * t * t * t
            interp = (start_rel + s * delta).tolist()
            if not self.set_pose(interp):
                self._send_stop()
                return False
            print(f"pt{i}:{interp}")
            time.sleep(dt)

        stable_cnt = 0
        deadline = time.time() + settle_timeout
        while time.time() < deadline:
            cur = self.read_current_pose()
            if cur is not None and cur.get("pose_raw") is not None:
                pose_rel = self._controller_rel_from_abs(np.array(cur["pose_raw"], dtype=float))
                if self._pose_close_to_target(pose_rel, target):
                    stable_cnt += 1
                    if stable_cnt >= _TARGET_STABLE_SAMPLES:
                        if not self._send_stop():
                            return False
                        self.last_error = None
                        return True
                else:
                    stable_cnt = 0

            time.sleep(0.02)

        self._send_stop()
        self.last_error = f"S curve target settle timeout: target={target.tolist()}"
        return False

    # ── 7. 回原点 ────────────────────────────────────────────

    def move_to_home(self) -> bool:
        """发送回原点指令"""
        if not self.connected:
            self.last_error = "未连接"
            return False
        ok, msg = self._client.send(build_plat_reset())
        if not ok:
            self.last_error = msg
            return False
        with self._tracked_lock:
            self._tracked_rel[:] = 0.0
        return True


# ═══════════════════════════════════════════════════════════════
#  演示程序
# ═══════════════════════════════════════════════════════════════

def _print_pose(line_header, data):
    """多行显示正解六轴位姿 + 行程 + 扭矩"""
    if data is None:
        print(f"  {line_header}: (no feedback)", flush=True)
        return
    raw = data.get("pose_raw")
    rel = data.get("pose_rel")
    s = data.get("strokes")
    t = data.get("torques")
    print(f"  {line_header}:")
    if raw:
        print(f"    绝对[X{raw[0]:7.1f} Y{raw[1]:7.1f} Z{raw[2]:7.1f}"
              f" rx{np.degrees(raw[3]):6.2f} ry{np.degrees(raw[4]):6.2f} rz{np.degrees(raw[5]):6.2f}]")
    if rel:
        print(f"    增量[x{rel[0]:7.1f} y{rel[1]:7.1f} z{rel[2]:7.1f}"
              f" rx{np.degrees(rel[3]):6.2f} ry{np.degrees(rel[4]):6.2f} rz{np.degrees(rel[5]):6.2f}]")
    if s:
        print(f"    S:[{s[0]:6.1f} {s[1]:6.1f} {s[2]:6.1f} {s[3]:6.1f} {s[4]:6.1f} {s[5]:6.1f}]")
    if t:
        print(f"    T:[{t[0]:5d} {t[1]:5d} {t[2]:5d} {t[3]:5d} {t[4]:5d} {t[5]:5d}]")


def _demo_read_worker(plat, stop_event, show_plot=False):
    while not stop_event.is_set():
        data = plat.read_current_pose(show_plot=show_plot, record_error=False)
        if data is not None and data.get("pose_raw") is not None:
            r = data["pose_raw"]
            s = data["strokes"]
            print(f"    X{r[0]:7.1f} ，Y{r[1]:7.1f} ，Z{r[2]:7.1f}"
                  f" rx{np.degrees(r[3]):6.2f} ry{np.degrees(r[4]):6.2f} rz{np.degrees(r[5]):6.2f}"
                  f"  S[{s[0]:5.1f} {s[1]:5.1f} {s[2]:5.1f} {s[3]:5.1f} {s[4]:5.1f} {s[5]:5.1f}]",
                  flush=True)
        stop_event.wait(0.1)


def _demo(ip=None, port=8080):

    plat = SixAxisPlatform()
    ok = plat.connect(ip, port)
    if not ok:
        print(f"  last_error: {plat.last_error}")
        print("  (hardware not reachable, skip network tests)")
    else:
        print(f"  connected: {plat.connected}")

        # ── 主线程创建实时曲线窗口（可选） ────────────────────
        show_plot = True   # 改成 False 则不弹窗
        if show_plot:
            plat.start_realtime_plot()
        print("--- 2. 异步读取线程已启动" + (" (带实时曲线)" if show_plot else "") + " ---")

        stop_read = threading.Event()
        read_thread = threading.Thread(target=_demo_read_worker, args=(plat, stop_read, show_plot), daemon=True)
        read_thread.start()

        print("--- 3. move_pose_s_curve: [0,0,0,0,0,5] 1s ---")
        ok = plat.move_pose_s_curve([0, 0, 0, 0, 0, 5], duration=1.0)
        print(f"  move_pose_s_curve returned: {ok}, last_error={plat.last_error}")

        # ── 泵 GUI 事件循环，FuncAnimation 才能刷新曲线 ──────
        print("--- 4. sleep 10s (GUI active) ---")
        t_end = time.time() + 10
        while time.time() < t_end:
            if show_plot and _HAS_MPL:
                plt.pause(0.05)
            else:
                time.sleep(0.05)

        # ── 停止读取线程 ──────────────────────────────────────
        stop_read.set()
        read_thread.join(timeout=2)

        # ── 断开 ──────────────────────────────────────────────
        print("--- 5. disconnect ---")
        plat.close()
        plat.disconnect()
        print(f"  connected: {plat.connected}")
        print()

    print("=" * 70)
    print("  demo finished")
    print("=" * 70)


if __name__ == "__main__":
    _demo()
