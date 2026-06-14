"""
六轴平台轻量化控制接口

Usage:
    from interface import SixAxisPlatform

    plat = SixAxisPlatform()
    plat.connect()

    pose = plat.set("read_pose")
    plat.set("pose", [0, 0, 100, 0, 0, 0])
    plat.set("move_pose_s_curve", [0, 0, 0, 0, 0, 5], duration=1.0)
    plat.set("move_to_mid")
    plat.set("move_to_home")
    plat.set("disconnect")
"""

from __future__ import annotations

import json
import os
import socket
import sys
import threading
import time
from collections import deque
from typing import Any

import numpy as np

try:
    import matplotlib.pyplot as plt
    _HAS_MPL = True
except ImportError:
    _HAS_MPL = False

_src_dir = os.path.dirname(os.path.abspath(__file__))
_project_dir = os.path.dirname(_src_dir)
if _src_dir not in sys.path:
    sys.path.insert(0, _src_dir)

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
from logger_config import get_logger

logger = get_logger(__name__)

MID_POSE_ABS = np.array([0, 0, 920, 0, 0, 0], dtype=float)
DEFAULT_POSE_Z_OFFSET_MM = 10.0
_FK_TO_CONTROLLER_SIGN = np.array([-1.0, -1.0, 1.0, -1.0, -1.0, 1.0])

_PLOT_WINDOW_SEC = 10.0
_PLOT_MAX_POINTS = 300
_SCURVE_DT_SEC = 0.02
_TARGET_POS_TOL_MM = 0.5
_TARGET_ATT_TOL_DEG = 0.05
_TARGET_STABLE_SAMPLES = 5

# 轴限位（相对中位增量）
_POS_LIMIT_MM = 200.0
_ROLL_LIMIT_DEG = 25.0
_PITCH_LIMIT_DEG = 25.0
_YAW_LIMIT_DEG = 28.0


class SixAxisPlatform:

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
        self._tracked_rel: np.ndarray = np.zeros(6)

        self._plot_buffers: dict | None = None
        self._plot_fig: plt.Figure | None = None
        self._plot_axes: list[plt.Axes] | None = None
        self._plot_twin_axes: list[plt.Axes] | None = None
        self._plot_lines_pos: list[plt.Line2D] | None = None
        self._plot_lines_vel: list[plt.Line2D] | None = None
        self._animation: Any = None
        self._plot_lock = threading.Lock()

        self.load_config(config_path)

    def load_config(self, config_path: str) -> bool:
        path = config_path if os.path.isabs(config_path) else os.path.join(_project_dir, config_path)
        if not os.path.exists(path):
            self.last_error = f"config not found: {path}"
            logger.error("config not found: %s", path)
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
            self._pose_z_offset_mm = float(
                d.get("pose_control", {}).get("z_offset_mm", DEFAULT_POSE_Z_OFFSET_MM)
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
            logger.exception("config load failed path=%s", path)
            return False

    def connect(self, platform_ip: str | None = None, platform_port: int | None = None) -> bool:
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
        pose = np.asarray(pose_mid, dtype=float)
        if pose.shape != (6,):
            raise ValueError("pose must have 6 values [x,y,z,rx,ry,rz]")
        protocol_pose = pose.copy()
        protocol_pose[2] -= self._pose_z_offset_mm
        return protocol_pose

    def read_current_pose(self, show_plot: bool = False, record_error: bool = True) -> dict | None:
        snap = self._get_latest()
        if snap is None:
            if record_error:
                self.last_error = "no udp feedback"
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
                self.last_error = f"FK failed: {e}"

        if pose_raw is not None:
            pose_raw = pose_raw * _FK_TO_CONTROLLER_SIGN
            self.last_error = None
            delta = pose_raw - MID_POSE_ABS
            with self._tracked_lock:
                self._tracked_rel = delta.copy()
            pose_deg = [float(round(delta[0], 2)), float(round(delta[1], 2)), float(round(delta[2], 2)),
                        float(round(np.degrees(delta[3]), 4)),
                        float(round(np.degrees(delta[4]), 4)),
                        float(round(np.degrees(delta[5]), 4))]
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

    def set_pose(self, pose_deg: list[float]) -> bool:
        if not self.connected:
            self.last_error = "not connected"
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

    def compute_pose_from_strokes(self, strokes: list[float]) -> dict:
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

    def move_to_mid(self) -> bool:
        if not self.connected:
            self.last_error = "not connected"
            return False

        first = self._wait_for_pose()
        if first is None:
            self.last_error = "no udp feedback"
            return False

        ok, msg = self._client.send(build_plat_mid())
        if not ok:
            self.last_error = msg
            return False

        pos_tol = 0.5
        ang_tol = np.radians(0.05)
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

        if not self._send_stop():
            return False

        time.sleep(0.05)
        ok, msg = self._client.send(build_plat_mid())
        if not ok:
            self.last_error = msg
            return False
        return True

    def _wait_for_pose(self, timeout: float = 2.0) -> np.ndarray | None:
        t0 = time.time()
        while time.time() - t0 < timeout:
            cur = self.read_current_pose(record_error=False)
            if cur is not None and cur.get("pose_raw") is not None:
                return np.array(cur["pose_raw"], dtype=float)
            time.sleep(0.02)
        return None

    def _controller_rel_from_abs(self, pose_abs: np.ndarray) -> np.ndarray:
        rel = np.asarray(pose_abs, dtype=float) - MID_POSE_ABS
        rel[3:] = np.degrees(rel[3:])
        return rel

    @staticmethod
    def _pose_close_to_target(
        pose_rel: np.ndarray, target_rel: np.ndarray,
        *, pos_tol_mm: float = _TARGET_POS_TOL_MM, att_tol_deg: float = _TARGET_ATT_TOL_DEG,
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

    def start_realtime_plot(self) -> bool:
        """
        在主线程中调用，弹出 matplotlib 实时曲线窗口。
        FuncAnimation 定时从缓冲区读取数据刷新曲线。
        read_current_pose(show_plot=True) 仅向缓冲区追加数据。
        """
        if not _HAS_MPL:
            self.last_error = "matplotlib 未安装，无法显示实时曲线"
            logger.error("realtime plot unavailable: matplotlib not installed")
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
            logger.info("realtime plot started")
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
            logger.exception("realtime plot initialization failed")
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
            logger.error("move_pose_s_curve rejected: not connected target=%s", target_pose)
            return False

        self.last_error = None
        logger.info("s_curve start target=%s duration=%.3f settle_timeout=%.3f", target_pose, duration, settle_timeout)
        cur_abs = self._wait_for_pose()
        if cur_abs is None:
            self.last_error = "未收到 UDP 反馈，无法确认 S 曲线起点"
            logger.error("s_curve failed: no start feedback target=%s", target_pose)
            return False

        # print(f"当前绝对位姿: {cur_abs.tolist()}")
        start_rel = self._controller_rel_from_abs(cur_abs)
        logger.info("s_curve start_pose_abs=%s start_rel=%s", cur_abs.tolist(), start_rel.tolist())
        # print(f"中位绝对位姿: {MID_POSE_ABS.tolist()}")
        # print(f"(中位 - home_pose Z差: {MID_POSE_ABS[2] - self._platform.home_pose[2]:.1f}mm)")

        target = np.array(target_pose, dtype=float)
        if target.shape != (6,):
            self.last_error = "target_pose 需要 6 个值 [x_mm,y_mm,z_mm,rx_deg,ry_deg,rz_deg]"
            logger.error("s_curve invalid target=%s", target_pose)
            return False

        limits = [
            ("x", _POS_LIMIT_MM), ("y", _POS_LIMIT_MM), ("z", _POS_LIMIT_MM),
            ("rx(roll)", _ROLL_LIMIT_DEG),
            ("ry(pitch)", _PITCH_LIMIT_DEG),
            ("rz(yaw)", _YAW_LIMIT_DEG),
        ]
        for i, (name, limit) in enumerate(limits):
            if abs(target[i]) > limit:
                self.last_error = f"{name} 超限: {target[i]:.2f} (limit ±{limit})"
                logger.error("s_curve target %s out of range: %.2f (limit ±%s)", name, target[i], limit)
                return False

        delta = target - start_rel
        # print(f"当前位姿 (相对中位): {start_rel.tolist()}")
        # print(f"目标位姿 (相对中位): {target.tolist()}")
        # print(f"位姿增量: {delta.tolist()}")

        dt = _SCURVE_DT_SEC
        steps = max(2, int(duration / dt))
        logger.info("s_curve trajectory steps=%s dt=%.3f delta=%s", steps, dt, delta.tolist())

        for i in range(1, steps + 1):
            t = i / steps
            s = 3 * t * t - 2 * t * t * t
            interp = (start_rel + s * delta).tolist()
            if not self.set_pose(interp):
                self._send_stop()
                logger.error("s_curve interrupted: set_pose failed at step=%s/%s error=%s", i, steps, self.last_error)
                return False
            # print(f"pt{i}:{interp}")
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
                        logger.info("s_curve reached target=%s stable_samples=%s", target.tolist(), stable_cnt)
                        return True
                else:
                    stable_cnt = 0

            time.sleep(0.02)

        self._send_stop()
        self.last_error = f"S curve target settle timeout: target={target.tolist()}"
        logger.error("s_curve settle timeout target=%s stable_count=%s", target.tolist(), stable_cnt)
        return False

    # ── 7. 回原点 ────────────────────────────────────────────

    def move_to_home(self) -> bool:
        """发送回原点指令"""
        if not self.connected:
            self.last_error = "未连接"
            logger.error("move_to_home rejected: not connected")
            return False
        ok, msg = self._client.send(build_plat_reset())
        if not ok:
            self.last_error = msg
            logger.error("move_to_home send failed: %s", msg)
            return False
        with self._tracked_lock:
            self._tracked_rel[:] = 0.0
        self.last_error = None
        logger.info("move_to_home command sent")
        return True

    # ── 8. 统一 Set / Get 接口 ─────────────────────────────────

    def set(self, key: str, value: Any = None, **kwargs) -> Any:
        """
        统一命令接口。

        key 支持:
          connect           - value=str(ip) 可选
          disconnect
          read_pose         - 返回 read_current_pose() 结果
          pose              - value=list[6] 相对中位增量
          move_pose_s_curve - value=list[6], kwargs: duration=
          move_to_home
          move_to_mid
        """
        actions = {
            "connect": lambda: self.connect(
                value if isinstance(value, str) else None,
                kwargs.get("port"),
            ),
            "disconnect": lambda: self.disconnect(),
            "pose": lambda: self.set_pose(value, **kwargs),
            "move_pose_s_curve": lambda: self.move_pose_s_curve(value, **kwargs),
            "move_to_home": lambda: self.move_to_home(),
            "move_to_mid": lambda: self.move_to_mid(),
        }
        fn = actions.get(key)
        if fn is None:
            self.last_error = f"unknown command: {key}"
            return None
        return fn()

    def get(self, key: str) -> Any:
        """统一查询接口。key 支持: pose, connected, last_error, pose_raw"""
        state = {
            "pose": lambda: self.read_current_pose(),
            "connected": lambda: self.connected,
            "last_error": lambda: self.last_error,
            # "read_pose": lambda: self.read_current_pose(**kwargs),
            "pose_deg": lambda: (
                r.get("pose_deg") if (r := self.read_current_pose()) else None
            ),
        }
        fn = state.get(key)
        return fn() if fn is not None else None
