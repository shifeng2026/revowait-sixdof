"""
UDP 周期反馈接收 + 正解联动服务（后台线程）

行程：ΔL = round(P÷N÷S×D−10, 4)；正解杆长 L = 几何 L0 + ΔL（与正解计算页一致）。
"""

from __future__ import annotations

import socket
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Optional

import numpy as np

from calibration import internal_array_to_ui
from protocol_udp import (
    CylinderFormula,
    CyclicFeedback,
    absolute_lengths_from_strokes,
    parse_udp_packet,
    protocol_lengths_to_internal,
    pulses_to_strokes_internal,
)
from stewart_fk import StewartPlatform


class UdpBindError(OSError):
    """本机 UDP 监听端口绑定失败（供界面提示）。"""


def _normalize_bind_host(host: str) -> str:
    h = (host or "").strip()
    return h if h else "0.0.0.0"


def format_udp_bind_error(host: str, port: int, exc: OSError) -> str:
    """将 Windows/Linux 套接字错误转成可操作的说明。"""
    host = _normalize_bind_host(host)
    win = getattr(exc, "winerror", None)
    errno = getattr(exc, "errno", None)
    lines = [
        f"无法在本机 {host}:{port} 上监听 UDP。",
        f"系统错误：{exc}",
    ]
    if win == 10013 or errno == 13:
        lines.extend(
            [
                "",
                "常见原因：",
                "  1. 该端口已被其他程序占用（含上次未退出的本程序、RW_DEMO、Tomcat 等）；",
                "  2. Windows 因 Hyper-V / WSL 保留了该端口；",
                "  3. 安全软件拦截。",
                "",
                "建议处理：",
                "  · 在任务管理器结束占用端口的程序，或重启电脑后再试；",
                "  · 将「本机监听端口」改为 8090、9000 等未占用端口，并在平台/上位机配置里",
                "    把「周期反馈目标端口」改成与这里一致；",
                "  · 管理员 CMD 执行：netsh interface ipv4 show excludedportrange",
                "    查看是否落在系统保留段内。",
            ]
        )
    elif win == 10048 or errno == 98:
        lines.extend(
            [
                "",
                "该端口已被占用。请关闭占用程序，或换一个本机监听端口。",
            ]
        )
    elif win == 10049 or errno == 99:
        lines.extend(
            [
                "",
                "监听地址无效。请将「本机 IP」留空或填 0.0.0.0，不要填平台 IP。",
            ]
        )
    return "\n".join(lines)


def bind_udp_listen_socket(host: str, port: int) -> socket.socket:
    """创建并绑定 UDP 监听套接字；失败时抛出 UdpBindError。"""
    host = _normalize_bind_host(host)
    if not (1 <= int(port) <= 65535):
        raise UdpBindError(f"端口号无效：{port}（应为 1~65535）")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((host, port))
    except OSError as e:
        sock.close()
        raise UdpBindError(format_udp_bind_error(host, port, e)) from e
    return sock


@dataclass
class UdpListenConfig:
    host: str = "0.0.0.0"
    port: int = 8080
    platform_ip: str = "192.168.31.200"
    platform_port: int = 8080
    formula: CylinderFormula = field(default_factory=CylinderFormula)
    protocol_to_internal: tuple = (0, 1, 2, 3, 4, 5)
    cylinder_home_lengths: Optional[list] = None


@dataclass
class FkSnapshot:
    """一次周期反馈对应的正解结果。"""

    timestamp: float
    feedback: CyclicFeedback
    lengths_protocol: list
    lengths_internal: list
    lengths_ui: list
    strokes_internal: list
    strokes_ui: list
    pose_raw: Optional[np.ndarray] = None
    pose_display: Optional[np.ndarray] = None
    delta_display: Optional[np.ndarray] = None
    fk_ok: bool = False
    fk_residual: float = 0.0
    fk_iter: int = 0
    error: Optional[str] = None
    limit_warning: Optional[str] = None


class UdpFkReceiver:
    def __init__(
        self,
        platform: StewartPlatform,
        config: UdpListenConfig,
        on_update: Optional[Callable[[FkSnapshot], None]] = None,
    ) -> None:
        self.platform = platform
        self.config = config
        self.on_update = on_update
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._sock: Optional[socket.socket] = None
        self.latest: Optional[FkSnapshot] = None
        self._lock = threading.Lock()
        self.packet_count = 0
        self.last_bind_error: Optional[str] = None
        self._listening = False

    def set_pulse_zero_from_latest(self) -> bool:
        """UDP 正解已用绝对行程公式，此按钮仅提示。"""
        with self._lock:
            return self.latest is not None

    def process_packet(self, data: bytes) -> Optional[FkSnapshot]:
        parsed = parse_udp_packet(data, self.config.formula)
        if not isinstance(parsed, CyclicFeedback):
            return None

        fb = parsed
        idx_map = list(self.config.protocol_to_internal)
        strokes_proto = list(fb.cylinder_lengths_mm)
        strokes_int = pulses_to_strokes_internal(
            fb.pulses, self.config.formula, idx_map
        )
        L0 = self.platform.L0
        lengths_int = absolute_lengths_from_strokes(strokes_int, L0)
        strokes_ui = list(internal_array_to_ui(np.array(strokes_int)))
        lengths_ui = list(internal_array_to_ui(np.array(lengths_int)))

        snap = FkSnapshot(
            timestamp=time.time(),
            feedback=fb,
            lengths_protocol=strokes_proto,
            lengths_internal=lengths_int,
            lengths_ui=lengths_ui,
            strokes_internal=list(strokes_int),
            strokes_ui=strokes_ui,
        )

        try:
            strokes_for_fk = np.array(strokes_int, dtype=float)
            ok_lim, lim_msg = self.platform.check_strokes(strokes_for_fk)
            if not ok_lim:
                snap.limit_warning = (
                    lim_msg + "（仍尝试正解；行程为协议公式相对几何零位的 ΔL）"
                )
            pose, ok, n_iter, res = self.platform.forward_kinematics(
                strokes_for_fk, enforce_stroke_limits=False
            )
            delta = pose - self.platform.home_pose
            snap.pose_raw = pose
            snap.pose_display = self.platform.display_pose(pose, strokes_for_fk)
            snap.delta_display = self.platform.display_delta(
                delta,
                strokes_for_fk,
                raw_pose=pose,
                raw_home_pose=self.platform.home_pose,
            )
            snap.fk_ok = ok
            snap.fk_residual = res
            snap.fk_iter = n_iter
        except Exception as e:
            snap.error = str(e)

        with self._lock:
            self.latest = snap
            self.packet_count += 1

        if self.on_update:
            self.on_update(snap)
        return snap

    def _run(self) -> None:
        try:
            if self._sock is None:
                return
            self._sock.settimeout(0.5)
            while not self._stop.is_set():
                try:
                    data, _addr = self._sock.recvfrom(4096)
                    self.process_packet(data)
                except socket.timeout:
                    continue
                except OSError:
                    if not self._stop.is_set():
                        raise
        finally:
            self._listening = False
            if self._sock:
                try:
                    self._sock.close()
                except OSError:
                    pass
                self._sock = None

    def start(self) -> None:
        if self._listening and self._thread and self._thread.is_alive():
            return
        if self._thread is not None and not self._thread.is_alive():
            self._thread = None
        self._stop.clear()
        self.last_bind_error = None
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None
        try:
            self._sock = bind_udp_listen_socket(
                self.config.host, self.config.port
            )
        except UdpBindError as e:
            self.last_bind_error = str(e)
            self._listening = False
            raise
        self._listening = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """停止监听；先关套接字以打断 recvfrom，避免界面长时间卡住。"""
        self._listening = False
        self._stop.set()
        sock = self._sock
        self._sock = None
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass
        t = self._thread
        self._thread = None
        if t is not None and t.is_alive():
            t.join(timeout=0.5)

    def is_running(self) -> bool:
        """界面用：是否处于「已点过开始监听」状态（与按钮文案一致）。"""
        return self._listening
