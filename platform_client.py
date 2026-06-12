"""向六自由度平台发送 UDP 控制命令。"""

from __future__ import annotations

import socket
from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass
class PlatformTarget:
    host: str = "192.168.31.88"
    port: int = 8080


class PlatformClient:
    def __init__(self, target: PlatformTarget) -> None:
        self.target = target
        self._sock: Optional[socket.socket] = None
        self.connected = False
        self.last_error: Optional[str] = None
        self.packets_sent = 0

    def connect(self, probe: Optional[bytes] = None) -> Tuple[bool, str]:
        """建立 UDP 发送通道，可选发送探测包（如 CMD=0x00 查询信息）。"""
        self.close()
        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            addr = (self.target.host, int(self.target.port))
            if probe is not None:
                self._sock.sendto(probe, addr)
                self.packets_sent += 1
            self.connected = True
            self.last_error = None
            return True, "连接成功"
        except OSError as e:
            self.last_error = str(e)
            self.connected = False
            if self._sock is not None:
                try:
                    self._sock.close()
                except OSError:
                    pass
                self._sock = None
            return False, str(e)

    def disconnect(self) -> None:
        self.close()
        self.connected = False

    def send(self, packet: bytes) -> Tuple[bool, str]:
        if not self.connected or self._sock is None:
            return False, "未连接平台，请先点击「连接平台」"
        try:
            self._sock.sendto(packet, (self.target.host, int(self.target.port)))
            self.packets_sent += 1
            self.last_error = None
            return True, f"已发送 {len(packet)} 字节 → {self.target.host}:{self.target.port}"
        except OSError as e:
            self.last_error = str(e)
            return False, str(e)

    def close(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None
