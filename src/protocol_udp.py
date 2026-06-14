"""
铂沃 RWH743 ECAT 通信协议 — 反馈报文解析（Python）

对应《通信协议 V1.0.5》第五章：
  - 8 字节：命令反馈（帧头 A5 55）
  - 54 字节：周期性反馈（帧头 AA 55，CMD=0x50，与 RW_DEMO / rw_udp.c 一致）

电缸行程（由周期反馈脉冲 P 换算）：

  UDP 实时 / 协议正解（与手册一致）：
    行程 ΔL = round(P ÷ N ÷ S × D − offset, 4)   N=2^18, D=5, S=1.5, offset=10
    杆长 L = 几何零位 L0 + ΔL（与「正解计算」页相同）

  RW_DEMO 显示用（可选）：
    L = P / pulse_scale × (D / S)
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np

# 8 字节命令反馈
CMD_FRAME_HEAD = 0xA5
CMD_FRAME_ACK = 0x55

# 54 字节周期反馈（第五章 #2）
CYCLIC_FRAME_HEAD = 0xAA
CYCLIC_FRAME_ACK = 0x55
CYCLIC_CMD = 0x50
CYCLIC_FRAME_LEN = 54

# 周期反馈 [28..51] 为 6×INT32 电机转矩；RW_DEMO 显示为 raw÷10
TORQUE_DISPLAY_DIVISOR = 10


def crc16_modbus(data: bytes) -> Tuple[int, int]:
    """返回 (CRC高字节, CRC低字节)，与 JS 版一致。"""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    hi = (crc >> 8) & 0xFF
    lo = crc & 0xFF
    return hi, lo


def verify_crc(data: bytes) -> bool:
    if len(data) < 3:
        return False
    payload = data[:-2]
    hi, lo = crc16_modbus(payload)
    return data[-2] == hi and data[-1] == lo


def is_cyclic_feedback(data: bytes) -> bool:
    """第五章 54 字节周期反馈：AA 55 50 …"""
    return (
        len(data) >= 3
        and data[0] == CYCLIC_FRAME_HEAD
        and data[1] == CYCLIC_FRAME_ACK
        and data[2] == CYCLIC_CMD
    )


@dataclass
class CylinderFormula:
    """电缸长度换算参数。"""

    lead_mm: float = 5.0  # D 导程
    gear_ratio: float = 1.5  # S 减速比
    # rw_demo：与官方上位机一致
    pulse_scale: float = 10000.0
    # protocol_doc：协议印刷公式（易导致 L 为负，勿与 Demo 混用）
    pulses_per_rev: int = 262144
    offset_mm: float = 10.0
    mode: str = "protocol_doc"  # rw_demo | protocol_doc
    stroke_decimals: int = 4

    def protocol_stroke_mm(self, pulse: int) -> float:
        """协议行程 ΔL = P÷N÷S×D − offset，保留小数位。"""
        raw = (
            pulse / self.pulses_per_rev / self.gear_ratio * self.lead_mm
        ) - self.offset_mm
        return round(raw, self.stroke_decimals)

    def length_from_pulse(self, pulse: int) -> float:
        if self.mode == "protocol_doc":
            return self.protocol_stroke_mm(pulse)
        return round(
            (pulse / self.pulse_scale) * (self.lead_mm / self.gear_ratio),
            self.stroke_decimals,
        )

    def stroke_from_pulse(self, pulse: int) -> float:
        """当前电缸行程（UDP 正解用 protocol_stroke_mm）。"""
        return self.protocol_stroke_mm(pulse)


@dataclass
class CyclicFeedback:
    """54 字节周期反馈解析结果。"""

    raw: bytes
    cmd: int
    error_code: int
    motor_errors: List[bool]
    pulses: List[int]
    torques: List[int]
    cylinder_lengths_mm: List[float]  # 协议电机顺序：行程 ΔL (mm)，保留 4 位小数

    @property
    def ok(self) -> bool:
        return self.error_code == 0 and not any(self.motor_errors)


def protocol_index_for_internal(
    internal_index: int, protocol_to_internal: Sequence[int]
) -> int:
    """内部电缸下标 0..5 → 协议电机下标 0..5。"""
    for p, int_i in enumerate(protocol_to_internal):
        if int(int_i) == int(internal_index):
            return p
    return int(internal_index)


def map_protocol_values_to_ui(
    values: Sequence[int | bool],
    protocol_to_internal: Sequence[int],
) -> List[int | bool]:
    """协议电机顺序 → 界面电缸 1~6 顺序。"""
    from calibration import UI_TO_INTERNAL

    out: List[int | bool] = []
    for ui in range(6):
        proto = protocol_index_for_internal(int(UI_TO_INTERNAL[ui]), protocol_to_internal)
        out.append(values[proto])
    return out


def torque_raw_to_display(
    raw: int, *, divisor: float = TORQUE_DISPLAY_DIVISOR
) -> float:
    """与 RW_DEMO 一致：显示值 = 原始 INT32 ÷ divisor。"""
    return float(raw) / float(divisor)


def parse_command_response(data: bytes) -> Optional[dict]:
    if len(data) < 8 or data[0] != CMD_FRAME_HEAD or data[1] != CMD_FRAME_ACK:
        return None
    if not verify_crc(data[:8]):
        return None
    return {
        "type": "command_response",
        "cmd": data[2],
        "data": list(data[3:6]),
    }


def parse_cyclic_response(
    data: bytes,
    formula: Optional[CylinderFormula] = None,
    *,
    check_crc: bool = True,
) -> Optional[CyclicFeedback]:
    """
    解析 54 字节周期性反馈报文（第五章 #2）。

    结构（与 revowait-motion-cueing/rw_udp.c、抓包一致）：
      [0] 0xAA  [1] 0x55  [2] 0x50  [3] 错误码
      [4..27]  6×INT32 电机位置脉冲 P
      [28..51] 6×INT32 电机转矩
      [52..53] CRC16（对前 52 字节）
    """
    data = bytes(data[:CYCLIC_FRAME_LEN])
    if len(data) < CYCLIC_FRAME_LEN or not is_cyclic_feedback(data):
        return None
    if check_crc and not verify_crc(data):
        return None

    formula = formula or CylinderFormula()
    error_code = data[3]

    pulses = []
    for i in range(6):
        off = 4 + i * 4
        pulses.append(struct.unpack_from("<i", data, off)[0])

    torques = []
    for i in range(6):
        off = 28 + i * 4
        torques.append(struct.unpack_from("<i", data, off)[0])

    lengths = [formula.stroke_from_pulse(p) for p in pulses]
    motor_errors = [bool(error_code & (1 << i)) for i in range(6)]

    return CyclicFeedback(
        raw=data,
        cmd=data[2],
        error_code=error_code,
        motor_errors=motor_errors,
        pulses=pulses,
        torques=torques,
        cylinder_lengths_mm=lengths,
    )


def parse_udp_packet(
    data: bytes,
    formula: Optional[CylinderFormula] = None,
) -> Optional[dict | CyclicFeedback]:
    if len(data) == 8:
        return parse_command_response(data)
    if len(data) >= CYCLIC_FRAME_LEN:
        cyclic = parse_cyclic_response(data[:CYCLIC_FRAME_LEN], formula)
        if cyclic is not None:
            return cyclic
    return None


def protocol_lengths_to_internal(
    lengths_protocol: Sequence[float],
    protocol_to_internal: Sequence[int],
) -> List[float]:
    """
    协议电机顺序 -> 内部电缸 1~6 顺序。

    protocol_to_internal[协议下标] = 内部下标
    例：协议电机1 对应 内部4 -> [3,4,5,0,1,2] 若协议顺序=界面1~6
    """
    src = list(lengths_protocol)
    if len(src) != 6 or len(protocol_to_internal) != 6:
        raise ValueError("需要 6 个长度与 6 个映射下标")
    out = [0.0] * 6
    for proto_i in range(6):
        internal_i = int(protocol_to_internal[proto_i])
        out[internal_i] = float(src[proto_i])
    return out


def lengths_to_strokes(
    current_lengths_internal: Sequence[float],
    home_lengths_internal: Sequence[float],
) -> List[float]:
    """绝对杆长 -> 相对零位行程 ΔL（抓包回放相对零位用）。"""
    return [
        float(current_lengths_internal[i]) - float(home_lengths_internal[i])
        for i in range(6)
    ]


def pulses_to_strokes_internal(
    pulses_protocol: Sequence[int],
    formula: CylinderFormula,
    protocol_to_internal: Sequence[int],
) -> List[float]:
    """
    协议脉冲 -> 内部电缸行程 ΔL（UDP 正解）。
    ΔL_i = round(P÷N÷S×D−offset, 4)，再按 protocol_to_internal 映射。
    """
    strokes_proto = [formula.stroke_from_pulse(int(p)) for p in pulses_protocol]
    return protocol_lengths_to_internal(strokes_proto, protocol_to_internal)


def absolute_lengths_from_strokes(
    strokes_internal: Sequence[float],
    L0_internal: Sequence[float],
) -> List[float]:
    """几何零位杆长 L0 + 行程 ΔL -> 当前杆长。"""
    s = np.asarray(strokes_internal, dtype=float)
    l0 = np.asarray(L0_internal, dtype=float)
    return [round(float(l0[i] + s[i]), 4) for i in range(6)]
