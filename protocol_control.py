"""
铂沃平台控制命令报文构建（通信协议 V1.0.5 八字节命令）。

八字节：CRC16-MODBUS，[6]=CRC 高字节，[7]=CRC 低字节。

轴点动 CMD=0x10：byte2=轴号，byte3=增量(UINT8 mm)，byte4=方向，byte5=0
位姿点动 CMD=0x11：byte2=轴号，byte3=位置增量 mm，byte4=姿态增量 °，byte5=方向

八字节点动单位为整 mm / 整 °（1~255）。界面可输入小数，发送时四舍五入为整数。
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import List, Sequence, Tuple

from protocol_udp import crc16_modbus

FRAME_HEAD = 0xA5
DIR_POSITIVE = 0x0E
DIR_NEGATIVE = 0x0F

CMD_GET_INFO = 0x00
CMD_AXIS_JOG = 0x10
CMD_POSE_JOG = 0x11
CMD_HARMONIC = 0x16
CMD_PLAT_RESET = 0x77
CMD_PLAT_MID = 0x78
CMD_PLAT_TOP = 0x79
CMD_PLAT_STOP = 0x80
CMD_PLAT_CONTINUE = 0x81
CMD_POSE_FOLLOW = 0x20
CMD_AXIS_FOLLOW = 0x21
CMD_JOG = CMD_AXIS_JOG  # 兼容旧名

HARMONIC_PKT_LEN = 104
FOLLOW_PKT_LEN = 29
DEFAULT_FOLLOW_SPEED_LEVEL = 3
AXIS_NAMES = ("X", "Y", "Z", "A", "B", "C")
POSE_AXIS_POS_MAX = 100.0
POSE_AXIS_ATT_MAX = 10.0
# 位姿轴 1~6 (X,Y,Z,A,B,C) -> 0x20 帧内 float 槽位：Z,A,B,C,X,Y
_POSE_AXIS_TO_SLOT = [4, 5, 0, 1, 2, 3]

def append_crc(payload: bytes) -> bytes:
    if len(payload) < 2:
        raise ValueError("报文过短")
    hi, lo = crc16_modbus(payload)
    return payload + bytes([hi, lo])


def build_cmd8(cmd: int, d0: int, d1: int, d2: int, d3: int) -> bytes:
    body = bytes(
        [
            FRAME_HEAD,
            cmd & 0xFF,
            d0 & 0xFF,
            d1 & 0xFF,
            d2 & 0xFF,
            d3 & 0xFF,
        ]
    )
    return append_crc(body)


def _jog_direction(positive: bool) -> int:
    return DIR_POSITIVE if positive else DIR_NEGATIVE


def _jog_inc_byte(magnitude: float, *, label: str = "点动增量") -> int:
    """八字节协议：UINT8 整 mm 或整 °。"""
    mag = abs(float(magnitude))
    if mag <= 0:
        raise ValueError(f"{label}须 > 0")
    inc = int(round(mag))
    if inc <= 0:
        raise ValueError(f"{label}四舍五入后须 ≥ 1 mm/°")
    if inc > 255:
        raise ValueError(f"{label}不能超过 255 mm/°")
    return inc



# ──────────────────────────────────────────────
# 29 字节随动协议辅助函数
# ──────────────────────────────────────────────

def _write_float_le(buf: bytearray, offset: int, value: float) -> None:
    struct.pack_into("<f", buf, offset, float(value))


def quantize_follow_value(value: float) -> float:
    """协议精度 0.0001：量化到小数点后 4 位。"""
    return round(float(value), 4)


def _signed_quantized_float(delta: float) -> float:
    mag = quantize_follow_value(abs(delta))
    return mag if delta >= 0 else -mag


def _doubles_to_follow_floats(values: Sequence[float]) -> List[float]:
    return [float(quantize_follow_value(v)) for v in values]


def _build_follow_29(cmd: int, values: Sequence[float],
                     speed_level: int = DEFAULT_FOLLOW_SPEED_LEVEL) -> bytes:
    """
    29 字节随动帧核心构建。

    偏移  长度  内容
    ────────────────────────────────
    0     1     帧头 0xA5
    1     1     命令字 (0x20 / 0x21)
    2~25  24    6 个 float (LE) 轴数据
    26    1     速度等级
    27~28 2     CRC16 Modbus (大端在前)
    """
    buf = bytearray(FOLLOW_PKT_LEN)
    buf[0] = FRAME_HEAD
    buf[1] = cmd & 0xFF
    for i in range(6):
        _write_float_le(buf, 2 + i * 4, float(values[i]) if i < len(values) else 0.0)
    buf[26] = speed_level & 0xFF
    hi, lo = crc16_modbus(bytes(buf[:27]))
    buf[27] = hi
    buf[28] = lo
    return bytes(buf)


# ──────────────────────────────────────────────
# 8 字节命令（保留原始实现）
# ──────────────────────────────────────────────

def build_axis_jog_legacy(axis_1based: int, inc_mm: int, positive: bool) -> bytes:
    """整毫米轴点动8字节（CMD=0x10）。"""
    inc = max(1, min(255, int(inc_mm)))
    direction = DIR_POSITIVE if positive else DIR_NEGATIVE
    return build_cmd8(CMD_AXIS_JOG, int(axis_1based), inc, direction, 0)


def build_pose_jog_legacy(
    axis_1based: int, pos_inc: int, att_inc: int, positive: bool
) -> bytes:
    direction = DIR_POSITIVE if positive else DIR_NEGATIVE
    return build_cmd8(
        CMD_POSE_JOG,
        int(axis_1based),
        max(0, min(255, int(pos_inc))),
        max(0, min(255, int(att_inc))),
        direction,
    )


def build_plat_reset() -> bytes:
    return build_cmd8(CMD_PLAT_RESET, 0, 0, 0, 0)


def build_plat_mid() -> bytes:
    return build_cmd8(CMD_PLAT_MID, 0, 0, 0, 0)


def build_plat_top() -> bytes:
    return build_cmd8(CMD_PLAT_TOP, 0, 0, 0, 0)


def build_plat_stop() -> bytes:
    return build_cmd8(CMD_PLAT_STOP, 0, 0, 0, 0)


def build_plat_continue() -> bytes:
    return build_cmd8(CMD_PLAT_CONTINUE, 0, 0, 0, 0)


def build_get_info() -> bytes:
    return build_cmd8(CMD_GET_INFO, 0, 0, 0, 0)

# ──────────────────────────────────────────────
# 29 字节随动协议（六轴随动控制）
# ──────────────────────────────────────────────

def build_axis_jog(axis_1based: int, delta_mm: float) -> bytes:
    """
    电缸轴点动，使用 29 字节 0x21 随动协议。
    仅目标轴有非零伸长量，其余轴为 0。
    """
    if not 1 <= int(axis_1based) <= 6:
        raise ValueError("电缸轴号应为 1~6")
    mag = abs(float(delta_mm))
    if mag <= 0:
        raise ValueError("轴点动增量须 > 0")

    lengths = [0.0] * 6
    lengths[axis_1based - 1] = _signed_quantized_float(delta_mm)
    return _build_follow_29(CMD_AXIS_FOLLOW, lengths)


def build_pose_jog(axis_1based: int, delta: float, *, is_attitude: bool = False) -> bytes:
    """
    位姿点动，使用 29 字节 0x20 随动协议。
    axis: 1=X, 2=Y, 3=Z, 4=A, 5=B, 6=C
    帧内槽位重排：Z, A, B, C, X, Y
    """
    if not 1 <= int(axis_1based) <= 6:
        raise ValueError("位姿轴号应为 1~6")
    mag = abs(float(delta))
    if mag <= 0:
        raise ValueError("位姿点动增量须 > 0")

    slot = _POSE_AXIS_TO_SLOT[axis_1based - 1]
    pose = [0.0] * 6
    pose[slot] = _signed_quantized_float(delta)
    return _build_follow_29(CMD_POSE_FOLLOW, pose)


def build_axis_follow(lengths_mm: Sequence[float]) -> bytes:
    """0x21 六轴电缸随动：6 轴累计伸长量 (mm)。"""
    if len(lengths_mm) != 6:
        raise ValueError("需要 6 轴电缸伸长量")
    return _build_follow_29(CMD_AXIS_FOLLOW, _doubles_to_follow_floats(lengths_mm))


def build_pose_follow_xyz_abc(xyzabc: Sequence[float]) -> bytes:
    """
    0x20 六轴位姿随动。
    输入 (X, Y, Z, A, B, C) 累计位移/角度，
    帧内重排为 (Z, A, B, C, X, Y)。
    """
    if len(xyzabc) != 6:
        raise ValueError("需要 6 个位姿值 (X,Y,Z,A,B,C)")
    slots = [xyzabc[2], xyzabc[3], xyzabc[4], xyzabc[5], xyzabc[0], xyzabc[1]]
    return _build_follow_29(CMD_POSE_FOLLOW, _doubles_to_follow_floats(slots))


# ──────────────────────────────────────────────
# 简谐运动（CMD=0x16）
# ──────────────────────────────────────────────

@dataclass
class HarmonicAxisParams:
    amplitude: float = 0.0
    frequency_hz: float = 0.0
    phase_deg: float = 0.0
    bias: float = 0.0


def build_harmonic_motion(
    axes: Sequence[HarmonicAxisParams],
    *,
    duration_s: float = 0.0,
) -> bytes:
    """
    104 字节简谐 / 组合运动（CMD=0x16，通信协议 V1.0.5 第三章）。

    y = A·sin(2πf·t + φ) + B

    [0]=A5 [1]=16
    [2..25]   X~C 六轴幅值 A（float LE）
    [26..49]  六轴频率 f（Hz）
    [50..73]  六轴相位 φ（°）
    [74..97]  六轴偏置 B
    [98..101] 运动时间（s，float）
    [102..103] CRC16（前 102 字节）
    """
    if len(axes) != 6:
        raise ValueError("需要 6 轴简谐参数")
    buf = bytearray(HARMONIC_PKT_LEN)
    buf[0] = FRAME_HEAD
    buf[1] = CMD_HARMONIC
    for i, ax in enumerate(axes):
        _write_float_le(buf, 2 + i * 4, ax.amplitude)
        _write_float_le(buf, 26 + i * 4, ax.frequency_hz)
        _write_float_le(buf, 50 + i * 4, ax.phase_deg)
        _write_float_le(buf, 74 + i * 4, ax.bias)
    _write_float_le(buf, 98, duration_s)
    hi, lo = crc16_modbus(bytes(buf[:102]))
    buf[102] = hi
    buf[103] = lo
    return bytes(buf)


def format_packet_hex(data: bytes) -> str:
    return " ".join(f"{b:02X}" for b in data)


def ui_cylinder_to_protocol_axis(
    ui_axis_1based: int, protocol_to_internal: Sequence[int]
) -> int:
    """界面电缸 1~6 -> 协议电机轴 1~6。"""
    from calibration import UI_TO_INTERNAL

    internal = int(UI_TO_INTERNAL[int(ui_axis_1based) - 1])
    for proto_i, int_i in enumerate(protocol_to_internal):
        if int(int_i) == internal:
            return proto_i + 1
    return int(ui_axis_1based)
