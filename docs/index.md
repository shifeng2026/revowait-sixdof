# 六轴平台控制接口

提供 `set()` / `get()` 统一命令接口，通过 `SixAxisPlatform` 类操作六轴平台。

## 快速开始

```python
from interface import SixAxisPlatform

plat = SixAxisPlatform()
plat.set("connect")
print(plat.get("connected"))      # True

plat.set("move_pose_s_curve", [0, 0, 100, 0, 0, 0], duration=2.0)
print(plat.get("pose_raw"))       # 当前位姿

plat.set("disconnect")
```

## 依赖

- Python ≥ 3.10
- `numpy`
- `matplotlib`（可选，用于实时曲线）

## 位姿单位

所有位姿的平移为 **mm**，姿态为 **°（度）**。

## 轴限位（相对中位增量）

| 轴 | 范围 |
|----|------|
| x / y / z | ±200 mm |
| rx (roll) | ±25° |
| ry (pitch) | ±25° |
| rz (yaw) | ±28° |

超出限位的目标位姿会被拒绝并返回错误。

## 配置

默认读取 `platform_config.json`，包含 UDP 监听地址、平台 IP/端口、电缸参数等。

## 项目结构

```
.
├── platform_config.json     # 配置文件
├── README.md
├── src/
│   ├── __init__.py
│   └── interface.py         # SixAxisPlatform 核心类
├── tests/
│   ├── __init__.py
│   └── test.py              # 使用示例
└── docs/
    ├── index.md             # 本文档
    └── api.md               # API 参考
```
