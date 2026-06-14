# API 参考

## 位姿单位

所有反馈与输入的位姿均为 **mm（平移）** 与 **°（姿态/角度）**。

| 轴 | 含义 | 范围（相对中位） |
|----|------|------------------|
| x  | X 方向平移 | ±200 mm |
| y  | Y 方向平移 | ±200 mm |
| z  | Z 方向平移 | ±200 mm |
| rx | 横摇 (roll) | ±25° |
| ry | 俯仰 (pitch) | ±25° |
| rz | 偏航 (yaw) | ±28° |

超出上述范围时命令会被拒绝并返回 `False`，同时记录错误日志。

---

## `set(key, value=None, **kwargs)`

统一命令接口。

| key | value | kwargs | 说明 |
|-----|-------|--------|------|
| `connect` | `str` IP 地址（可选） | `port=` 端口号 | 连接平台。不传 IP 则使用配置文件 |
| `disconnect` | — | — | 断开连接 |
| `read_pose` | — | — | 读取当前位姿，返回 dict |
| `pose` | `list[6]` 相对中位增量 | — | 设定位姿 [x,y,z,rx,ry,rz]（单位 mm/°） |
| `move_pose_s_curve` | `list[6]` 目标位姿 | `duration=` 总时长(秒) | S 曲线插补到目标，输入范围受轴限位约束 |
| `move_to_mid` | — | — | 移动到中位 |
| `move_to_home` | — | — | 回原点 |

**返回值**: `bool` / `dict` / `None`。命令成功返回 `True`，`read_pose` 返回位姿 dict，未知 key 返回 `None`。

### 示例

```python
# 连接
plat.set("connect", "192.168.31.88", port=8080)

# 设定位姿
plat.set("pose", [0, 0, 100, 5, 0, 0])

# S 曲线运动
plat.set("move_pose_s_curve", [50, 0, 100, 0, 0, 0], duration=3.0)

# 读取位姿
data = plat.set("read_pose")
print(data["pose_raw"])

# 断开
plat.set("disconnect")
```

---

## `get(key)`

统一查询接口。

| key | 返回值 | 说明 |
|-----|--------|------|
| `pose` | `dict` / `None` | 同 `read_current_pose()` |
| `pose_raw` | `list[6]` / `None` | 绝对位姿 [x,y,z,rx,ry,rz]（平移 mm，姿态 rad） |
| `pose_deg` | `list[6]` / `None` | 相对中位增量 [x,y,z,rx,ry,rz]（平移 mm，姿态 °） |
| `connected` | `bool` | 连接状态 |
| `last_error` | `str` / `None` | 最近错误信息 |

### 示例

```python
if plat.get("connected"):
    raw = plat.get("pose_raw")
    print(raw)

err = plat.get("last_error")
```
