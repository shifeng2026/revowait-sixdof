# 六轴平台控制接口

`SixAxisPlatform` — 通过 `set()` / `get()` 命令接口控制六轴并联平台。

详见 [docs/index.md](docs/index.md) 和 [docs/api.md](docs/api.md)。

## 快速安装

```bash
pip install numpy matplotlib
```

## 一句话示例

```python
from interface import SixAxisPlatform

plat = SixAxisPlatform()
plat.set("connect")
plat.set("move_pose_s_curve", [0, 0, 100, 0, 0, 0], duration=2.0)
print(plat.get("pose_raw"))
plat.set("disconnect")
```
