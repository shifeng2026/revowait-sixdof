"""
test.py - 使用 set/get 接口的演示程序，每次执行记录位姿到 logs/ 目录
"""

import os
import sys
import threading
import time
from datetime import datetime

_src = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src')
if _src not in sys.path:
    sys.path.insert(0, _src)

from interface import SixAxisPlatform


_log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'logs')
os.makedirs(_log_dir, exist_ok=True)

_existing = [int(f.replace('.log', '')) for f in os.listdir(_log_dir)
             if f.replace('.log', '').isdigit()]
_next_num = max(_existing) + 1 if _existing else 1
_log_path = os.path.join(_log_dir, f"{_next_num}.log")

_log_file = open(_log_path, 'w', encoding='utf-8')
_log_file.write(f"# pose log started at {datetime.now():%Y-%m-%d %H:%M:%S}\n")
_log_file.flush()
print(f"pose log -> {_log_path}")


keep_running = True
_plat_ref = [None]


def _read_loop(interval=0.1):
    while keep_running:
        p = _plat_ref[0]
        if p is not None:
            lp = p.get("pose_deg")
            if lp:
                ts = time.time()
                line = f"{ts:.3f}, " + ", ".join(f"{v:.4f}" for v in lp)
                _log_file.write(line + "\n")
                _log_file.flush()
                print(f"Pose: {[f'{v:.1f}' for v in lp]}", end="\r", flush=True)
        time.sleep(interval)


t = threading.Thread(target=_read_loop, args=(0.05,), daemon=True)
t.start()


def demo(ip=None, port=8080):
    plat = SixAxisPlatform()
    _plat_ref[0] = plat

    ok = plat.set("connect", ip, port=port)
    if not ok:
        print(f"connect failed: {plat.get('last_error')}")
        return

    print(f"connected: {plat.get('connected')}")
    raw = plat.get("pose_deg")
    print(f"initial pose_deg: {raw}")

    target = [100, 0, 0, 0, 0, 0]
    _log_file.write(f"# target: " + ", ".join(f"{v}" for v in target) + "\n")
    _log_file.write("# timestamp, x, y, z, rx, ry, rz\n")
    _log_file.flush()

    print(f"move_pose_s_curve {target} 5s")
    ok = plat.set("move_pose_s_curve", target, duration=5)
    print(f"s_curve returned: {ok}")

    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        global keep_running
        keep_running = False
        plat.set("disconnect")
        _log_file.close()
        print("\ndone")


if __name__ == "__main__":
    demo()
