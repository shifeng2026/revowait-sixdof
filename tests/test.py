"""
test.py - 使用 set/get 接口的演示程序
"""

import os
import sys
_src = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src')
if _src not in sys.path:
    sys.path.insert(0, _src)

from interface import SixAxisPlatform
import time


def demo(ip=None, port=8080):
    plat = SixAxisPlatform()

    # ── connect ──────────────────────────────────────────────
    ok = plat.set("connect", ip, port=port)
    if not ok:
        print(f"connect failed: {plat.get('last_error')}")
        return

    print(f"connected: {plat.get('connected')}")

    # ── read pose ─────────────────────────────────────────────
    raw = plat.get("pose_deg")
    print(f"initial pose_deg: {raw}")

    # ── S-curve move ─────────────────────────────────────────
    print("move_pose_s_curve [0,0,0,0,0,5] 1s")
    ok = plat.set("move_pose_s_curve", [0, 0, 100, 0, 0, 19], duration=1.0)
    print(f"s_curve returned: {ok}")

    # ── poll pose for 3 seconds ──────────────────────────────
    print("polling pose ...")
    for _ in range(30):
        raw = plat.get("pose_deg")
        if raw is not None:
            print(f"  {[round(x, 3) for x in raw]}")
        time.sleep(0.1)

    # ── disconnect ──────────────────────────────────────────
    plat.set("disconnect")
    print("done")


if __name__ == "__main__":
    demo()
