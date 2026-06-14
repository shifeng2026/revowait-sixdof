import os
import sys

_src = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src")
if _src not in sys.path:
    sys.path.insert(0, _src)

from interface import SixAxisPlatform


def test_instantiate():
    config = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "platform_config.json")
    plat = SixAxisPlatform(config)
    assert plat._platform is not None
    assert plat.last_error is None
