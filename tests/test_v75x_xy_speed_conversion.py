"""test_v75x_xy_speed_conversion.py — mm/s ↔ SMS-% conversion uses the REAL max.

Root cause of "XY moves ~2.8x slower than commanded": on real hardware
``_protocol_max_speed_um_s`` was set only in the simulator branch, so
``set_speed_mm_s`` fell back to a hardcoded 50000 µm/s assumption — sending e.g.
``SMS,2`` for 1 mm/s against an unverified 50 mm/s top speed. The fix resolves
the top speed from a measured override → sim value → protocol ``max_speed`` →
50000 default, and exposes ``set_max_speed_um_s`` to set the real value.
"""

import unittest
from types import SimpleNamespace

from SupportClasses.XYStage import XYStageManager


def _xy():
    xy = XYStageManager.__new__(XYStageManager)
    sent = {}
    xy._send_protocol_command = (
        lambda name, fallback_cmd=None, **kw: sent.update(name=name, **kw))
    xy._protocol = None
    return xy, sent


class TestMaxSpeedResolution(unittest.TestCase):
    def test_default_when_nothing_known(self):
        xy, _ = _xy()
        self.assertEqual(xy._max_speed_um_s(), 50000.0)

    def test_protocol_param_used(self):
        xy, _ = _xy()
        xy._protocol = SimpleNamespace(
            _config={"parameters": {"max_speed": 18000}})
        self.assertEqual(xy._max_speed_um_s(), 18000.0)

    def test_sim_override_beats_protocol(self):
        xy, _ = _xy()
        xy._protocol = SimpleNamespace(
            _config={"parameters": {"max_speed": 18000}})
        xy._protocol_max_speed_um_s = 20000.0      # sim branch value
        self.assertEqual(xy._max_speed_um_s(), 20000.0)

    def test_measured_override_wins(self):
        xy, _ = _xy()
        xy._protocol_max_speed_um_s = 20000.0
        xy.set_max_speed_um_s(33000.0)             # measured/configured truth
        self.assertEqual(xy._max_speed_um_s(), 33000.0)


class TestSpeedToSMS(unittest.TestCase):
    def test_1mmps_against_50mmps_max(self):
        xy, sent = _xy()                           # default 50000 µm/s
        xy.set_speed_mm_s(1.0)
        self.assertEqual(sent["speed"], 2)         # 1000/50000*100 = 2%

    def test_1mmps_against_measured_18mmps_max(self):
        xy, sent = _xy()
        xy.set_max_speed_um_s(18000.0)
        xy.set_speed_mm_s(1.0)                      # 1000/18000*100 = 5.56 → 6%
        self.assertEqual(sent["speed"], 6)

    def test_rounds_not_truncates(self):
        # 2.9 mm/s vs 50 mm/s = 5.8% → should round to 6, not truncate to 5
        xy, sent = _xy()
        xy.set_speed_mm_s(2.9)
        self.assertEqual(sent["speed"], 6)

    def test_set_velocity_um_s_uses_real_max(self):
        xy, sent = _xy()
        xy.set_max_speed_um_s(18000.0)
        xy.set_velocity(18000)                      # µm/s (>100) → 100%
        self.assertEqual(sent["speed"], 100)


if __name__ == "__main__":
    unittest.main()
