"""
v7.5.x tests — ZP last-known-position save/restore.

Marlin (ZP) loses its position on power cycle. To recover, the app saves the
last-known ZP position (zero-ref mm) on clean shutdown and, on the next ZP
connect, offers to re-stamp the firmware counter with it. These tests cover the
backend pieces that feature relies on:

  1. ``StageController.get_zp_position_zero_ref`` — the saved frame
     (displayed = raw - zero_position), axis_map-aware.
  2. Save -> restore reproduces the original firmware *raw* counter, even when
     a zero reference is in play (snapshot in zero-ref, override adds it back).
  3. ``StageController.on_connect`` fires with "ZP" on a successful connect.

The GUI prompt itself (QMessageBox) is exercised manually; here we lock down the
backend contract it stands on.
"""

import unittest

from SupportClasses.StageController import StageController
from SupportClasses.SafetyLimits import SafetyLimits

DEFAULT_MAP = {"Z": "X", "P1": "Y", "P2": "Z", "P3": "E"}
ME3B_V1_MAP = {"Z": "Z", "P1": "X", "P2": "Y", "P3": "E"}


class _FakeZP:
    """ZPStageManager stand-in: axis_map + records set_position calls."""

    def __init__(self, axis_map, simulate=False):
        self.axis_map = dict(axis_map)
        self.simulate = simulate
        self.last_set = None
        self.set_return = True

    def set_position(self, logical_axis, value_mm):
        self.last_set = (logical_axis, value_mm)
        return self.set_return


def _stop(ctrl):
    try:
        ctrl._pos_poller.stop()
    except Exception:
        pass


class TestZeroRefSnapshot(unittest.TestCase):
    def setUp(self):
        self.ctrl = StageController(simulate_xy=True, simulate_zp=True)
        self.addCleanup(_stop, self.ctrl)

    def _attach(self, axis_map, physical_tuple):
        self.ctrl.zp_stage = _FakeZP(axis_map)
        self.ctrl.get_zp_position = lambda cached=True: physical_tuple

    def test_zero_ref_subtracts_zero_position(self):
        # Default map: Z->X(0), P1->Y(1), P2->Z(2), P3->E(3).
        self._attach(DEFAULT_MAP, (10.0, 2.0, 3.0, 4.0))
        self.ctrl.zero_position.update({"Z": 1.0, "P1": 2.0, "P2": 0.0, "P3": 0.5})
        zr = self.ctrl.get_zp_position_zero_ref()
        self.assertAlmostEqual(zr["Z"], 9.0, places=6)    # 10 - 1
        self.assertAlmostEqual(zr["P1"], 0.0, places=6)   # 2 - 2
        self.assertAlmostEqual(zr["P2"], 3.0, places=6)   # 3 - 0
        self.assertAlmostEqual(zr["P3"], 3.5, places=6)   # 4 - 0.5

    def test_zero_ref_respects_axis_map(self):
        # ME3B V1: Z->Z(idx 2). Real Z value sits at tuple index 2.
        self._attach(ME3B_V1_MAP, (80.0, 0.0, 12.0, 0.0))
        self.ctrl.zero_position.update({"Z": 2.0})
        zr = self.ctrl.get_zp_position_zero_ref()
        self.assertAlmostEqual(zr["Z"], 10.0, places=6)   # idx 2 (12) - 2

    def test_none_slot_is_none(self):
        self._attach(DEFAULT_MAP, (None, None, None, None))
        zr = self.ctrl.get_zp_position_zero_ref()
        self.assertIsNone(zr["Z"])


class TestRestoreReproducesRaw(unittest.TestCase):
    """The save→restore contract: snapshot in zero-ref, then override adds the
    zero reference back, landing the firmware counter on the original raw."""

    def setUp(self):
        self.ctrl = StageController(simulate_xy=True, simulate_zp=True)
        self.addCleanup(_stop, self.ctrl)
        self.ctrl.safety_limits = SafetyLimits(enabled=True)

    def test_snapshot_then_override_restores_raw(self):
        raw = (10.0, 2.0, 3.0, 4.0)  # firmware raw before "shutdown"
        self.ctrl.zp_stage = _FakeZP(DEFAULT_MAP)
        self.ctrl.get_zp_position = lambda cached=True: raw
        self.ctrl.zero_position.update({"Z": 1.0, "P1": 2.0, "P2": 0.0, "P3": 0.5})

        # Save (what save_settings persists): zero-ref snapshot.
        snap = self.ctrl.get_zp_position_zero_ref()

        # Restart: firmware now at 0, but zero_position persisted (same dict).
        # Apply the restore for the Z axis and confirm the override would set
        # the firmware counter back to the original raw value.
        res = self.ctrl.override_zp_position("Z", snap["Z"])
        self.assertTrue(res["ok"])
        self.assertAlmostEqual(res["raw"], raw[0], places=6)        # back to 10.0
        self.assertEqual(self.ctrl.zp_stage.last_set, ("Z", raw[0]))

    def test_restore_all_axes_reproduces_each_raw(self):
        raw = {"Z": 7.0, "P1": -1.5, "P2": 0.0, "P3": 3.25}
        # zero_position all zero → zero-ref == raw → override sets raw == value.
        phys = (raw["Z"], raw["P1"], raw["P2"], raw["P3"])
        self.ctrl.zp_stage = _FakeZP(DEFAULT_MAP)
        self.ctrl.get_zp_position = lambda cached=True: phys
        snap = self.ctrl.get_zp_position_zero_ref()
        for ax in ("Z", "P1", "P2", "P3"):
            res = self.ctrl.override_zp_position(ax, snap[ax])
            self.assertTrue(res["ok"])
            self.assertAlmostEqual(res["raw"], raw[ax], places=6)


class TestOnConnectCallback(unittest.TestCase):
    def test_on_connect_fires_zp_on_connect(self):
        ctrl = StageController(simulate_xy=True, simulate_zp=True)
        self.addCleanup(_stop, ctrl)
        seen = []
        ctrl.on_connect = lambda name: seen.append(name)
        ctrl.connect_zp(simulate=True)
        self.addCleanup(lambda: ctrl.disconnect_zp())
        self.assertIn("ZP", seen)

    def test_on_connect_not_fired_for_already_connected(self):
        ctrl = StageController(simulate_xy=True, simulate_zp=True)
        self.addCleanup(_stop, ctrl)
        ctrl.connect_zp(simulate=True)
        self.addCleanup(lambda: ctrl.disconnect_zp())
        seen = []
        ctrl.on_connect = lambda name: seen.append(name)
        # Already connected → the `is None` guard skips reconnect → no fire.
        ctrl.connect_zp(simulate=True)
        self.assertNotIn("ZP", seen)


if __name__ == "__main__":
    unittest.main()
