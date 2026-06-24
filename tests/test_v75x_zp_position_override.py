"""
v7.5.x tests — manual ZP-axis position override (power-cycle recovery).

Marlin (the ZP board) has no absolute encoder: after a power cycle it
powers up reporting 0, so the position readout is wrong and a Refresh
just re-reads the wrong value. The override lets the operator declare
where an axis physically is by sending a Marlin G92 (rebase the counter,
no motion).

Covered here:
  1. ZPStageSimulator now honors G92 (previously a silent no-op) so both
     set_zero and the override are visible in the simulated M114 readout.
  2. ZPStage.set_position sends the correct G92 and respects axis_map.
  3. StageController.override_zp_position computes raw = value +
     zero_position[axis] (preserving the established zero reference) and
     handles the error cases.
  4. End-to-end through a real ZPStageManager(simulate=True).
"""

import tempfile
import time
import unittest
from pathlib import Path

from SupportClasses.StageController import StageController
from SupportClasses.SafetyLimits import SafetyLimits
from SupportClasses.ZPStage import ZPStageManager
from SupportClasses.ZPStageSimulator import ZPStageSimulator

DEFAULT_MAP = {"Z": "X", "P1": "Y", "P2": "Z", "P3": "E"}
ME3B_V1_MAP = {"Z": "Z", "P1": "X", "P2": "Y", "P3": "E"}


class _RecordingSerial:
    """Captures the strings written to a (fake) serial port."""

    def __init__(self):
        self.writes: list[str] = []
        self.is_open = True

    def write(self, data: bytes) -> None:
        self.writes.append(data.decode("utf-8").strip())

    def flush(self) -> None:
        pass

    def readline(self) -> bytes:
        # v7.5.x flow control: send_data waits for 'ok' — ack each command.
        return b"ok\n"

    def close(self) -> None:
        self.is_open = False


class _FakeZP:
    """Minimal ZPStageManager stand-in: carries an axis_map and records
    the last set_position call."""

    def __init__(self, axis_map):
        self.axis_map = dict(axis_map)
        self.last_set = None
        self.set_return = True

    def set_position(self, logical_axis, value_mm):
        self.last_set = (logical_axis, value_mm)
        return self.set_return


# ──────────────────────────────────────────────────────────────────────
# 1. Simulator honors G92
# ──────────────────────────────────────────────────────────────────────
class TestSimulatorG92(unittest.TestCase):
    def setUp(self):
        # Isolate from any persisted sim state file — point at a path
        # that does not exist so the sim starts from clean defaults
        # (and no spurious "failed to load state" JSON-decode warning).
        self._tmpdir = tempfile.mkdtemp()
        self.sim = ZPStageSimulator(
            state_file=Path(self._tmpdir) / "nonexistent_state.json")

    def tearDown(self):
        try:
            import shutil
            shutil.rmtree(self._tmpdir, ignore_errors=True)
        except Exception:
            pass

    def test_g92_rebases_named_axis_without_moving(self):
        # _execute_command acquires the lock and dispatches — no threads.
        self.sim._execute_command("G92 X12.5")
        report = self.sim._execute_command("M114")
        self.assertIn("X:12.5000", report)
        # G92 does not enqueue motion: position == target.
        self.assertEqual(self.sim.position["X"], self.sim.target_position["X"])

    def test_g92_zero_form_resets_to_zero(self):
        self.sim._execute_command("G92 Y8.0")
        self.assertEqual(self.sim.position["Y"], 8.0)
        self.sim._execute_command("G92 Y0")  # the set_zero form
        self.assertEqual(self.sim.position["Y"], 0.0)

    def test_g92_only_touches_named_axes(self):
        self.sim._execute_command("G92 Z3.0")
        self.assertEqual(self.sim.position["Z"], 3.0)
        self.assertEqual(self.sim.position["X"], 0.0)
        self.assertEqual(self.sim.position["E"], 0.0)


# ──────────────────────────────────────────────────────────────────────
# 2. ZPStage.set_position
# ──────────────────────────────────────────────────────────────────────
class TestZPStageSetPosition(unittest.TestCase):
    def _make(self, axis_map):
        zp = ZPStageManager.__new__(ZPStageManager)  # skip __init__/serial
        zp.axis_map = dict(axis_map)
        zp.simulate = False  # so __del__ → stop() takes the serial.close() path
        import threading
        zp._serial_lock = threading.Lock()
        zp.serial = _RecordingSerial()
        return zp

    def test_sends_g92_to_mapped_physical_axis(self):
        zp = self._make(DEFAULT_MAP)
        self.assertTrue(zp.set_position("Z", 5.25))  # Z -> X
        self.assertIn("G92 X5.2500", zp.serial.writes)

    def test_respects_non_default_axis_map(self):
        zp = self._make(ME3B_V1_MAP)
        self.assertTrue(zp.set_position("Z", 5.25))  # Z -> Z under ME3B V1
        self.assertIn("G92 Z5.2500", zp.serial.writes)
        self.assertTrue(zp.set_position("P1", -2.0))  # P1 -> X
        self.assertIn("G92 X-2.0000", zp.serial.writes)

    def test_unmapped_axis_returns_false(self):
        zp = self._make({"Z": "X"})  # P1 missing
        self.assertFalse(zp.set_position("P1", 1.0))


# ──────────────────────────────────────────────────────────────────────
# 3. StageController.override_zp_position
# ──────────────────────────────────────────────────────────────────────
class TestOverrideController(unittest.TestCase):
    def setUp(self):
        self.ctrl = StageController(simulate_xy=True, simulate_zp=True)
        self.ctrl.safety_limits = SafetyLimits(enabled=True)

    def tearDown(self):
        try:
            self.ctrl._pos_poller.stop()
        except Exception:
            pass

    def _attach(self, axis_map, physical_tuple):
        self.ctrl.zp_stage = _FakeZP(axis_map)
        self.ctrl.get_zp_position = lambda cached=True: physical_tuple

    def test_raw_is_value_plus_zero_reference(self):
        # zero ref of 2.0 mm preserved → raw counter set to value + zero.
        self._attach(DEFAULT_MAP, (0.0, 0.0, 0.0, 0.0))
        self.ctrl.zero_position["Z"] = 2.0
        res = self.ctrl.override_zp_position("Z", 5.0)
        self.assertTrue(res["ok"])
        self.assertAlmostEqual(res["raw"], 7.0, places=6)
        # The G92 went to the physical letter mapped to Z, value 7.0.
        self.assertEqual(self.ctrl.zp_stage.last_set, ("Z", 7.0))
        # Zero reference must be preserved (NOT reset to 0 like Set Zero).
        self.assertEqual(self.ctrl.zero_position["Z"], 2.0)

    def test_zero_reference_zero_gives_raw_equals_value(self):
        self._attach(DEFAULT_MAP, (0.0, 0.0, 0.0, 0.0))
        self.ctrl.zero_position["P1"] = 0.0
        res = self.ctrl.override_zp_position("P1", -3.5)
        self.assertTrue(res["ok"])
        self.assertAlmostEqual(res["raw"], -3.5, places=6)

    def test_previous_raw_reported(self):
        # ME3B V1: Z->Z (index 2). previous_raw should read that slot.
        self._attach(ME3B_V1_MAP, (80.0, 0.0, 10.0, 0.0))
        res = self.ctrl.override_zp_position("Z", 1.0)
        self.assertTrue(res["ok"])
        self.assertAlmostEqual(res["previous_raw"], 10.0, places=6)

    def test_unsupported_axis_rejected(self):
        self._attach(DEFAULT_MAP, (0.0, 0.0, 0.0, 0.0))
        res = self.ctrl.override_zp_position("X", 1.0)
        self.assertFalse(res["ok"])
        self.assertIn("unsupported", res["error"])

    def test_not_connected_rejected(self):
        self.ctrl.zp_stage = None
        res = self.ctrl.override_zp_position("Z", 1.0)
        self.assertFalse(res["ok"])
        self.assertIn("not connected", res["error"])

    def test_unmapped_axis_propagates_failure(self):
        self._attach({"Z": "X"}, (0.0, 0.0, 0.0, 0.0))  # P1 unmapped
        self.ctrl.zp_stage.set_return = False
        res = self.ctrl.override_zp_position("P1", 1.0)
        self.assertFalse(res["ok"])


# ──────────────────────────────────────────────────────────────────────
# 4. End-to-end through a real simulated ZPStageManager
# ──────────────────────────────────────────────────────────────────────
class TestOverrideEndToEnd(unittest.TestCase):
    def setUp(self):
        self.zp = ZPStageManager(simulate=True)

    def tearDown(self):
        try:
            self.zp.stop()
        except Exception:
            pass

    def _read_axis(self, idx, expected, tries=50):
        """Poll M114 until the slot reaches ``expected`` (sim is threaded)."""
        last = None
        for _ in range(tries):
            last = self.zp.get_current_position()[idx]
            if abs(last - expected) < 1e-3:
                return last
            time.sleep(0.02)
        return last

    def test_set_position_then_m114_reports_value(self):
        # Default map: Z -> X (tuple index 0).
        self.assertTrue(self.zp.set_position("Z", 7.0))
        got = self._read_axis(0, 7.0)
        self.assertAlmostEqual(got, 7.0, places=2)

    def test_set_zero_now_actually_zeroes_in_sim(self):
        self.zp.set_position("Z", 4.0)
        self.assertAlmostEqual(self._read_axis(0, 4.0), 4.0, places=2)
        self.zp.set_zero("Z")  # G92 X0
        self.assertAlmostEqual(self._read_axis(0, 0.0), 0.0, places=2)


if __name__ == "__main__":
    unittest.main()
