"""
v7.5.x — PrintExecutionLogger tests.

Covers:
- Logger lifecycle (start → events → stop) producing valid JSONL
- xy_cmd_fields clamp detection + lag-target tracking
- Background sampler emitting 'sample' events with lag_um
- manifest_for_job command-plan extraction + path stats
- PrintManager integration: discrete job auto-creates the log with
  command/path/settle/job_end events
- Settle-timeout capture
- Global disable flag
"""

import json
import sys
import time
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from SupportClasses.PrintExecutionLogger import PrintExecutionLogger


# ── Fakes ────────────────────────────────────────────────────────────

class FakeXYStage:
    def __init__(self):
        self.speeds = []

    def set_speed_mm_s(self, v):
        self.speeds.append(v)


class FakeLimits:
    """Safety limits that clamp X to <= 1000 µm."""
    enabled = True

    def clamp_xy(self, x_um, y_um):
        return min(x_um, 1000.0), y_um


class FakeController:
    """Instantly-arriving stage: position == last commanded target."""

    def __init__(self, limits=None):
        self.zero_position = {"x": 0.0, "y": 0.0, "Z": 0.0,
                              "P1": 0.0, "P2": 0.0, "P3": 0.0}
        self.is_xy_connected = True
        self.is_zp_connected = False
        self.simulate = True
        self.xy_stage = FakeXYStage()
        self.zp_stage = None
        self.safety_limits = limits or SimpleNamespace(enabled=False)
        self.position_logger = None
        self._pos = (0.0, 0.0)
        self.xy_targets = []
        self.pump_calls = []

    def move_xy_absolute(self, x, y, from_zero_ref=True, fast=False):
        self.xy_targets.append((x, y))
        self._pos = (x * 1000.0 + self.zero_position["x"],
                     y * 1000.0 + self.zero_position["y"])

    def get_xy_position(self, cached=True):
        return self._pos

    def get_zp_position_logical_tuple(self, cached=True):
        return (0.0, 0.0, 0.0, 0.0)

    def move_z_absolute(self, z, from_zero_ref=True, feedrate_mm_min=None):
        pass

    def move_pump_uL(self, pump, vol, rate=None):
        self.pump_calls.append((pump, vol, rate))


def read_events(path):
    with open(path, encoding="utf-8") as f:
        return [json.loads(line) for line in f if line.strip()]


# ── Logger unit tests ────────────────────────────────────────────────

class TestLoggerLifecycle(unittest.TestCase):

    def test_start_log_stop_jsonl(self):
        ctrl = FakeController()
        with TemporaryDirectory() as td:
            lg = PrintExecutionLogger("My Job", "discrete", log_dir=td)
            path = lg.start(ctrl, {"hello": "world"})
            self.assertIsNotNone(path)
            lg.log("xy_cmd", x_mm=1.0, y_mm=2.0)
            lg.stop("completed")
            self.assertFalse(lg.active)

            events = read_events(path)
            self.assertEqual(events[0]["ev"], "job_start")
            self.assertEqual(events[0]["hello"], "world")
            self.assertEqual(events[0]["job_name"], "My Job")
            kinds = [e["ev"] for e in events]
            self.assertIn("xy_cmd", kinds)
            self.assertEqual(events[-1]["ev"], "job_end")
            self.assertEqual(events[-1]["status"], "completed")
            # Every event carries a monotonic-relative timestamp
            self.assertTrue(all("t" in e for e in events))

    def test_stop_is_idempotent(self):
        ctrl = FakeController()
        with TemporaryDirectory() as td:
            lg = PrintExecutionLogger("j", log_dir=td)
            path = lg.start(ctrl, {})
            lg.stop("completed")
            lg.stop("error")  # second stop ignored
            events = read_events(path)
            ends = [e for e in events if e["ev"] == "job_end"]
            self.assertEqual(len(ends), 1)
            self.assertEqual(ends[0]["status"], "completed")

    def test_disabled_flag(self):
        ctrl = FakeController()
        old = PrintExecutionLogger.enabled
        try:
            PrintExecutionLogger.enabled = False
            with TemporaryDirectory() as td:
                lg = PrintExecutionLogger("j", log_dir=td)
                self.assertIsNone(lg.start(ctrl, {}))
                self.assertFalse(lg.active)
                lg.log("xy_cmd", x_mm=0)  # must not raise
                lg.stop("completed")
        finally:
            PrintExecutionLogger.enabled = old

    def test_log_never_raises_after_close(self):
        ctrl = FakeController()
        with TemporaryDirectory() as td:
            lg = PrintExecutionLogger("j", log_dir=td)
            lg.start(ctrl, {})
            lg.stop("aborted")
            lg.log("xy_cmd", x_mm=1)  # post-close: silently ignored


class TestXYCmdFields(unittest.TestCase):

    def test_clamp_detection(self):
        ctrl = FakeController(limits=FakeLimits())
        with TemporaryDirectory() as td:
            lg = PrintExecutionLogger("j", log_dir=td)
            lg.start(ctrl, {})
            # 5 mm → 5000 µm, clamped to 1000 µm by FakeLimits
            fields = lg.xy_cmd_fields(ctrl, 5.0, 2.0)
            lg.stop("completed")
        self.assertTrue(fields.get("clamped"))
        self.assertAlmostEqual(fields["clamp_dx_um"], -4000.0, places=0)
        self.assertAlmostEqual(fields["abs_x_um"], 5000.0, places=0)
        # Lag target tracks the CLAMPED destination
        self.assertAlmostEqual(lg._target_um[0], 1000.0, places=0)

    def test_no_clamp_within_limits(self):
        ctrl = FakeController(limits=FakeLimits())
        with TemporaryDirectory() as td:
            lg = PrintExecutionLogger("j", log_dir=td)
            lg.start(ctrl, {})
            fields = lg.xy_cmd_fields(ctrl, 0.5, 0.5)
            lg.stop("completed")
        self.assertNotIn("clamped", fields)

    def test_zero_offset_applied(self):
        ctrl = FakeController()
        ctrl.zero_position["x"] = 10000.0
        with TemporaryDirectory() as td:
            lg = PrintExecutionLogger("j", log_dir=td)
            lg.start(ctrl, {})
            fields = lg.xy_cmd_fields(ctrl, 1.0, 0.0)
            lg.stop("completed")
        self.assertAlmostEqual(fields["abs_x_um"], 11000.0, places=0)


class TestSampler(unittest.TestCase):

    def test_samples_with_lag(self):
        ctrl = FakeController()
        ctrl._pos = (100.0, 0.0)
        with TemporaryDirectory() as td:
            lg = PrintExecutionLogger("j", log_dir=td, sample_hz=50.0)
            path = lg.start(ctrl, {})
            lg.note_xy_target(400.0, 400.0)
            time.sleep(0.3)
            lg.stop("completed")
            events = read_events(path)
        samples = [e for e in events if e["ev"] == "sample"]
        self.assertGreater(len(samples), 0)
        s = samples[-1]
        self.assertAlmostEqual(s["x_um"], 100.0, places=0)
        # lag = hypot(400-100, 400-0) = 500
        self.assertAlmostEqual(s["lag_um"], 500.0, places=0)


class TestManifest(unittest.TestCase):

    def test_manifest_for_job(self):
        from SupportClasses.PrintManager import (
            PrintJob, PrintSettings, PrintCommand, CommandType,
        )
        circle = [(0.0, 1.0), (1.0, 0.0), (0.0, -1.0), (0.0, 1.0)]
        job = PrintJob(
            name="Manifest Test", settings=PrintSettings(num_layers=2),
            commands=[
                PrintCommand(CommandType.MOVE_XY, {"x": 3.0, "y": 4.0}),
                PrintCommand(CommandType.PRINT_PATH,
                             {"points": circle, "pump": "P2",
                              "flow_rate_uL_s": 0.5}),
                PrintCommand(CommandType.HOME_XY),
            ])
        ctrl = FakeController()
        m = PrintExecutionLogger.manifest_for_job(job, ctrl, "discrete")

        self.assertEqual(m["job"]["name"], "Manifest Test")
        self.assertEqual(m["settings"]["num_layers"], 2)
        self.assertEqual(m["zero_position"]["x"], 0.0)
        plan = m["command_plan"]
        self.assertEqual(plan[0]["type"], "move_xy")
        self.assertEqual(plan[0]["x"], 3.0)
        self.assertEqual(plan[1]["n_points"], 4)
        self.assertEqual(plan[1]["pump"], "P2")
        self.assertIn("path_len_mm", plan[1])
        self.assertIn("bbox_mm", plan[1])

    def test_path_stats(self):
        stats = PrintExecutionLogger._path_stats(
            [(0, 0), (3, 4), (3, 4)])
        self.assertAlmostEqual(stats["path_len_mm"], 5.0, places=3)
        self.assertAlmostEqual(stats["seg_mm_max"], 5.0, places=3)
        self.assertAlmostEqual(stats["seg_mm_min"], 0.0, places=3)


# ── PrintManager integration ─────────────────────────────────────────

class TestPrintManagerIntegration(unittest.TestCase):

    def _run_job(self, ctrl, job, timeout=15.0):
        import SupportClasses.PrintExecutionLogger as pel_mod
        from SupportClasses.PrintManager import PrintManager, PrintState

        td = TemporaryDirectory()
        self.addCleanup(td.cleanup)
        old_dir = pel_mod._DEF_LOG_DIR
        pel_mod._DEF_LOG_DIR = Path(td.name)
        self.addCleanup(lambda: setattr(pel_mod, "_DEF_LOG_DIR", old_dir))

        pm = PrintManager(ctrl)
        pm.load_job(job)
        pm.start()
        pm._thread.join(timeout=timeout)
        self.assertFalse(pm._thread.is_alive(), "print thread hung")
        self.assertEqual(pm.state, PrintState.COMPLETED)
        files = sorted(Path(td.name).glob("*.jsonl"))
        self.assertEqual(len(files), 1)
        return pm, read_events(files[0])

    def test_discrete_job_produces_full_log(self):
        from SupportClasses.PrintManager import (
            PrintJob, PrintSettings, PrintCommand, CommandType,
        )
        ctrl = FakeController()
        pts = [(0.0, 0.0), (0.5, 0.0), (0.5, 0.5)]
        job = PrintJob(
            name="QuickTest", settings=PrintSettings(print_speed_mm_s=5.0),
            commands=[
                PrintCommand(CommandType.MOVE_XY, {"x": 1.0, "y": 1.0},
                             label="to well"),
                PrintCommand(CommandType.PRINT_PATH,
                             {"points": pts, "pump": "P1",
                              "flow_rate": 0.01, "flow_rate_uL_s": 0.25},
                             label="print"),
                PrintCommand(CommandType.HOME_XY, label="home"),
            ])
        pm, events = self._run_job(ctrl, job)

        kinds = [e["ev"] for e in events]
        self.assertEqual(kinds[0], "job_start")
        self.assertEqual(events[-1]["ev"], "job_end")
        self.assertEqual(events[-1]["status"], "completed")
        for required in ("command_start", "command_end", "path_start",
                         "path_segment", "path_end", "settle_wait",
                         "xy_cmd", "speed_set"):
            self.assertIn(required, kinds, f"missing event: {required}")

        # Per-segment detail: 2 segments with targets + sleep budget
        segs = [e for e in events if e["ev"] == "path_segment"]
        self.assertEqual(len(segs), 2)
        self.assertAlmostEqual(segs[0]["x_mm"], 0.5, places=3)
        self.assertAlmostEqual(segs[1]["y_mm"], 0.5, places=3)
        self.assertIn("slp_s", segs[0])
        self.assertIn("drift_s", segs[0])
        self.assertIn("vol_uL", segs[0])

        # HOME_XY recorded with its context
        homes = [e for e in events
                 if e["ev"] == "xy_cmd" and e.get("context") == "home"]
        self.assertEqual(len(homes), 1)

        # Settle waits succeeded against the instantly-arriving fake
        settles = [e for e in events if e["ev"] == "settle_wait"]
        self.assertTrue(all(s["ok"] for s in settles))

        # job_start manifest carries the command plan
        self.assertEqual(len(events[0]["command_plan"]), 3)

    def test_logger_closed_after_run(self):
        from SupportClasses.PrintManager import (
            PrintJob, PrintSettings, PrintCommand, CommandType,
        )
        ctrl = FakeController()
        job = PrintJob(name="tiny", settings=PrintSettings(), commands=[
            PrintCommand(CommandType.COMMENT, label="noop")])
        pm, events = self._run_job(ctrl, job)
        self.assertFalse(pm.exec_logger.active)


class TestSettleTimeoutCapture(unittest.TestCase):

    def test_timeout_event(self):
        from SupportClasses.PrintManager import PrintManager

        class StuckController(FakeController):
            def move_xy_absolute(self, x, y, from_zero_ref=True, fast=False):
                self.xy_targets.append((x, y))  # never updates _pos

        ctrl = StuckController()
        ctrl._pos = (99999.0, 99999.0)
        with TemporaryDirectory() as td:
            pm = PrintManager(ctrl)
            pm.exec_logger = PrintExecutionLogger("stuck", log_dir=td)
            path = pm.exec_logger.start(ctrl, {})
            pm._wait_for_xy_settle(1.0, 1.0, timeout=0.2)
            pm.exec_logger.stop("test")
            events = read_events(path)
        settles = [e for e in events if e["ev"] == "settle_wait"]
        self.assertEqual(len(settles), 1)
        self.assertFalse(settles[0]["ok"])
        self.assertEqual(settles[0]["reason"], "timeout")
        self.assertIsNotNone(settles[0]["final_err_um"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
