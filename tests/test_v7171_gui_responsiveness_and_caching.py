"""v7.17.1 — three GUI-thread defects found in one operator log (2026-08-13).

All three are the same shape: work that must not sit on the Qt event loop was
sitting on it, and the only reason it was ever noticed is that v7.16's
GuiWatchdog started writing the stalls down.

1. ``camera_settings_dialog`` wired every slider on ``valueChanged`` straight
   into a full calibration-store write. One gamma drag (84 -> 128) wrote the
   file ~50 times in 9 seconds — a JSON serialise + atomic replace per mouse
   tick, on the GUI thread. Same failure as the v7.16 crop-offset spins, and
   the same remedy: live push immediate, WRITE debounced.

2. ``PrintFileManager.load`` globs the prints directory and JSON-parses every
   file to match one by ``metadata.name``. The Quick Print refreshes call it per
   tick; the session logged 302 loads over 39 prints — roughly 12,000 file opens.
   ``import_csv_trajectory`` then re-read and re-parsed the same CSV each time.

3. ``CalibrationPage._safe_navigate_to`` ran the multi-second retract-then-XY
   travel on the GUI thread — a measured 10.0 s stall, 29 across three days.

The tests that matter here are the ones that would FAIL with the fix reverted,
so each names the behaviour rather than the implementation.
"""

import os
import sys
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")


# ── 2. Load / parse caching ───────────────────────────────────────────

class TestPrintFileLoadIsCached(unittest.TestCase):
    """A repeat load must cost no file I/O, yet still hand every caller its
    own mutable object and still notice an edit on disk."""

    def setUp(self):
        import tempfile, json
        self._tmp = tempfile.TemporaryDirectory()
        self.dir = Path(self._tmp.name)
        # A few decoys, because the cost being removed is the directory scan.
        for i in range(6):
            (self.dir / f"decoy{i}.json").write_text(json.dumps({
                "schema_version": "7.2",
                "metadata": {"name": f"Decoy {i}"},
                "objects": {},
            }))
        self.path = self.dir / "target.json"
        self.path.write_text(json.dumps({
            "schema_version": "7.2",
            "metadata": {"name": "Target"},
            # Non-empty on purpose: PrintFileData.from_dict assigns
            # ``pf.objects = data["objects"]`` BY REFERENCE, so ``objects`` is
            # the only place a shared cached dict actually leaks between
            # callers. A fixture with no objects cannot detect that.
            "objects": {"dot": {"object_type": "dot", "params": {"x": 1.0}}},
        }))

    def tearDown(self):
        self._tmp.cleanup()

    def _mgr(self):
        from SupportClasses.PrintFileManager import PrintFileManager
        return PrintFileManager(self.dir)

    def _count_opens(self, fn):
        import builtins
        n = {"c": 0}
        real = builtins.open

        def counting(*a, **k):
            try:
                if str(a[0]).endswith((".json", ".csv")):
                    n["c"] += 1
            except Exception:
                pass
            return real(*a, **k)

        builtins.open = counting
        try:
            fn()
        finally:
            builtins.open = real
        return n["c"]

    def test_repeat_loads_open_no_files_at_all(self):
        m = self._mgr()
        self.assertIsNotNone(m.load("Target"))
        opens = self._count_opens(lambda: [m.load("Target") for _ in range(25)])
        self.assertEqual(
            opens, 0,
            "25 repeat loads still touched the filesystem — the cache is not "
            "being hit, which is the 12,000-file-open bug")

    def test_the_first_load_really_does_scan(self):
        """Guard the guard: if the first load were free too, the test above
        would pass even with caching removed entirely."""
        m = self._mgr()
        self.assertGreater(
            self._count_opens(lambda: m.load("Target")), 0,
            "the uncached first load opened nothing — this fixture cannot "
            "detect whether caching works")

    def test_each_caller_gets_its_own_object(self):
        """Callers mutate what they get back (the print pages edit and save),
        so a shared cached dict would leak one caller's edit into every later
        load. Mutating ``objects`` specifically, because from_dict rebuilds
        metadata but assigns objects by reference."""
        m = self._mgr()
        m.load("Target")            # populate the cache
        a = m.load("Target")        # served FROM the cache
        b = m.load("Target")        # served FROM the cache
        # Both must be cache HITS: the first, uncached load builds from its own
        # freshly parsed dict, so comparing it against a hit would pass even
        # with the copy removed.
        self.assertIsNot(a, b)
        self.assertIsNot(
            a.objects, b.objects,
            "two cached loads share one objects dict — an edit in one page "
            "would appear in every other")
        a.objects["dot"]["params"]["x"] = 999.0
        a.objects["injected"] = {"object_type": "dot", "params": {}}
        fresh = m.load("Target")
        self.assertEqual(fresh.objects["dot"]["params"]["x"], 1.0)
        self.assertNotIn("injected", fresh.objects)

    def test_the_edit_would_not_survive_a_reload_from_disk_either(self):
        """Guard the guard: proves the assertion above is about the CACHE, not
        merely restating that from_dict copies scalars."""
        m = self._mgr()
        a = m.load("Target")
        a.objects["dot"]["params"]["x"] = 999.0
        self.assertEqual(
            self._mgr().load("Target").objects["dot"]["params"]["x"], 1.0)

    def test_an_edit_on_disk_invalidates_the_cache(self):
        import json
        m = self._mgr()
        self.assertEqual(len(m.load("Target").objects), 1)
        time.sleep(0.02)
        self.path.write_text(json.dumps({
            "schema_version": "7.2",
            "metadata": {"name": "Target"},
            "objects": {"dot": {"object_type": "dot", "params": {"x": 1.0}},
                        "second": {"object_type": "dot", "params": {}}},
        }))
        self.assertEqual(
            len(m.load("Target").objects), 2,
            "a file edited on disk was served from the cache — stale forever")

    def test_a_deleted_file_is_not_served_from_cache(self):
        m = self._mgr()
        self.assertIsNotNone(m.load("Target"))
        self.path.unlink()
        self.assertIsNone(
            m.load("Target"),
            "a deleted print was still returned from the cache")


class TestCsvTrajectoryIsCached(unittest.TestCase):

    def setUp(self):
        import tempfile
        self._tmp = tempfile.TemporaryDirectory()
        self.path = Path(self._tmp.name) / "t.csv"
        self.path.write_text("0,0,0,0,0,0,0\n1,1,1,0,0,0,1\n")

    def tearDown(self):
        self._tmp.cleanup()

    def test_repeat_import_does_not_reread_the_file(self):
        import builtins
        from SupportClasses.TrajectoryPlanner import import_csv_trajectory
        import_csv_trajectory(self.path)
        n = {"c": 0}
        real = builtins.open

        def counting(*a, **k):
            n["c"] += 1
            return real(*a, **k)

        builtins.open = counting
        try:
            for _ in range(20):
                import_csv_trajectory(self.path)
        finally:
            builtins.open = real
        self.assertEqual(n["c"], 0)

    def test_the_caller_cannot_corrupt_the_cache(self):
        """A cached ndarray handed out by reference would let one consumer's
        in-place edit rewrite what every later consumer sees.

        The mutation has to be applied to the result of a CACHE HIT: the very
        first call returns the freshly parsed array (the cache stores its own
        copy), so corrupting that one proves nothing.
        """
        from SupportClasses.TrajectoryPlanner import import_csv_trajectory
        import_csv_trajectory(self.path)          # populate
        hit = import_csv_trajectory(self.path)    # served FROM the cache
        hit[0, 0] = 999.0
        self.assertEqual(
            import_csv_trajectory(self.path)[0, 0], 0.0,
            "a consumer's in-place edit rewrote the cached trajectory — every "
            "later print preview would read the corrupted path")

    def test_an_edit_on_disk_invalidates_the_cache(self):
        from SupportClasses.TrajectoryPlanner import import_csv_trajectory
        self.assertEqual(len(import_csv_trajectory(self.path)), 2)
        time.sleep(0.02)
        self.path.write_text("0,0,0,0,0,0,0\n5,5,5,0,0,0,1\n2,2,2,0,0,0,2\n")
        self.assertEqual(len(import_csv_trajectory(self.path)), 3)

    def test_an_invalid_file_is_never_cached(self):
        """Non-monotonic time raises. If the failure were cached, the raise
        would have to be re-synthesised — and a later FIXED file would still
        be reported broken."""
        from SupportClasses.TrajectoryPlanner import import_csv_trajectory
        self.path.write_text("0,0,0,0,0,0,5\n1,1,1,0,0,0,1\n")
        with self.assertRaises(ValueError):
            import_csv_trajectory(self.path)
        time.sleep(0.02)
        self.path.write_text("0,0,0,0,0,0,0\n1,1,1,0,0,0,1\n")
        self.assertEqual(len(import_csv_trajectory(self.path)), 2)


# ── 1. Camera settings debounce ───────────────────────────────────────

def _qapp():
    from PySide6.QtWidgets import QApplication
    return QApplication.instance() or QApplication([])


class _FakeManager:
    """Minimal CameraManager surface the dialog touches."""

    def __init__(self):
        self.state = {"gamma": 84, "brightness": 0, "contrast": 2,
                      "resolution": (640, 480), "auto_exposure": False,
                      "exposure_us": 15000, "exposure_gain_pct": 1}

    def hardware_capabilities(self, idx):
        return {"controllable": True, "source": "opencv",
                "resolution": False, "device_name": "Fake", "controls": {
            "gamma": {"range": [1, 255, 84]},
            "brightness": {"range": [-64, 64, 0]},
            "contrast": {"range": [0, 100, 2]},
        }}

    def get_hw_settings(self, idx):
        return dict(self.state)

    def set_hw_gamma(self, idx, v):
        self.state["gamma"] = v

    def set_hw_brightness(self, idx, v):
        self.state["brightness"] = v

    def set_hw_contrast(self, idx, v):
        self.state["contrast"] = v

    def set_hw_auto_exposure(self, idx, v):
        self.state["auto_exposure"] = v

    def log_hw_settings(self, *a, **k):
        pass

    def camera_identity(self, idx):
        return ("fake:0", "Fake Camera")


class TestCameraSettingsPersistIsDebounced(unittest.TestCase):
    """The live push must stay immediate while the WRITE coalesces."""

    def setUp(self):
        _qapp()
        import tempfile
        import SupportClasses.CameraCalibrationStore as ccs
        from gui.dialogs.camera_settings_dialog import CameraSettingsDialog

        # ⚠ Redirect the store BEFORE building the dialog. _flush_persist calls
        # the module-level get_store(), which resolves this machine's REAL
        # per-machine camera_calibrations.json — an earlier revision of this
        # test wrote a bogus "fake:0" camera into the operator's live
        # calibration file. A test must never be able to reach it.
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        tmp_store = ccs.CameraCalibrationStore(
            Path(self._tmp.name) / "camera_calibrations.json")
        real_get_store = ccs.get_store
        ccs.get_store = lambda *a, **k: tmp_store
        self.addCleanup(lambda: setattr(ccs, "get_store", real_get_store))
        self.store = tmp_store

        self.mgr = _FakeManager()
        self.writes = []
        self.dlg = CameraSettingsDialog(
            self.mgr, 0, identity_getter=lambda: ("fake:0", "Fake Camera"))
        # Intercept at the store boundary — everything above it is real code.
        self.dlg._flush_persist = self._counting_flush(self.dlg._flush_persist)

    def _counting_flush(self, real):
        def wrapper():
            pending = getattr(self.dlg, "_persist_pending", False)
            real()
            if pending:
                self.writes.append(1)
        return wrapper

    def tearDown(self):
        self.dlg.deleteLater()

    def test_a_drag_of_fifty_ticks_writes_once(self):
        """The reported bug: ~50 writes in 9 s from one gamma drag."""
        for v in range(84, 134):
            self.dlg._on_gamma_changed(v)
        self.assertEqual(
            self.writes, [],
            "a write happened DURING the drag — the store is still being "
            "hammered once per slider tick")
        self.assertTrue(self.dlg._persist_pending)
        self.dlg._flush_persist()
        self.assertEqual(len(self.writes), 1)

    def test_the_live_push_is_NOT_debounced(self):
        """Aiming a control is a visual task: the camera must follow the
        slider immediately even though the file write waits."""
        self.dlg._on_gamma_changed(120)
        self.assertEqual(
            self.mgr.state["gamma"], 120,
            "the value reached the store path but not the camera — the "
            "preview would lag the slider by the debounce interval")

    def test_a_discrete_decision_writes_immediately(self):
        self.dlg._on_auto_toggled(True)
        self.assertEqual(
            len(self.writes), 1,
            "a checkbox is one deliberate decision and must not wait")

    def test_hiding_the_dialog_flushes_a_pending_write(self):
        """Closing mid-drag must not lose the value — a setting that survives
        the session but not a restart is worse than one never applied."""
        # Qt only delivers hideEvent to a widget that was actually shown, so
        # exercising the real teardown path means showing it first.
        self.dlg.show()
        self.dlg._on_contrast_changed(40)
        self.assertTrue(self.dlg._persist_pending)
        self.dlg.hide()
        self.assertEqual(len(self.writes), 1)

    def test_flush_with_nothing_pending_writes_nothing(self):
        self.dlg._flush_persist()
        self.dlg._flush_persist()
        self.assertEqual(self.writes, [])

    def test_the_debounced_write_actually_reaches_the_store(self):
        """Guard the guard: every test above counts calls to _flush_persist,
        which proves nothing about whether the value is persisted. Deferring a
        write that never lands is worse than writing too often."""
        self.dlg._on_gamma_changed(133)
        self.assertIsNone(
            (self.store.get_hw_controls("fake:0") or {}).get("gamma"),
            "the value was written DURING the drag")
        self.dlg._flush_persist()
        self.assertEqual(
            (self.store.get_hw_controls("fake:0") or {}).get("gamma"), 133)

    def test_this_test_cannot_touch_the_real_machine_store(self):
        """An earlier revision of this suite wrote a bogus camera into the
        operator's live per-machine calibration file."""
        import SupportClasses.CameraCalibrationStore as ccs
        self.assertIs(ccs.get_store(), self.store)


# ── 3. Calibration travel no longer freezes the event loop ────────────

class TestCalibrationTravelKeepsTheEventLoopTurning(unittest.TestCase):

    def setUp(self):
        _qapp()

    def _stub(self, travel_secs=0.25, result=True, record=None):
        from PySide6.QtWidgets import QWidget

        class _Ctrl:
            def safe_travel_to(self, x, y, **kw):
                if record is not None:
                    record.append((x, y, kw))
                time.sleep(travel_secs)
                return result

        w = QWidget()
        w.controller = _Ctrl()
        return w

    def _run(self, stub, **kw):
        from gui.pages.calibration import run_safe_travel_responsive
        return run_safe_travel_responsive(
            stub, 1000.0, 2000.0, safe_z_mm=-40.0, target_z_mm=None, **kw)

    def _tick_count_during(self, fn):
        """Run `fn` and report how many QTimer ticks the event loop delivered."""
        from PySide6.QtCore import QTimer
        ticks = []
        t = QTimer()
        t.setInterval(20)
        t.timeout.connect(lambda: ticks.append(1))
        t.start()
        try:
            result = fn()
        finally:
            t.stop()
        return result, len(ticks)

    def test_timers_still_fire_while_the_stage_is_travelling(self):
        """THE fix. Every camera feed is a QTimer; if none fires during the
        travel the GUI is frozen exactly as before."""
        ok, ticks = self._tick_count_during(
            lambda: self._run(self._stub(travel_secs=0.4)))
        self.assertTrue(ok)
        self.assertGreater(
            ticks, 3,
            "no timer fired during a 0.4 s travel — the event loop was blocked")

    def test_the_PRODUCTION_entry_point_is_the_responsive_one(self):
        """_safe_navigate_to is what the twelve callers actually use.

        Asserting on the helper alone says nothing about whether the page
        still routes through it — reverting that one line would restore the
        freeze while every helper test stayed green.
        """
        from gui.pages.calibration import CalibrationPage
        stub = self._stub(travel_secs=0.4)
        stub._safe_z = -40.0

        ok, ticks = self._tick_count_during(
            lambda: CalibrationPage._safe_navigate_to(
                stub, 1000.0, 2000.0, lower_z=False))

        self.assertTrue(ok)
        self.assertGreater(
            ticks, 3,
            "_safe_navigate_to blocked the event loop — the page is calling "
            "safe_travel_to directly again, which IS the 10 s freeze")

    def test_the_boolean_contract_is_preserved(self):
        """Twelve callers branch on this; _on_park arms a measurement session
        on it."""
        self.assertTrue(self._run(self._stub(result=True)))
        self.assertFalse(self._run(self._stub(result=False)))

    def test_a_raising_travel_reports_failure_rather_than_propagating(self):
        from PySide6.QtWidgets import QWidget

        class _Ctrl:
            def safe_travel_to(self, x, y, **kw):
                raise RuntimeError("serial died mid-travel")

        w = QWidget()
        w.controller = _Ctrl()
        self.assertFalse(self._run(w))

    def test_the_move_is_passed_through_unchanged(self):
        """The travel itself must be byte-identical — only where it runs
        changed. A reordered or dropped safe_z would be a crash-down."""
        rec = []
        self._run(self._stub(travel_secs=0.01, record=rec))
        self.assertEqual(len(rec), 1)
        x, y, kw = rec[0]
        self.assertEqual((x, y), (1000.0, 2000.0))
        self.assertEqual(kw.get("safe_z_mm"), -40.0)
        self.assertIsNone(kw.get("target_z_mm"))

    def test_a_second_travel_is_refused_while_one_is_in_flight(self):
        """Spinning the event loop hands back the ability to command motion
        mid-travel, which the frozen GUI accidentally prevented."""
        stub = self._stub(travel_secs=0.01)
        stub._nav_travel_in_flight = True
        self.assertFalse(self._run(stub))

    def test_the_guard_clears_so_later_travels_still_work(self):
        stub = self._stub(travel_secs=0.01)
        self.assertTrue(self._run(stub))
        self.assertFalse(getattr(stub, "_nav_travel_in_flight", False))
        self.assertTrue(self._run(stub))

    def test_no_controller_is_refused_not_crashed(self):
        from PySide6.QtWidgets import QWidget
        w = QWidget()
        w.controller = None
        self.assertFalse(self._run(w))


if __name__ == "__main__":
    unittest.main()
