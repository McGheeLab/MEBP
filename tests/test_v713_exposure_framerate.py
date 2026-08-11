"""
test_v713_exposure_framerate.py — the Zyla FrameRate/exposure bench fixes.

Root cause reproduced and pinned: pylablib's ``set_exposure()`` pins FrameRate
at its MAXIMUM (``set_frame_period(0)``) before writing ExposureTime, so the
SDK truncates every exposure request against ``ExposureTime.max ~= 1/FrameRate``
— "exposure resets to a small number". The same pinned-at-max FrameRate
exceeds the USB3 link's MaxInterfaceTransferRate at full resolution, which
overflows the camera's internal buffer — "frames dropped / feed goes dead".

The stub below models the REAL SDK3 coupling:
  ExposureTime.max = 1/FrameRate      (live; STALE unless update_properties)
  FrameRate.max    = min(sensor_max, 1/ExposureTime)
  direct attribute writes RAISE when out of range (they never truncate)
so a backend that skips the frame-rate management cannot pass these tests.
"""

import ast
import inspect
import os
import sys
import threading
import time
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import numpy as np

from gui.widgets.andor_backend import AndorBackend


# ── FrameRate-coupled stub camera ─────────────────────────────────────

class _Attr:
    def __init__(self, lo=None, hi=None):
        self.min, self.max = lo, hi


class _FrCam:
    """pylablib-shaped stub with the Zyla's frame-rate coupling modeled."""

    EXP_MIN_S = 1e-5

    def __init__(self, sensor_max_fps=48.0, fr_min=0.1, mitr=26.0,
                 exposure_s=0.02, frame_rate=None):
        self.sensor_max_fps = float(sensor_max_fps)
        self.fr_min = float(fr_min)
        self.mitr = mitr                       # None = attribute absent
        self.exposure_s = float(exposure_s)
        self.frame_rate = float(frame_rate if frame_rate is not None
                                else sensor_max_fps)
        self.attr_calls = []                   # (name, update_properties)
        self.sets = []                         # (name, value)
        self.ops = []                          # "start" / "stop"
        self.acquiring = True
        self.closed = False
        # STALE limits — what a pylablib attribute object reports when
        # update_properties is NOT requested (cached at construction).
        self._stale = {
            "ExposureTime": _Attr(self.EXP_MIN_S, self._exp_max()),
            "FrameRate": _Attr(self.fr_min, self._fr_max()),
        }

    # the SDK3 coupling
    def _exp_max(self):
        return 1.0 / self.frame_rate

    def _fr_max(self):
        return min(self.sensor_max_fps, 1.0 / max(self.exposure_s, 1e-9))

    # attributes
    def get_attribute(self, name, update_properties=False):
        self.attr_calls.append((name, bool(update_properties)))
        if name not in ("ExposureTime", "FrameRate"):
            raise KeyError(name)
        if not update_properties:
            return self._stale[name]
        if name == "ExposureTime":
            return _Attr(self.EXP_MIN_S, self._exp_max())
        return _Attr(self.fr_min, self._fr_max())

    def get_attribute_value(self, name):
        if name == "FrameRate":
            return self.frame_rate
        if name == "ExposureTime":
            return self.exposure_s
        if name == "MaxInterfaceTransferRate":
            if self.mitr is None:
                raise KeyError(name)
            return self.mitr
        raise KeyError(name)

    def set_attribute_value(self, name, value):
        v = float(value)
        self.sets.append((name, v))
        if name == "FrameRate":
            if v > self._fr_max() * 1.0001 or v < self.fr_min * 0.9999:
                raise RuntimeError(f"OUTOFRANGE FrameRate {v}")
            self.frame_rate = v
        elif name == "ExposureTime":
            if v > self._exp_max() * 1.0001 or v < self.EXP_MIN_S * 0.9999:
                raise RuntimeError(f"OUTOFRANGE ExposureTime {v}")
            self.exposure_s = v
        else:
            raise KeyError(name)

    def set_frame_period(self, t):
        # pylablib: FrameRate = 1/period truncated into the legal range.
        t = max(float(t), 1e-9)
        self.frame_rate = min(max(1.0 / t, self.fr_min), self._fr_max())

    def get_exposure(self):
        return self.exposure_s

    def set_exposure(self, s):
        # pylablib's POISONED set_exposure — FrameRate pinned at max first,
        # then the request truncated against the now-minimal ExposureTime.max.
        self.set_frame_period(0)
        self.exposure_s = min(max(float(s), self.EXP_MIN_S), self._exp_max())

    # acquisition / lifecycle (enough for the backend paths under test)
    def start_acquisition(self, mode="sequence", nframes=10):
        self.acquiring = True
        self.ops.append("start")

    def stop_acquisition(self):
        self.acquiring = False
        self.ops.append("stop")

    def acquisition_in_progress(self):
        return self.acquiring

    def wait_for_frame(self, timeout=0.5):
        time.sleep(0.001)
        raise TimeoutError("no frame")

    def read_newest_image(self):
        return None

    def set_roi(self, *a, **k):
        pass

    def get_roi(self):
        return (0, 64, 0, 64, 1, 1)

    def close(self):
        self.closed = True


def _backend(cam) -> AndorBackend:
    be = AndorBackend()
    be._cam = cam
    return be


# ── The bug, reproduced then fixed ────────────────────────────────────

class TestExposureSticks(unittest.TestCase):
    def test_pylablib_set_exposure_reproduces_the_bug(self):
        """The stub models the poison: ask pylablib's path for 500 ms at a
        48 fps sensor max and the exposure collapses to ~one frame period."""
        cam = _FrCam()
        cam.set_exposure(0.5)
        self.assertLess(cam.exposure_s, 0.03)          # ~1/48 s, not 500 ms
        self.assertAlmostEqual(cam.frame_rate, 48.0)   # pinned at max

    def test_long_exposure_sticks(self):
        """THE FIX: 500 ms through the backend actually runs at 500 ms."""
        cam = _FrCam()
        be = _backend(cam)
        self.assertTrue(be.put_exposure_time(500_000))
        self.assertAlmostEqual(cam.exposure_s, 0.5, places=3)
        self.assertEqual(be.get_exposure_time(), 500_000)
        # FrameRate was lowered to fit — at most 1/exposure.
        self.assertLessEqual(cam.frame_rate, 1.0 / 0.5 + 1e-6)

    def test_multi_second_exposure(self):
        cam = _FrCam()
        be = _backend(cam)
        self.assertTrue(be.put_exposure_time(5_000_000))
        self.assertAlmostEqual(cam.exposure_s, 5.0, places=2)

    def test_short_exposure_keeps_fast_feed(self):
        """A short exposure after a long one raises FrameRate back up (to the
        link cap) instead of leaving the feed at 2 fps."""
        cam = _FrCam()
        be = _backend(cam)
        be.put_exposure_time(500_000)
        self.assertTrue(be.put_exposure_time(10_000))
        self.assertGreater(cam.frame_rate, 20.0)
        # ...but never above the link's sustainable rate.
        self.assertLessEqual(cam.frame_rate, cam.mitr)

    def test_never_uses_pylablib_set_exposure(self):
        """Regressing to cam.set_exposure() would re-pin FrameRate at max —
        the request would collapse and the setter would report failure."""
        cam = _FrCam()
        be = _backend(cam)
        orig = cam.set_exposure
        called = []
        cam.set_exposure = lambda s: (called.append(s), orig(s))
        self.assertTrue(be.put_exposure_time(500_000))
        self.assertEqual(called, [])

    def test_unachievable_request_returns_false_with_truth_readable(self):
        """A request beyond what any frame rate allows clamps to the maximum
        achievable, returns False, and get_exposure_time reads the truth."""
        cam = _FrCam(fr_min=0.5)          # exposure ceiling = 2 s
        be = _backend(cam)
        self.assertFalse(be.put_exposure_time(20_000_000))
        got = be.get_exposure_time()
        self.assertIsNotNone(got)
        self.assertAlmostEqual(got / 1e6, 2.0, delta=0.1)

    def test_exposure_after_long_set_survives_resync(self):
        """Raising FrameRate back (link cap) can never clamp the exposure —
        the coupled stub RAISES if the backend ever tries."""
        cam = _FrCam()
        be = _backend(cam)
        be.put_exposure_time(500_000)
        be._sync_frame_rate()
        be._sync_frame_rate()
        self.assertAlmostEqual(cam.exposure_s, 0.5, places=3)


class TestExposureRange(unittest.TestCase):
    def test_range_reports_achievable_max_not_current_frame_period(self):
        """The dialog spin's ceiling: with FrameRate at 48 fps the live
        ExposureTime.max is ~21 ms, but the ACHIEVABLE max (the setter lowers
        FrameRate) is 1/FrameRate.min = 10 s here."""
        cam = _FrCam(fr_min=0.1)
        be = _backend(cam)
        rng = be.get_exposure_time_range()
        self.assertIsNotNone(rng)
        self.assertGreaterEqual(rng[1], 9_900_000)      # ~10 s in µs

    def test_range_capped_at_30s(self):
        cam = _FrCam(fr_min=0.001)                      # 1000 s naively
        be = _backend(cam)
        rng = be.get_exposure_time_range()
        self.assertLessEqual(rng[1], 30_000_000 + 1000)

    def test_range_uses_live_limits(self):
        """update_properties=True must be requested — the stale cached max is
        the tiny one the old code served the dialog."""
        cam = _FrCam()
        be = _backend(cam)
        be.get_exposure_time_range()
        self.assertIn(("ExposureTime", True), cam.attr_calls)


# ── Link-rate capping (the 2048x2048 drops) ───────────────────────────

class TestLinkRateCap(unittest.TestCase):
    def test_sync_caps_to_link_rate(self):
        cam = _FrCam(sensor_max_fps=48.0, mitr=26.0, exposure_s=0.01,
                     frame_rate=48.0)
        be = _backend(cam)
        be._sync_frame_rate()
        self.assertLessEqual(cam.frame_rate, 26.0)
        self.assertGreater(cam.frame_rate, 20.0)

    def test_sync_without_mitr_uses_sensor_max(self):
        cam = _FrCam(mitr=None, exposure_s=0.01, frame_rate=10.0)
        be = _backend(cam)
        be._sync_frame_rate()
        self.assertAlmostEqual(cam.frame_rate, 48.0, delta=0.5)

    def test_sync_is_a_noop_without_framerate_attribute(self):
        class _NoFr(_FrCam):
            def get_attribute(self, name, update_properties=False):
                raise KeyError(name)
        be = _backend(_NoFr())
        be._sync_frame_rate()      # must not raise

    def test_resolution_change_resyncs(self):
        cam = _FrCam()
        be = _backend(cam)
        calls = []
        be._sync_frame_rate = lambda: calls.append(1)
        be._running = False
        be.set_resolution_index(3)
        self.assertEqual(len(calls), 1)

    def test_readout_rate_change_resyncs(self):
        cam = _FrCam()
        # Bolt the sensor-feature surface onto the coupled stub.
        cam.features = {"PixelReadoutRate": None}
        be = _backend(cam)
        be._features = {"andor_readout_rate": {
            "sdk_name": "PixelReadoutRate", "kind": "enum",
            "values": ["100 MHz", "216 MHz"]}}
        applied = []
        cam.set_attribute_value = lambda n, v: applied.append((n, v))
        calls = []
        be._sync_frame_rate = lambda: calls.append(1)
        be._read_clip_level = lambda: 65535
        self.assertTrue(be.set_sensor_feature("andor_readout_rate", "216 MHz"))
        self.assertEqual(len(calls), 1)

    def test_open_calls_sync_frame_rate(self):
        """AST, not substring: open() must contain a _sync_frame_rate CALL
        (the v7.10 lesson — a substring test passes on an import line)."""
        src = inspect.getsource(AndorBackend.open)
        tree = ast.parse("class _C:\n" + "\n".join(
            "    " + line for line in src.splitlines()))
        found = any(
            isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == "_sync_frame_rate"
            for node in ast.walk(tree))
        self.assertTrue(found, "open() no longer syncs the frame rate")

    def test_settings_carry_frame_rate_and_link_max(self):
        cam = _FrCam(frame_rate=25.0, mitr=26.0)
        be = _backend(cam)
        st = be.get_settings()
        self.assertAlmostEqual(st["frame_rate"], 25.0)
        self.assertAlmostEqual(st["max_interface_transfer_rate"], 26.0)


# ── Reader-loop forced re-arm (the dead feed) ─────────────────────────

class _StallCam(_FrCam):
    """wait_for_frame always fails while acquisition CLAIMS to be running —
    the wedged-USB signature the old reader could never recover from."""

    def __init__(self, exposure_s=0.03):
        super().__init__(exposure_s=exposure_s)
        self.wait_calls = 0
        self.first_rearm_at = None

    def wait_for_frame(self, timeout=0.5):
        self.wait_calls += 1
        time.sleep(0.001)
        raise TimeoutError("stalled")

    def acquisition_in_progress(self):
        return True                       # the lie

    def start_acquisition(self, mode="sequence", nframes=10):
        if self.first_rearm_at is None:
            self.first_rearm_at = self.wait_calls
        super().start_acquisition(mode=mode, nframes=nframes)


class TestReaderStallRecovery(unittest.TestCase):
    def _run_reader(self, cam, max_s=5.0, until=lambda c: c.ops):
        be = _backend(cam)
        be._running = True
        t = threading.Thread(target=be._reader_loop, daemon=True)
        t.start()
        deadline = time.monotonic() + max_s
        while time.monotonic() < deadline and not until(cam):
            time.sleep(0.02)
        be._running = False
        t.join(timeout=2.0)
        return be

    def test_forced_rearm_fires_despite_in_progress_claim(self):
        cam = _StallCam()
        self._run_reader(cam)
        self.assertIn("start", cam.ops,
                      "reader never re-armed a wedged-but-claiming stream")
        # ...and not before the stall window (threshold 21 fails; the re-arm
        # check runs every 10th fail, so the first chance is fail 30).
        self.assertGreaterEqual(cam.first_rearm_at, 21)
        self.assertLessEqual(cam.first_rearm_at, 40)

    def test_stall_window_scales_with_exposure(self):
        """A 5 s exposure legitimately yields many wait timeouts per frame —
        the threshold must widen so long exposures don't spuriously re-arm."""
        be_short = _backend(_FrCam(exposure_s=0.03))
        be_long = _backend(_FrCam(exposure_s=5.0))
        short_thr = be_short._stall_threshold_fails()
        long_thr = be_long._stall_threshold_fails()
        self.assertGreaterEqual(long_thr, short_thr * 1.8)
        self.assertGreaterEqual(short_thr, 20)

    def test_backoff_spaces_repeat_rearms(self):
        """A truly dead camera must not re-arm-spin: the second forced re-arm
        needs at least twice the fail count of the first."""
        cam = _StallCam()
        rearms = []
        orig = cam.start_acquisition

        def record(*a, **k):
            rearms.append(cam.wait_calls)
            orig(*a, **k)

        cam.start_acquisition = record
        self._run_reader(cam, max_s=5.0, until=lambda c: len(rearms) >= 2)
        self.assertGreaterEqual(len(rearms), 2)
        # fails reset to 0 after each re-arm; the second window is doubled.
        gap = rearms[1] - rearms[0]
        self.assertGreaterEqual(gap, 40)


# ── Dialog truth-sync ─────────────────────────────────────────────────

class TestDialogResync(unittest.TestCase):
    def test_spin_resyncs_to_achieved_value(self):
        """A clamped set must snap the spin to the CAMERA's value — the spin
        never displays an exposure the camera isn't running."""
        try:
            from gui.dialogs.camera_settings_dialog import CameraSettingsDialog
            from tests.test_v75x_andor_zyla_camera import _andor_widget
        except Exception as exc:  # pragma: no cover
            self.skipTest(f"camera stack unavailable: {exc}")
        mgr, cam = _andor_widget()
        fake = cam._andor
        # Model an SDK clamp: whatever is asked, the camera runs 30 ms.
        fake.put_exposure_time = lambda us: setattr(
            fake, "_exposure_us", min(int(us), 30_000)) or True
        dlg = CameraSettingsDialog(mgr, 0)
        dlg._on_exposure_changed(500.0)          # ask for 500 ms
        self.assertAlmostEqual(dlg._exp_spin.value(), 30.0, places=1)


if __name__ == "__main__":
    unittest.main()
