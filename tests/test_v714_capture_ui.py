"""
test_v714_capture_ui.py — the capture buttons, controller and settings dialog.

Offscreen Qt, no camera and no codec: the controller is driven against a fake
manager, and the recording path uses an injected writer.
"""

import ast
import inspect
import os
import sys
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import numpy as np
from PySide6.QtWidgets import QApplication

_app = QApplication.instance() or QApplication(sys.argv)

from gui.widgets.camera_feed_view import CameraFeedView
from gui.widgets.camera_manager import CameraManager
import SupportClasses.CaptureContext as CC


class _FakeCam:
    """Duck-typed CameraWidget for the capture paths."""

    def __init__(self, frame=None, raw=None):
        self._frame = frame if frame is not None else \
            np.full((6, 8, 3), 40, np.uint8)
        self._raw = raw
        self.frame_captured = SimpleNamespace(connect=lambda f: None,
                                              disconnect=lambda f: None)

    def capture_fresh_frame(self, discard_n_frames=0, settle_ms=0):
        return self._frame

    def get_current_frame(self):
        return self._frame


class _FakeMgr:
    def __init__(self, cam=None, raw=None, resolutions=None, current=None):
        self._cam = cam or _FakeCam()
        self._raw = raw
        self.cameras = [self._cam]
        self.res = list(resolutions or [(1024, 1024), (2048, 2048)])
        self.current = tuple(current or (1024, 1024))
        self.set_calls = []

    def camera_identity(self, i):
        return ("andor:SN-TEST", "Zyla")

    def get_hw_settings(self, i):
        return {"exposure_us": 340000.0, "resolution": self.current,
                "resolutions": list(self.res), "raw_clip_level": 65535}

    def effective_um_per_px(self, i, w):
        return 1.30 if w <= 1024 else 0.65

    def get_um_per_px(self, i):
        return 1.30

    def capture_raw_average(self, i, n, timeout_s=10.0):
        return self._raw

    def set_capture_resolution(self, i, w, h):
        self.set_calls.append((w, h))
        self.current = (int(w), int(h))
        return self.current


def _controller(mgr, tmpdir, **over):
    from gui.widgets.capture_controller import CaptureController
    c = CaptureController(mgr, 0)
    cfg = {"output_dir": str(tmpdir), "subfolder_by_date": False,
           "still_template": "{camera}_{kind}", "embed_metadata": True,
           "write_sidecar": True}
    cfg.update(over)
    from SupportClasses.CaptureSpec import merged_settings
    c.settings = lambda: merged_settings(cfg)
    return c


class _Capture(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp())
        CC.reset_for_tests()
        os.environ.pop("MEBP_CAPTURE_DIR", None)

    def tearDown(self):
        CC.reset_for_tests()

    def _wait(self, pred, timeout=5.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            _app.processEvents()
            if pred():
                return True
            time.sleep(0.01)
        return False


# ── Overlay buttons ───────────────────────────────────────────────────

class TestOverlayButtons(unittest.TestCase):
    def _view(self, **kw):
        v = CameraFeedView(camera_manager=CameraManager(max_cameras=1),
                           cam_idx=0, **kw)
        v.resize(400, 300)
        return v

    def test_buttons_exist_by_default(self):
        v = self._view()
        self.assertIsNotNone(v._capture_btn)
        self.assertIsNotNone(v._record_btn)

    def test_opt_out_creates_none(self):
        v = self._view(enable_capture=False)
        self.assertIsNone(v._capture_btn)
        self.assertIsNone(v._record_btn)

    # v7.15: the toolbar is HOVER-REVEALED, so a button's visibility is owned
    # by _position_settings_btn (hover state + availability + room), not by
    # calling show() on it. These tests set the state instead.
    # ⚠ isHidden(), not isVisible(): isVisible() is False while any ancestor
    # is unshown, which would make every assertion below pass vacuously.
    @staticmethod
    def _reveal(v, capture=True, settings=True):
        v._hovering = True
        v._capture_available = capture
        v._settings_available = settings
        v._position_settings_btn()

    @staticmethod
    def _shown(btn):
        return btn is not None and not btn.isHidden()

    def test_hidden_until_a_frame_arrives(self):
        v = self._view()
        v.show()
        v._hovering = True
        v._position_settings_btn()
        self.assertFalse(self._shown(v._capture_btn),
                         "capture offered before the camera delivered anything")
        v._frame_count = 1
        v._update_capture_visibility()
        self.assertTrue(self._shown(v._capture_btn))

    def test_gear_position_unchanged_when_capture_buttons_hidden(self):
        """Regression pin: adding buttons must not move the gear on the ~18
        feeds that predate them."""
        a = self._view(enable_capture=False)
        a.show()
        self._reveal(a, capture=False)
        b = self._view()
        b.show()
        self._reveal(b, capture=False)      # capture buttons unavailable
        self.assertTrue(self._shown(a._settings_btn),
                        "the gear is hidden — this comparison would be vacuous")
        self.assertEqual(a._settings_btn.x(), b._settings_btn.x())
        self.assertEqual(a._settings_btn.y(), b._settings_btn.y())

    def test_buttons_do_not_overlap(self):
        v = self._view()
        v.show()
        self._reveal(v)
        shown = [b for b in (v._settings_btn, v._capture_btn, v._record_btn)
                 if self._shown(b)]
        self.assertEqual(len(shown), 3, "not all three are on screen")
        xs = sorted((b.x(), b.x() + b.width()) for b in shown)
        for (_l1, r1), (l2, _r2) in zip(xs, xs[1:]):
            self.assertLessEqual(r1, l2, "overlay buttons overlap")

    def test_record_button_shows_elapsed_and_reflows(self):
        v = self._view()
        v.show()
        self._reveal(v)
        idle_x = v._capture_btn.x()
        v._on_record_state(True, 67.0, 120)
        self.assertIn("1:07", v._record_btn.text())
        self.assertNotEqual(v._capture_btn.x(), idle_x)   # row re-flowed
        v._on_record_state(False, 0.0, 0)
        self.assertEqual(v._record_btn.text(), "⏺")
        self.assertEqual(v._capture_btn.x(), idle_x)


# ── Still capture ─────────────────────────────────────────────────────

class TestStillCapture(_Capture):
    def test_display_capture_writes_image_and_sidecar(self):
        mgr = _FakeMgr()
        ctrl = _controller(mgr, self.tmp)
        got = []
        ctrl.captured.connect(lambda p, n: got.append((p, n)))
        self.assertTrue(ctrl.capture_still())
        self.assertTrue(self._wait(lambda: got), "capture never completed")
        path = Path(got[0][0])
        self.assertTrue(path.exists())
        self.assertTrue(path.with_suffix(path.suffix + ".json").exists())
        from SupportClasses.CaptureImageWriter import read_embedded_metadata
        meta = read_embedded_metadata(path)
        self.assertEqual(meta["source_mode"], "display")
        self.assertEqual(meta["camera_name"], "Zyla")

    def test_second_click_while_busy_is_refused(self):
        ctrl = _controller(_FakeMgr(), self.tmp)
        ctrl._busy = True
        self.assertFalse(ctrl.capture_still())

    def test_raw_unavailable_refuses_and_writes_nothing(self):
        """A raw request must never be silently served a display frame — an
        auto-scaled 8-bit frame is not the same measurement."""
        mgr = _FakeMgr(raw=None)
        ctrl = _controller(mgr, self.tmp, still_source="raw",
                           still_format="tiff")
        fails = []
        ctrl.failed.connect(fails.append)
        ctrl.capture_still()
        self.assertTrue(self._wait(lambda: fails))
        self.assertIn("raw", fails[0].lower())
        self.assertEqual(list(self.tmp.glob("*")), [])

    def test_raw_capture_writes_16bit_tiff(self):
        raw = (np.arange(48, dtype=np.uint16).reshape(6, 8) * 1000)
        ctrl = _controller(_FakeMgr(raw=raw), self.tmp, still_source="raw",
                           still_format="tiff", still_raw_avg_frames=4)
        got = []
        ctrl.captured.connect(lambda p, n: got.append((p, n)))
        ctrl.capture_still()
        self.assertTrue(self._wait(lambda: got))
        path = Path(got[0][0])
        self.assertEqual(path.suffix, ".tif")
        from PIL import Image
        with Image.open(str(path)) as im:
            self.assertEqual(np.array(im).dtype, np.uint16)
        from SupportClasses.CaptureImageWriter import read_embedded_metadata
        self.assertEqual(read_embedded_metadata(path)["source_mode"],
                         "raw(avg 4)")

    def test_full_res_switches_and_ALWAYS_restores(self):
        """Restore lives in a finally: a raw capture that raises must not
        strand the camera at full resolution."""
        raw = np.zeros((6, 8), np.uint16)
        mgr = _FakeMgr(raw=raw)
        ctrl = _controller(mgr, self.tmp, still_source="raw",
                           still_format="tiff", still_full_res=True)
        boom = []
        mgr.capture_raw_average = lambda i, n, timeout_s=10.0: (
            boom.append(1) or (_ for _ in ()).throw(RuntimeError("sdk died")))
        fails = []
        ctrl.failed.connect(fails.append)
        ctrl.capture_still()
        self.assertTrue(self._wait(lambda: fails))
        self.assertEqual(mgr.set_calls, [(2048, 2048), (1024, 1024)])
        self.assertEqual(mgr.current, (1024, 1024))

    def test_context_provider_reaches_filename_and_metadata(self):
        ctrl = _controller(_FakeMgr(), self.tmp,
                           still_template="{channel}_{well}")
        ctrl.set_context_provider(lambda: {"channel": "FITC", "well": "A1"})
        got = []
        ctrl.captured.connect(lambda p, n: got.append(p))
        ctrl.capture_still()
        self.assertTrue(self._wait(lambda: got))
        path = Path(got[0])
        self.assertEqual(path.stem, "FITC_A1")
        from SupportClasses.CaptureImageWriter import read_embedded_metadata
        self.assertEqual(read_embedded_metadata(path)["channel"], "FITC")

    def test_orientation_provider_is_applied(self):
        """The saved 'display' image must match the screen, not the raw
        sensor read."""
        frame = np.zeros((6, 8, 3), np.uint8)
        frame[0, 0] = (255, 255, 255)
        ctrl = _controller(_FakeMgr(cam=_FakeCam(frame=frame)), self.tmp)
        ctrl.set_orientation_provider(lambda: (180.0, False, False))
        got = []
        ctrl.captured.connect(lambda p, n: got.append(p))
        ctrl.capture_still()
        self.assertTrue(self._wait(lambda: got))
        from PIL import Image
        with Image.open(got[0]) as im:
            arr = np.array(im)
        self.assertEqual(tuple(arr[-1, -1]), (255, 255, 255))  # corner moved


# ── Recording ─────────────────────────────────────────────────────────

class TestRecording(_Capture):
    def _session(self, **over):
        ctrl = _controller(_FakeMgr(), self.tmp,
                           video_template="{kind}", **over)
        return ctrl

    def test_start_stop_writes_a_file(self):
        from SupportClasses.CaptureVideoWriter import EncodedVideoWriter
        ctrl = self._session()
        self.assertTrue(ctrl.start_recording())
        rec = ctrl._rec
        # Feed frames through the real ingress → encoder path with a stub
        # cv2 writer so no codec is needed.
        written = {"n": 0}

        class _W:
            def isOpened(self): return True
            def write(self, f): written["n"] += 1
            def release(self): pass
            def set(self, p, v): return True
            def get(self, p): return 80.0

        orig = EncodedVideoWriter._make_writer
        EncodedVideoWriter._make_writer = staticmethod(
            lambda p, fc, fps, size: _W())
        try:
            for _ in range(5):
                rec._ingest(np.zeros((6, 8, 3), np.uint8))
            self.assertTrue(self._wait(lambda: written["n"] > 0))
            # Give the encoder a real file to judge.
            self.assertTrue(self._wait(lambda: rec._writer is not None))
            rec._writer.path.write_bytes(b"x" * 4096)
            ctrl.stop_recording()
        finally:
            # ⚠ staticmethod(), not a bare assignment. Reading
            # ``EncodedVideoWriter._make_writer`` unwraps the descriptor to a
            # plain function; assigning that back makes it an INSTANCE method,
            # so ``self._make_writer`` binds and every later call passes an
            # extra ``self`` → TypeError → swallowed by open()'s except →
            # "no available video encoder" for the rest of the process. That
            # silently broke every recording test that ran after this one.
            EncodedVideoWriter._make_writer = staticmethod(orig)
        self.assertFalse(ctrl.is_recording)

    def test_double_start_refused(self):
        ctrl = self._session()
        self.assertTrue(ctrl.start_recording())
        self.assertFalse(ctrl.start_recording())
        ctrl.stop_recording()

    def test_ingest_never_blocks_when_the_queue_is_full(self):
        """The ingress runs on the GUI thread — a full queue must drop and
        count, never wait for the encoder."""
        ctrl = self._session(video_fps=1.0)
        ctrl.start_recording()
        rec = ctrl._rec
        rec._stop.set()                 # stall the encoder
        time.sleep(0.05)
        start = time.monotonic()
        for _ in range(200):
            rec._ingest(np.zeros((6, 8, 3), np.uint8))
        self.assertLess(time.monotonic() - start, 1.0)
        self.assertGreater(rec.dropped, 0)
        ctrl.stop_recording()

    def test_shutdown_finalizes_a_live_recording(self):
        ctrl = self._session()
        CC.register_recorder(ctrl)
        ctrl.start_recording()
        self.assertEqual(CC.finalize_all_recordings(), 1)
        self.assertFalse(ctrl.is_recording)

    def test_camera_manager_shutdown_calls_the_finalizer(self):
        """AST, not a substring: the call must actually be there, and it must
        precede stop_all() — a recorder outliving its camera truncates."""
        src = inspect.getsource(CameraManager.shutdown)
        tree = ast.parse("class _C:\n" + "\n".join(
            "    " + ln for ln in src.splitlines()))
        fn = tree.body[0].body[0]

        def calls_in(node):
            return {c.func.id if isinstance(c.func, ast.Name)
                    else getattr(c.func, "attr", "")
                    for c in ast.walk(node) if isinstance(c, ast.Call)}

        # SOURCE order (ast.walk is breadth-first, not source order).
        order = [(i, calls_in(stmt)) for i, stmt in enumerate(fn.body)]
        fin = [i for i, names in order if "finalize_all_recordings" in names]
        stop = [i for i, names in order if "stop_all" in names]
        self.assertTrue(fin, "shutdown no longer finalizes recordings")
        self.assertTrue(stop)
        self.assertLess(min(fin), min(stop),
                        "recordings must be finalized BEFORE the cameras stop")


# ── Settings dialog ───────────────────────────────────────────────────

class TestCaptureSettingsDialog(unittest.TestCase):
    """v7.15: the single combined dialog was SPLIT into an image one and a
    video one, on the operator's report that sharing them was confusing. The
    widget names this class asserted on (``_still_fmt``, ``_video_note``,
    ``_fps``) moved with their groups.

    Its coverage now lives in ``tests/test_v715_settings_split.py``, which is
    stricter — a full round-trip of every key each dialog owns, rather than
    four; per-kind previews and validation; and the load-bearing case this
    class never had, that editing one kind must not destroy the other's
    settings (``Settings.set_section`` replaces a whole section).

    Kept as a pointer so the history of the move is discoverable from here.
    """

    def test_coverage_moved_to_the_v715_suite(self):
        import importlib
        mod = importlib.import_module("tests.test_v715_settings_split")
        for name in ("TestSeparation", "TestPersistence", "TestSpecHelpers"):
            self.assertTrue(hasattr(mod, name),
                            f"{name} missing — the split dialogs lost coverage")

    def test_both_dialogs_exist_and_are_distinct(self):
        from gui.dialogs.capture_settings_dialog import (
            ImageCaptureSettingsDialog, VideoRecordingSettingsDialog)
        self.assertIsNot(ImageCaptureSettingsDialog,
                         VideoRecordingSettingsDialog)


# ── Structural guards ─────────────────────────────────────────────────

class TestStructure(unittest.TestCase):
    def test_capture_modules_are_qt_free(self):
        """Pure logic must stay unit-testable without a GUI."""
        root = Path(__file__).resolve().parent.parent / "SupportClasses"
        checked = 0
        for path in root.glob("Capture*.py"):
            src = path.read_text(encoding="utf-8")
            self.assertNotIn("PySide6", src, f"{path.name} imports Qt")
            checked += 1
        self.assertGreaterEqual(checked, 5)   # a matcher finding none passes

    def test_capture_fresh_frame_handles_tucam(self):
        """Pre-existing gap: the tucam branch was missing, so this returned
        None on the Libra and every caller silently got nothing."""
        from gui.widgets import camera_widget
        src = inspect.getsource(camera_widget.CameraWidget.capture_fresh_frame)
        self.assertIn("tucam", src)


if __name__ == "__main__":
    unittest.main()
