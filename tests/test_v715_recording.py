"""
test_v715_recording.py — video recording actually records.

THE BUG
-------
``CameraWidget.frame_captured`` is ``Signal(object)`` carrying a **QImage**
(RGB888), but ``_RecordingSession`` treated it as a numpy array.
``orient_array`` was contractually "never raises", so it swallowed the type
error and returned a non-image; that reached ``_open_writer``'s
``h, w = img.shape[:2]``, which raised **inside a daemon thread with no try**.
The thread died before ``self.active = False``, so the session was stranded as
permanently "recording": it never stopped at the requested duration, and a
manual stop found no writer and reported "no frames were recorded".

WHY THE v7.14 TESTS PASSED
--------------------------
``test_v714_capture_ui._FakeCam`` sets
``frame_captured = SimpleNamespace(connect=lambda f: None, …)`` — it never
delivers a frame, so the encode path was never executed at all. A stub that
agrees with its caller proves nothing; what has to be checked is the caller
agreeing with the REAL class.

So the fake here is a real ``QObject`` with a real ``Signal(object)`` that
emits a real ``QImage`` built exactly the way ``camera_widget._grab_frame``
builds it (``cv2.cvtColor(BGR→RGB)`` → ``QImage(..., Format_RGB888)``).
"""

import inspect
import os
import sys
import tempfile
import time
import unittest
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import numpy as np
from PySide6.QtCore import QObject, Signal
from PySide6.QtGui import QImage
from PySide6.QtWidgets import QApplication

_app = QApplication.instance() or QApplication(sys.argv)

import SupportClasses.CaptureContext as CC
from gui.widgets.capture_controller import CaptureController, to_bgr


# ── A camera that behaves like the real one ───────────────────────────

def _qimage_like_camera_widget(bgr):
    """Exactly ``camera_widget._grab_frame``'s construction (`:1516-1534`)."""
    import cv2
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    h, w, ch = rgb.shape
    return QImage(rgb.copy().data, w, h, ch * w, QImage.Format.Format_RGB888)


class _QImageCam(QObject):
    """The production contract: a real Signal delivering a real QImage."""

    frame_captured = Signal(object)

    # Big enough that a few seconds of mp4v clears EncodedVideoWriter's
    # MIN_USABLE_BYTES floor — a 64x48 clip encodes to under a kilobyte and
    # would be (correctly) reported as an empty file.
    def __init__(self, w=320, h=240, bgr_colour=(0, 0, 200)):
        super().__init__()
        self.bgr = np.zeros((h, w, 3), np.uint8)
        self.bgr[:, :] = bgr_colour
        # A little structure so the encoder is not compressing a flat field.
        self.bgr[: h // 2, : w // 2] = (255, 255, 255)
        self._q = _qimage_like_camera_widget(self.bgr)

    def emit_frame(self):
        self.frame_captured.emit(self._q)

    def get_current_frame(self):
        return self.bgr


class _Mgr:
    def __init__(self, cam):
        self.cameras = [cam]

    def camera_identity(self, i):
        return ("andor:SN-TEST", "Zyla")

    def get_hw_settings(self, i):
        return {"exposure_us": 1000.0, "resolution": (320, 240),
                "resolutions": [(320, 240)], "raw_clip_level": 65535}

    def effective_um_per_px(self, i, w):
        return 1.0

    def get_um_per_px(self, i):
        return 1.0


class _Base(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp())
        CC.reset_for_tests()
        os.environ.pop("MEBP_CAPTURE_DIR", None)
        self.cam = _QImageCam()
        self.mgr = _Mgr(self.cam)

    def tearDown(self):
        CC.reset_for_tests()

    def _ctrl(self, **over):
        from SupportClasses.CaptureSpec import merged_settings
        c = CaptureController(self.mgr, 0)
        cfg = {"output_dir": str(self.tmp), "subfolder_by_date": False,
               "video_template": "rec", "video_fps": 15.0,
               "video_max_seconds": 0, "video_max_gb": 0}
        cfg.update(over)
        c.settings = lambda: merged_settings(cfg)
        return c

    def _pump(self, n=6, delay=0.02):
        """Deliver n frames and let the encoder thread consume them."""
        for _ in range(n):
            self.cam.emit_frame()
            _app.processEvents()
            time.sleep(delay)

    def _wait(self, pred, timeout=5.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            _app.processEvents()
            if pred():
                return True
            time.sleep(0.01)
        return False


# ── The conversion at the boundary ────────────────────────────────────

class TestToBgr(unittest.TestCase):

    def test_qimage_round_trips_bit_exact_and_in_bgr(self):
        """Colour is the silent half of this bug: the QImage is RGB and cv2
        wants BGR, so a type-only fix would have written swapped video."""
        bgr = np.zeros((12, 16, 3), np.uint8)
        bgr[:, :, 2] = 200                    # pure red in BGR
        bgr[3:6, 4:9] = (10, 220, 30)         # some structure
        out = to_bgr(_qimage_like_camera_widget(bgr))
        self.assertTrue(np.array_equal(out, bgr),
                        "QImage did not round-trip back to the original BGR")
        self.assertEqual(out[0, 0].tolist(), [0, 0, 200])

    def test_padded_row_stride_is_handled(self):
        """Qt pads rows to a 4-byte boundary. Reshaping to (h, w, 3) without
        honouring bytesPerLine SHEARS the image."""
        for w in (5, 7, 13, 65, 99):
            bgr = np.random.default_rng(w).integers(
                0, 255, (9, w, 3), dtype=np.uint8)
            out = to_bgr(_qimage_like_camera_widget(bgr))
            self.assertEqual(out.shape, (9, w, 3))
            self.assertTrue(np.array_equal(out, bgr), f"sheared at width {w}")

    def test_ndarray_passes_through(self):
        bgr = np.full((4, 6, 3), 9, np.uint8)
        self.assertTrue(np.array_equal(to_bgr(bgr), bgr))

    def test_mono_is_expanded(self):
        self.assertEqual(to_bgr(np.full((4, 6), 7, np.uint8)).shape, (4, 6, 3))

    def test_uninterpretable_input_returns_none_not_garbage(self):
        for bad in (None, "frame", 42, np.zeros(5), object()):
            self.assertIsNone(to_bgr(bad))

    def test_result_is_contiguous(self):
        """cv2.VideoWriter.write on a non-contiguous view is a corruption
        source; the reversed channel slice must be materialised."""
        out = to_bgr(_qimage_like_camera_widget(
            np.full((8, 10, 3), 3, np.uint8)))
        self.assertTrue(out.flags["C_CONTIGUOUS"])


# ── Recording end to end ──────────────────────────────────────────────

class TestRecordingProducesAFile(_Base):

    def test_a_real_qimage_feed_produces_a_playable_file(self):
        """THE REGRESSION. Before the fix this wrote nothing and reported
        'no frames were recorded'."""
        c = self._ctrl()
        results = []
        c.captured.connect(lambda p, n: results.append((p, n)))
        c.failed.connect(lambda m: results.append(("FAILED", m)))
        self.assertTrue(c.start_recording())
        self._pump(20, delay=0.03)
        self.assertTrue(self._wait(lambda: c._rec.frames > 0),
                        "no frames reached the encoder")
        c.stop_recording()
        self.assertTrue(self._wait(lambda: results))
        kind, note = results[0]
        self.assertNotEqual(kind, "FAILED", note)
        path = Path(kind)
        self.assertTrue(path.exists(), "no file on disk")
        self.assertGreater(path.stat().st_size, 1024)

    def test_colour_survives_to_the_encoded_frames(self):
        """A red scene must not come back blue. Checked at the writer, since
        decoding the container back is a codec question, not ours."""
        written = []
        c = self._ctrl()
        self.assertTrue(c.start_recording())
        rec = c._rec
        real_write = rec._write

        def spy(img, t_mono):
            written.append(img.copy())
            return real_write(img, t_mono)

        rec._write = spy
        self._pump(4)
        self.assertTrue(self._wait(lambda: written))
        c.stop_recording()
        # bottom-right quadrant is the flat colour, BGR (0, 0, 200)
        px = written[0][-1, -1].tolist()
        self.assertEqual(px, [0, 0, 200], f"colour swapped: {px}")

    def test_frames_are_counted(self):
        c = self._ctrl()
        c.start_recording()
        self._pump(6)
        self.assertTrue(self._wait(lambda: c._rec.frames >= 1))
        c.stop_recording()


class TestSessionAlwaysEnds(_Base):
    """The stranding is what made the bug unrecoverable — worse than the
    failure itself, because the operator could not stop it."""

    def test_stops_itself_at_the_duration_limit(self):
        c = self._ctrl(video_max_seconds=1)
        states = []
        c.record_state.connect(lambda a, e, f: states.append(a))
        c.start_recording()
        rec = c._rec
        self._pump(2)
        # Age the session past the limit rather than sleeping through it.
        rec.started_at = time.monotonic() - 99.0
        self._pump(3)
        self.assertTrue(self._wait(lambda: not rec.active),
                        "the session ignored video_max_seconds")
        self.assertIn("limit", rec._reason)
        self.assertTrue(self._wait(lambda: c._rec is None),
                        "the controller never finalized the ended session")

    def test_stops_itself_at_the_size_limit(self):
        """video_max_gb was defined, shown in the dialog, and never read."""
        c = self._ctrl(video_max_gb=1e-9)      # ~1 byte
        c.start_recording()
        rec = c._rec
        self._pump(8)
        self.assertTrue(self._wait(lambda: not rec.active),
                        "the session ignored video_max_gb")
        self.assertIn("GB", rec._reason)

    def test_an_exception_in_the_encoder_ends_the_session(self):
        """Not stranded, and it says why. This is the exact shape of the
        original failure."""
        c = self._ctrl()
        c.start_recording()
        rec = c._rec

        def boom(img, t_mono):
            raise RuntimeError("synthetic encoder failure")

        rec._write = boom
        self._pump(3)
        self.assertTrue(self._wait(lambda: not rec.active),
                        "an exception left the session marked as recording")
        self.assertIn("synthetic", rec._reason)

    def test_stop_reports_a_reason_when_nothing_was_written(self):
        c = self._ctrl()
        msgs = []
        c.failed.connect(msgs.append)
        c.start_recording()
        c.stop_recording()          # no frames delivered at all
        self.assertTrue(self._wait(lambda: msgs))
        self.assertIn("failed", msgs[0].lower())

    def test_is_recording_goes_false_after_a_self_stop(self):
        c = self._ctrl(video_max_seconds=1)
        c.start_recording()
        c._rec.started_at = time.monotonic() - 99.0
        self._pump(3)
        self.assertTrue(self._wait(lambda: not c.is_recording),
                        "the button would still read as recording")


class TestValidationIsPerKind(_Base):

    def test_recording_refuses_impossible_settings(self):
        c = self._ctrl(video_fps=0)
        msgs = []
        c.failed.connect(msgs.append)
        self.assertFalse(c.start_recording())
        self.assertTrue(msgs)
        self.assertIn("frame rate", msgs[0].lower())

    def test_a_video_problem_does_not_block_a_photograph(self):
        """It used to: _do_still called the COMBINED validate, so a zero
        frame rate refused to take a picture."""
        from SupportClasses.CaptureSpec import validate_still, validate_video
        cfg = {"still_source": "display", "still_format": "png",
               "video_fps": 0}
        self.assertEqual(validate_still(cfg), [])
        self.assertTrue(validate_video(cfg))

    def test_still_problems_still_caught(self):
        from SupportClasses.CaptureSpec import validate_still
        self.assertTrue(validate_still(
            {"still_source": "raw", "still_format": "png"}))

    def test_do_still_uses_the_still_only_validator(self):
        """AST: a regression here silently re-couples the two paths."""
        import ast
        import inspect
        import gui.widgets.capture_controller as cc
        fn = [n for n in ast.walk(ast.parse(inspect.getsource(cc)))
              if isinstance(n, ast.FunctionDef) and n.name == "_do_still"][0]
        names = {n.id for n in ast.walk(fn) if isinstance(n, ast.Name)}
        self.assertIn("validate_still", names)
        self.assertNotIn("validate", names - {"validate_still"})


class TestWriterFactoryStaysStatic(unittest.TestCase):
    """Guards a cross-suite isolation bug these tests exposed.

    A v7.14 test patched ``EncodedVideoWriter._make_writer`` and restored it
    with a bare assignment. Reading the attribute unwraps the ``staticmethod``
    to a plain function, so restoring it that way made it an INSTANCE method —
    ``self._make_writer`` then bound, every call got an extra ``self``, the
    TypeError was swallowed by ``open()``'s except, and every recording for
    the rest of the process reported "no available video encoder".

    Silent, and only visible when suites run together — so it is pinned here
    rather than left to be rediscovered.
    """

    def test_make_writer_is_a_staticmethod(self):
        from SupportClasses.CaptureVideoWriter import EncodedVideoWriter
        self.assertIsInstance(
            inspect.getattr_static(EncodedVideoWriter, "_make_writer"),
            staticmethod,
            "_make_writer is no longer a staticmethod — a test almost "
            "certainly restored it with a bare assignment")

    def test_a_fresh_writer_can_open(self):
        """The observable consequence, independent of how it broke."""
        import tempfile
        from pathlib import Path
        from SupportClasses.CaptureVideoWriter import EncodedVideoWriter
        w = EncodedVideoWriter(Path(tempfile.mkdtemp()) / "probe.mp4",
                               15.0, (320, 240))
        self.assertTrue(w.open(), "no codec available for a plain writer")
        w.close()


class TestOrientationRefusesNonImages(unittest.TestCase):
    """The defensive catch moved the fault away from its cause; it now stops
    at the boundary where the type is known."""

    def test_qimage_is_refused(self):
        from SupportClasses.CaptureOrientation import orient_array
        q = _qimage_like_camera_widget(np.zeros((4, 6, 3), np.uint8))
        self.assertIsNone(orient_array(q, mirrored=False, flip_y=False,
                                       rotation_deg=0.0))

    def test_low_dimensional_input_is_refused(self):
        from SupportClasses.CaptureOrientation import orient_array
        self.assertIsNone(orient_array(np.zeros(5)))
        self.assertIsNone(orient_array(np.array(3)))

    def test_real_images_still_orient(self):
        from SupportClasses.CaptureOrientation import orient_array
        a = np.arange(24, dtype=np.uint8).reshape(4, 6)
        self.assertEqual(orient_array(a, rotation_deg=90.0).shape, (6, 4))
        self.assertTrue(np.array_equal(orient_array(a), a))

    def test_none_stays_none(self):
        from SupportClasses.CaptureOrientation import orient_array
        self.assertIsNone(orient_array(None))


if __name__ == "__main__":
    unittest.main()
