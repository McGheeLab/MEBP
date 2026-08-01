# -*- coding: utf-8 -*-
"""Tutorial walkthrough recorder (gui/action_recorder.py) + the tape → tour
builder (tools_build_tour_from_tape.py).

Covered:

  1. The recorder is inert until armed and removes its filter on stop.
  2. A click is captured with the clicked widget's rect as % of the window —
     the number tutorial spotlights are placed from.
  3. F9 flags the MOST RECENT step rather than recording one of its own.
  4. The filter never consumes an event (recording must not alter behaviour).
  5. A capture failure cannot propagate into the app.
  6. read_tape tolerates a truncated final line (crashed session).
  7. The builder keeps only F9-marked steps by default, all with --all-steps.
  8. Icon-only controls (the collapsed nav rail is pure emoji) fall back to
     objectName, so beats don't all slug to "step".
"""
import io
import json
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout
from PySide6.QtCore import Qt, QEvent, QPoint
from PySide6.QtGui import QMouseEvent, QKeyEvent

from gui.action_recorder import ActionRecorder, read_tape, MARK_KEY

_app = QApplication.instance() or QApplication([])


class _Host(QWidget):
    """Stand-in for MainWindow: the recorder only needs a top-level widget."""

    def __init__(self):
        super().__init__()
        self.resize(1000, 500)
        lay = QVBoxLayout(self)
        self.btn = QPushButton("Connect")
        self.btn.setObjectName("btn_connect")
        self.icon_btn = QPushButton("\U0001F4D0")     # emoji-only, like the nav rail
        self.icon_btn.setObjectName("btn_calibrate")
        lay.addWidget(self.btn)
        lay.addWidget(self.icon_btn)
        self.show()
        _app.processEvents()


def _click(widget):
    pos = widget.rect().center()
    glob = widget.mapToGlobal(pos)
    for etype in (QEvent.MouseButtonPress, QEvent.MouseButtonRelease):
        _app.sendEvent(widget, QMouseEvent(etype, pos, glob, Qt.LeftButton,
                                           Qt.LeftButton, Qt.NoModifier))
    _app.processEvents()


def _press(host, key):
    _app.sendEvent(host, QKeyEvent(QEvent.KeyPress, key, Qt.NoModifier))
    _app.processEvents()


class TestRecorder(unittest.TestCase):

    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix="rec_test_")
        self.host = _Host()
        self.rec = ActionRecorder(self.host, root_dir=self.tmp)

    def tearDown(self):
        if self.rec.armed:
            self.rec.stop()
        self.host.close()

    # 1 ─────────────────────────────────────────────────────────
    def test_inert_until_armed(self):
        _click(self.host.btn)
        self.assertEqual(self.rec.step_count, 0)
        self.rec.start(capture_screens=False)
        _click(self.host.btn)
        self.assertEqual(self.rec.step_count, 1)
        self.rec.stop()
        _click(self.host.btn)
        self.assertEqual(self.rec.step_count, 1,
                         "filter must be removed on stop")

    # 2 ─────────────────────────────────────────────────────────
    def test_click_captures_rect_as_percent(self):
        self.rec.start(capture_screens=False)
        _click(self.host.btn)
        self.rec.stop()
        _h, steps = read_tape(os.path.join(self.rec.session_dir, "tape.jsonl"))
        self.assertEqual(len(steps), 1)
        rect = steps[0]["rect_pct"]
        for key in ("left", "top", "width", "height"):
            self.assertIn(key, rect)
            self.assertGreaterEqual(rect[key], 0.0)
            self.assertLessEqual(rect[key], 100.0)
        # The button is wide and short inside a 1000x500 host.
        self.assertGreater(rect["width"], 50.0)
        self.assertLess(rect["height"], 30.0)
        self.assertEqual(steps[0]["object_name"], "btn_connect")

    # 3 ─────────────────────────────────────────────────────────
    def test_mark_flags_last_step_and_adds_none(self):
        self.rec.start(capture_screens=False)
        _click(self.host.btn)
        _click(self.host.icon_btn)
        _press(self.host, MARK_KEY)
        self.assertEqual(self.rec.step_count, 2, "F9 must not add a step")
        self.rec.stop()
        _h, steps = read_tape(os.path.join(self.rec.session_dir, "tape.jsonl"))
        self.assertFalse(steps[0]["marked"])
        self.assertTrue(steps[1]["marked"], "F9 marks the most recent step")

    def test_mark_with_no_steps_is_safe(self):
        self.rec.start(capture_screens=False)
        _press(self.host, MARK_KEY)          # must not raise
        self.assertEqual(self.rec.step_count, 0)

    # 4 ─────────────────────────────────────────────────────────
    def test_filter_never_consumes_the_event(self):
        self.rec.start(capture_screens=False)
        fired = []
        self.host.btn.clicked.connect(lambda: fired.append(1))
        _click(self.host.btn)
        self.assertEqual(fired, [1],
                         "recording must not swallow the click")

    # 5 ─────────────────────────────────────────────────────────
    def test_capture_failure_does_not_propagate(self):
        self.rec.start(capture_screens=False)

        def boom(*_a, **_k):
            raise RuntimeError("grab exploded")

        self.rec._record_click = boom
        _click(self.host.btn)                # must not raise

    # 6 ─────────────────────────────────────────────────────────
    def test_read_tape_tolerates_truncated_tail(self):
        path = os.path.join(self.tmp, "tape.jsonl")
        with io.open(path, "w", encoding="utf-8") as fh:
            fh.write(json.dumps({"kind": "session", "steps": 2}) + "\n")
            fh.write(json.dumps({"i": 1, "kind": "click"}) + "\n")
            fh.write('{"i": 2, "kind": "cli')      # crashed mid-write
        header, steps = read_tape(path)
        self.assertEqual(header["steps"], 2)
        self.assertEqual(len(steps), 1)


class TestTourBuilder(unittest.TestCase):

    def _tape(self, tmp, marked_flags):
        session = os.path.join(tmp, "20260101_000000")
        os.makedirs(session, exist_ok=True)
        path = os.path.join(session, "tape.jsonl")
        rect = {"left": 1.0, "top": 2.0, "width": 3.0, "height": 4.0}
        with io.open(path, "w", encoding="utf-8") as fh:
            fh.write(json.dumps({"kind": "session", "created": "x",
                                 "window": {"w": 1920, "h": 1080}}) + "\n")
            for i, marked in enumerate(marked_flags, start=1):
                fh.write(json.dumps({
                    "i": i, "t": float(i), "kind": "click",
                    "widget": "QPushButton",
                    "object_name": "btn_calibrate",
                    # emoji-only label, exactly like the collapsed nav rail
                    "label": "\U0001F4D0",
                    "rect_pct": rect, "page": "Jog Control",
                    "sub_page": "", "marked": marked, "screen": "",
                }) + "\n")
        return path

    def test_default_keeps_only_marked_steps(self):
        tmp = tempfile.mkdtemp(prefix="tour_test_")
        tape = self._tape(tmp, [False, True, False, True])
        out = os.path.join(tmp, "tour")
        import tools_build_tour_from_tape as builder
        builder.main([tape, "-o", out])
        sb = io.open(os.path.join(out, "STORYBOARD.md"),
                     encoding="utf-8").read()
        spots = json.load(io.open(os.path.join(out, "spotlights.json"),
                                  encoding="utf-8"))
        self.assertEqual(len(spots["beats"]), 2)
        self.assertIn("4 step(s) captured, 2 kept", sb)
        self.assertTrue(os.path.exists(os.path.join(out, "NARRATION.md")))

    def test_all_steps_keeps_everything(self):
        tmp = tempfile.mkdtemp(prefix="tour_test_")
        tape = self._tape(tmp, [False, True, False])
        out = os.path.join(tmp, "tour")
        import tools_build_tour_from_tape as builder
        builder.main([tape, "-o", out, "--all-steps"])
        spots = json.load(io.open(os.path.join(out, "spotlights.json"),
                                  encoding="utf-8"))
        self.assertEqual(len(spots["beats"]), 3)

    def test_icon_only_control_falls_back_to_object_name(self):
        """A pure-emoji label must not collapse every beat to "step"."""
        import tools_build_tour_from_tape as builder
        step = {"label": "\U0001F4D0", "object_name": "btn_calibrate",
                "widget": "QPushButton"}
        self.assertEqual(builder._display(step), "btn_calibrate")
        self.assertEqual(builder._slug(step), "btn-calibrate")
        # A real text label still wins.
        self.assertEqual(
            builder._display({"label": "Connect", "object_name": "b"}),
            "Connect")


if __name__ == "__main__":
    unittest.main(verbosity=2)
